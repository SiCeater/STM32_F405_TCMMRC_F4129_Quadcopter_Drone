/**
 * mpu6000.c
 * Driver LL MPU6000 — SPI1 + DMA2 — STM32F405
 * Gyro and accel, fast loop 1kHz
 */
 
#include "mpu6000_driver.h"
 
 
/* ─── Instance publique ─────────────────────────────────────────────────── */
mpu6000_data_t mpu6000 = {0};
 
/* ─── Buffers DMA ───────────────────────────────────────────────────────── */
/*
 * IMPORTANT : ces buffers DOIVENT être en SRAM (section .bss / .data, adresses
 * 0x20000000+). Le DMA2 n'a PAS accès au CCM RAM (0x10000000) sur F405.
 * Les variables statiques globales sont placées en SRAM par défaut par le linker
 * script STM32CubeIDE → aucune annotation __attribute__ nécessaire ici.
 *
 * tx_buf est préparé une seule fois dans mpu6000_init() et ne change jamais.
 * rx_buf est écrasé à chaque transaction DMA.
 */
static uint8_t dma_tx_buf[MPU6000_BURST_LEN];
static uint8_t dma_rx_buf[MPU6000_BURST_LEN];
 
/* ─── Macros CS ─────────────────────────────────────────────────────────── */
#define MPU6000_CS_LOW()    LL_GPIO_ResetOutputPin(MPU6000_CS_PORT, MPU6000_CS_PIN)
#define MPU6000_CS_HIGH()   LL_GPIO_SetOutputPin(MPU6000_CS_PORT, MPU6000_CS_PIN)
 
/* ═══════════════════════════════════════════════════════════════════════════
 * Fonctions SPI polling — utilisées UNIQUEMENT pendant l'init (write/read
 * de registres de config). La fast loop n'utilise que le chemin DMA.
 * ═══════════════════════════════════════════════════════════════════════════ */
 
/**
 * @brief  Écrit un octet dans un registre MPU6000 (SPI polling, bloquant).
 * @note   On flush le RX DR après chaque octet TX pour éviter OVR.
 *         La séquence SPI Mode 3 : CPOL=1, CPHA=1 (configuré dans CubeMX).
 */
static void spi_write_reg(uint8_t reg, uint8_t val)
{
    MPU6000_CS_LOW();
 
    /* ── Octet 1 : adresse registre (bit7=0 → write) ── */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, reg & ~MPU6000_READ_FLAG);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);    /* flush — valeur sans intérêt */
 
    /* ── Octet 2 : donnée ── */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, val);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);    /* flush */
 
    /* Attendre fin de shift register avant de relâcher CS */
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    MPU6000_CS_HIGH();
}
 
/**
 * @brief  Lit un octet depuis un registre MPU6000 (SPI polling, bloquant).
 * @retval Valeur du registre.
 */
static uint8_t spi_read_reg(uint8_t reg)
{
    uint8_t val;
 
    MPU6000_CS_LOW();
 
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, reg | MPU6000_READ_FLAG);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1); // on flush le premier uniquement
 
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, 0x00);   /* dummy TX pour générer les clocks */
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    val = LL_SPI_ReceiveData8(SPI1); // cette fois ci on recupère bien la donnée
 
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    MPU6000_CS_HIGH();
 
    return val;
}
 
 
void MPU6000_Init(void)
{
    if (debug)
        print_to_console("\n\r\n\rMPU6000 : initialisation...", sizeof("\n\r\n\rMPU6000 : initialisation..."));
 
    LL_SPI_Enable(SPI1);
    
    /* ── 1. Reset matériel du MPU6000 ── */
    spi_write_reg(MPU6000_REG_PWR_MGMT_1, 0x80U);  /* DEVICE_RESET = 1 */
    LL_mDelay(150);   /* datasheet §4.28 : 100ms min après DEVICE_RESET */
 
    /* ── 2. Wake-up + sélection horloge PLL sur gyro X (plus stable que RC interne) ── */
    /*
     * PWR_MGMT_1 : SLEEP=0, CLKSEL=1 (PLL gyro X)
     * L'oscillateur interne RC (CLKSEL=0) drift en température → toujours
     * utiliser la PLL interne calée sur l'oscillateur gyro.
     */
    spi_write_reg(MPU6000_REG_PWR_MGMT_1, 0x01U);
    LL_mDelay(150);
        uint8_t cfg = spi_read_reg(MPU6000_REG_PWR_MGMT_1);
        if (cfg != 0x01U) {
            // cfg vaut probablement 0x00 → write raté
            if (debug)
                print_to_console("\n\rREG_PWR_MGMT_1 MISMATCH", sizeof("\n\rREG_PWR_MGMT_1 MISMATCH"));
            while(1);
        }
        else {
            if (debug)
                print_to_console("\n\rMPU6000 : REG_PWR_MGMT_1 OK", sizeof("\n\rMPU6000 : REG_PWR_MGMT_1 OK"));
        }
 
    /* ── 3. Vérification WHO_AM_I ── */
    if(spi_read_reg(MPU6000_REG_WHO_AM_I) != MPU6000_WHO_AM_I_VAL) {
        /* MPU6000 absent ou adresse SPI incorrecte */
        if(debug)
            print_to_console("\n\rMPU6000 : REG_WHO_AM_I FAILED", sizeof("\n\rMPU6000 : REG_WHO_AM_I FAILED"));
        while(1);  /* stopper le firmware — MPU6000 non détecté */
    }
    else {
        if(debug)
            print_to_console("\n\rMPU6000 : REG_WHO_AM_I OK", sizeof("\n\rMPU6000 : REG_WHO_AM_I OK"));
    }
 
    /* ── 4. CONFIG : DLPF_CFG = 0 ── */
    /*
    * DLPF_CFG=0 → filtre HW gyro désactivé, BW = 256Hz, Fs interne = 8kHz.
    * On garde la main sur le filtrage en SW (PT2 + notch dans fast_loop).
    * EXT_SYNC_SET = 0 (pas de sync externe).
    */
    spi_write_reg(MPU6000_REG_CONFIG, 0x00U);
    LL_mDelay(150);
    cfg = spi_read_reg(MPU6000_REG_CONFIG);
    if (cfg != 0x00U) {
        // cfg vaut probablement 0x00 → write raté
        print_to_console("\n\rREG_CONFIG MISMATCH", sizeof("\n\rREG_CONFIG MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rMPU6000 : REG_CONFIG OK", sizeof("\n\rMPU6000 : REG_CONFIG OK"));
    }
 
    /* ── 5. SMPLRT_DIV = 7 → ODR 1kHz ── */
    /*
    * Sample Rate = Gyro_Output_Rate / (1 + SMPLRT_DIV)
    * Avec DLPF_CFG=0 : Gyro_Output_Rate = 8000Hz
    * SMPLRT_DIV = 7 → 8000 / 8 = 1000 Hz  ✓
    * C'est cet ODR qui cadence l'interruption DATA_RDY → EXTI3 → fast_loop.
    */
    spi_write_reg(MPU6000_REG_SMPLRT_DIV, 0x07U);
    LL_mDelay(150);
    cfg = spi_read_reg(MPU6000_REG_SMPLRT_DIV);
    if (cfg != 0x07U) {
        // cfg vaut probablement 0x00 → write raté
        if (debug)
            print_to_console("\n\rREG_SMPLRT_DIV MISMATCH", sizeof("\n\rREG_SMPLRT_DIV MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rMPU6000 : REG_SMPLRT_DIV OK", sizeof("\n\rMPU6000 : REG_SMPLRT_DIV OK"));
    }
 
    /* ── 6. GYRO_CONFIG : FSR ±2000°/s ── */
    /*
    * FS_SEL = 3 (0x18) → ±2000°/s, sensibilité = 16.4 LSB/°/s
    * Choix justifié par la dynamique d'un quadrotor racing : les taux de
    * rotation dépassent facilement ±500°/s en acrobatie.
    */
    spi_write_reg(MPU6000_REG_GYRO_CONFIG, MPU6000_GYRO_FSR_2000_DPS);
    LL_mDelay(150);
    cfg = spi_read_reg(MPU6000_REG_GYRO_CONFIG);
    if (cfg != MPU6000_GYRO_FSR_2000_DPS) {
        if (debug)
            print_to_console("\n\rREG_GYRO_CONFIG MISMATCH", sizeof("\n\rREG_GYRO_CONFIG MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rMPU6000 : REG_GYRO_CONFIG OK", sizeof("\n\rMPU6000 : REG_GYRO_CONFIG OK"));
    }
 
    /* ── 6b. ACCEL_CONFIG : FSR ±8g ── */
    /*
     * AFS_SEL = 2 (0x10) → ±8g, sensibilité = 4096 LSB/g
     * ±2g (reset default) saturerait en acrobatie (4-6g en sortie de virage).
     */
    spi_write_reg(MPU6000_REG_ACCEL_CONFIG, MPU6000_ACCEL_FSR_8G);
    LL_mDelay(150);
    cfg = spi_read_reg(MPU6000_REG_ACCEL_CONFIG);
    if (cfg != MPU6000_ACCEL_FSR_8G) {
        if (debug)
            print_to_console("\n\rREG_ACCEL_CONFIG MISMATCH", sizeof("\n\rREG_ACCEL_CONFIG MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rMPU6000 : REG_ACCEL_CONFIG OK", sizeof("\n\rMPU6000 : REG_ACCEL_CONFIG OK"));
    }
    /* ── 7. INT_PIN_CFG : configuration de la broche INT ── */
    /*
    * Bit 7 ACTL    = 0 → INT actif HIGH  (flanc montant → EXTI3 rising ✓)
    * Bit 6 OPEN    = 0 → push-pull
    * Bit 5 LATCH_INT_EN = 1 → INT reste HIGH jusqu'à lecture INT_STATUS
    * Bit 4 INT_RD_CLEAR = 1 → le flag INT_STATUS est effacé par n'importe
    *                           quelle lecture de données (pratique en DMA :
    *                           la lecture des registres gyro clear l'IT)
    */
    spi_write_reg(MPU6000_REG_INT_PIN_CFG, 0x30U);
    LL_mDelay(150);
    cfg = spi_read_reg(MPU6000_REG_INT_PIN_CFG);
    if (cfg != 0x30U) {
        // cfg vaut probablement 0x00 → write raté
        if (debug)
            print_to_console("\n\rREG_INT_PIN_CFG MISMATCH", sizeof("\n\rREG_INT_PIN_CFG MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rMPU6000 : REG_INT_PIN_CFG OK", sizeof("\n\rMPU6000 : REG_INT_PIN_CFG OK"));
    }
    /* ── 9. USER_CTRL : désactiver I2C, garder SPI uniquement ── */
    /*
    * I2C_IF_DIS = 1 (bit 4) → désactive l'interface I2C, SPI exclusif.
    * Indispensable pour éviter les collisions si des lignes SDA/SCL sont
    * présentes sur le PCB (strapping non câblé).
    */
    spi_write_reg(MPU6000_REG_USER_CTRL, 0x10U);
    LL_mDelay(150);
    cfg = spi_read_reg(MPU6000_REG_USER_CTRL);
    if (cfg != 0x10U) {
        // cfg vaut probablement 0x00 → write raté
        if (debug)
            print_to_console("\n\rREG_USER_CTRL MISMATCH", sizeof("\n\rREG_USER_CTRL MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rMPU6000 : REG_USER_CTRL OK", sizeof("\n\rMPU6000 : REG_USER_CTRL OK"));
    }
    /* ── 10. Préparer le buffer TX DMA ── */
    /*
     * tx_buf[0] = adresse ACCEL_XOUT_H avec bit READ = 1
     * Burst consécutif : ACCEL_X/Y/Z (6) + TEMP (2, ignoré) + GYRO_X/Y/Z (6) = 14 data
     * tx_buf[1..14] = 0x00 dummy bytes
     */
    dma_tx_buf[0] = MPU6000_READ_FLAG | MPU6000_REG_ACCEL_XOUT_H;
    memset(&dma_tx_buf[1], 0x00U, MPU6000_BURST_LEN - 1U);
 
    /* ── 11. Configurer les streams DMA ── */
// Seul ajout nécessaire : lier les adresses mémoire/périph aux streams
    // (CubeMX laisse ces champs à 0, ils sont transaction-dépendants)
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, LL_SPI_DMA_GetRegAddr(SPI1));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)dma_rx_buf);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_3, LL_SPI_DMA_GetRegAddr(SPI1));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_3, (uint32_t)dma_tx_buf);
 
    /* Activer l'IT TC sur Stream0 RX — CubeMX active le NVIC mais PAS ce bit */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
 
    /* ── 8. INT_ENABLE : activer DATA_RDY interrupt ── */
    spi_write_reg(MPU6000_REG_INT_ENABLE, 0x01U);  /* DATA_RDY_EN = 1 */
    LL_mDelay(150);
    cfg = spi_read_reg(MPU6000_REG_INT_ENABLE);
    if (cfg != 0x01U) {
        // cfg vaut probablement 0x00 → write raté
        if (debug)
            print_to_console("\n\rREG_INT_ENABLE MISMATCH", sizeof("\n\rREG_INT_ENABLE MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rMPU6000 : REG_INT_ENABLE OK", sizeof("\n\rMPU6000 : REG_INT_ENABLE OK"));
    }
 
    if (debug)
        print_to_console("\n\rMPU6000 : initialized", sizeof("\n\rMPU6000 : initialized")); 
 
    /* ── 9. Configurer NVIC pour EXTI3 (INT DATA_RDY) ── */
    NVIC_EnableIRQ(EXTI3_IRQn);
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 * Fast path — DMA (appelé @ 1kHz depuis EXTI3)
 * ═══════════════════════════════════════════════════════════════════════════ */
 
/**
 * @brief  Lance une transaction DMA SPI non-bloquante de 7 octets.
 *
 * Séquence critique (ordre important) :
 *   1. Disable streams  → obligatoire pour recharger NDTR en mode Normal
 *   2. Clear TC flags   → sinon le TC de la transaction précédente peut
 *                         déclencher immédiatement le callback
 *   3. Recharger NDTR   → compteur remis à 7 pour les deux streams
 *   4. CS LOW           → chip select avant tout clock SPI
 *   5. Enable RX stream → doit être activé EN PREMIER pour être prêt à
 *                         capturer le premier octet dès que TX démarre
 *   6. Enable TX stream
 *   7. Enable DMA req SPI RX → SPI demande des données au DMA RX
 *   8. Enable DMA req SPI TX → SPI commence à shifter, clocks générés
 *
 * Le DMA2 gère ensuite les 7 octets en autonomie. L'ISR DMA2_Stream0
 * (TC RX) signalera la fin.
 */
void MPU6000_Start_DMA_Read(void)
{
    /* ── Désactiver les streams pour pouvoir recharger NDTR ── */
    LL_DMA_DisableStream(MPU6000_DMA, MPU6000_DMA_RX_STREAM);
    LL_DMA_DisableStream(MPU6000_DMA, MPU6000_DMA_TX_STREAM);
    while (LL_DMA_IsEnabledStream(MPU6000_DMA, MPU6000_DMA_RX_STREAM));
    while (LL_DMA_IsEnabledStream(MPU6000_DMA, MPU6000_DMA_TX_STREAM));
 
    /* ── Clear flags résiduels ── */
    LL_DMA_ClearFlag_TC0(MPU6000_DMA);
    LL_DMA_ClearFlag_TE0(MPU6000_DMA);
    LL_DMA_ClearFlag_TC3(MPU6000_DMA);
    LL_DMA_ClearFlag_TE3(MPU6000_DMA);
 
    /* ── Recharger NDTR (registre remis à 0 après chaque transaction en mode Normal) ── */
    LL_DMA_SetDataLength(MPU6000_DMA, MPU6000_DMA_RX_STREAM, MPU6000_BURST_LEN);
    LL_DMA_SetDataLength(MPU6000_DMA, MPU6000_DMA_TX_STREAM, MPU6000_BURST_LEN);
 
    /* ── CS bas → chip sélectionné ── */
    MPU6000_CS_LOW();
 
    /* ── Activer RX en premier, puis TX ── */
    LL_DMA_EnableStream(MPU6000_DMA, MPU6000_DMA_RX_STREAM);
    LL_DMA_EnableStream(MPU6000_DMA, MPU6000_DMA_TX_STREAM);
 
    /* ── Activer les requêtes DMA côté SPI (lance le transfert) ── */
    LL_SPI_EnableDMAReq_RX(SPI1);
    LL_SPI_EnableDMAReq_TX(SPI1);
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 * Callbacks ISR
 * ═══════════════════════════════════════════════════════════════════════════ */
 
/**
 * @brief  Fin de transfert DMA RX — TC DMA2 Stream0.
 *
 * Layout dma_rx_buf (burst depuis ACCEL_XOUT_H 0x3B) :
 *   [0]          dummy
 *   [1][2]       ACCEL_X  → raw_ax
 *   [3][4]       ACCEL_Y  → raw_ay
 *   [5][6]       ACCEL_Z  → raw_az
 *   [7][8]       TEMP     → ignoré
 *   [9][10]      GYRO_X   → raw_gx
 *   [11][12]     GYRO_Y   → raw_gy
 *   [13][14]     GYRO_Z   → raw_gz
 */
void MPU6000_DMA_RX_Complete_Callback(void)
{
    /* ══════════════════════════════════════════════════════════════════════════
    * CODE DE DÉMONSTRATION SIMPLE — À intégrer dans MPU6000_DMA_RX_Complete_Callback()
    * Filtre complémentaire + commande P basique + détection décrochage
    * ══════════════════════════════════════════════════════════════════════════ */

    // Variables statiques (persistent entre appels @ 1kHz)
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // Quaternion [w, x, y, z]
    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // Erreur intégrale
    static float roll_angle = 0.0f;   // rad
    static float pitch_angle = 0.0f;  // rad
    static bool motors_armed = true;

    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_SPI_DisableDMAReq_RX(SPI1);
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    MPU6000_CS_HIGH();
 
    /* ── Parser accel [1..6] ── */
    mpu6000.raw_ax = (int16_t)((uint16_t)(dma_rx_buf[1])  << 8 | dma_rx_buf[2]);
    mpu6000.raw_ay = (int16_t)((uint16_t)(dma_rx_buf[3])  << 8 | dma_rx_buf[4]);
    mpu6000.raw_az = (int16_t)((uint16_t)(dma_rx_buf[5])  << 8 | dma_rx_buf[6]);
    /* [7][8] TEMP ignoré */
 
    /* ── Parser gyro [9..14] ── */
    mpu6000.raw_gx = (int16_t)((uint16_t)(dma_rx_buf[9])  << 8 | dma_rx_buf[10]);
    mpu6000.raw_gy = (int16_t)((uint16_t)(dma_rx_buf[11]) << 8 | dma_rx_buf[12]);
    mpu6000.raw_gz = (int16_t)((uint16_t)(dma_rx_buf[13]) << 8 | dma_rx_buf[14]);
 
    /* ── Conversions ── */
    mpu6000.ax = (float)mpu6000.raw_ax * MPU6000_ACCEL_SCALE_MPS2;
    mpu6000.ay = (float)mpu6000.raw_ay * MPU6000_ACCEL_SCALE_MPS2;
    mpu6000.az = (float)mpu6000.raw_az * MPU6000_ACCEL_SCALE_MPS2;
    mpu6000.gx = (float)mpu6000.raw_gx * MPU6000_GYRO_SCALE_RAD_S;
    mpu6000.gy = (float)mpu6000.raw_gy * MPU6000_GYRO_SCALE_RAD_S;
    mpu6000.gz = (float)mpu6000.raw_gz * MPU6000_GYRO_SCALE_RAD_S;



    /* ── Conversions pour affichage debug puis print ── */
 
    // int32_t gx = (int32_t)mpu6000.raw_gx * 10000 / 9397;
    // int32_t gy = (int32_t)mpu6000.raw_gy * 10000 / 9397;
    // int32_t gz = (int32_t)mpu6000.raw_gz * 10000 / 9397;

    // int16_t ax = (int16_t)((int32_t)mpu6000.raw_ax * 981 / 4096);
    // int16_t ay = (int16_t)((int32_t)mpu6000.raw_ay * 981 / 4096);
    // int16_t az = (int16_t)((int32_t)mpu6000.raw_az * 981 / 4096);

    // print_to_console("G:", 2);
    // print_gyro_rads(gx); UART_Debug_Transmit_Char_LL(' ');
    // print_gyro_rads(gy); UART_Debug_Transmit_Char_LL(' ');
    // print_gyro_rads(gz);
    // print_to_console("  A:", 4);
    // print_accel_mps2(ax); UART_Debug_Transmit_Char_LL(' ');
    // print_accel_mps2(ay); UART_Debug_Transmit_Char_LL(' ');
    // print_accel_mps2(az);
    // UART_Debug_Transmit_Char_LL('\n'); UART_Debug_Transmit_Char_LL('\r');



    //ici faire la fusion basique pour estimer l'attitude, puis calculer les consignes moteurs, puis envoyer les consignes aux ESC pour tester 
    //(pas de PID dans ce callback, juste les conversions et le debug)
    

    // ----------- COMPLEMENTARY FILTER  -----------


    // // ─────────────────────────────────────────────────────────────────────────
    // // 1. Calcul angles à partir accéléromètre (référence absolue)
    // // ─────────────────────────────────────────────────────────────────────────
    // float pitch_accel = atan2f(mpu6000.ay, mpu6000.az);   // atan2(ay, az) donne roll
    // float roll_accel = atan2f(-mpu6000.ax, sqrtf(mpu6000.ay*mpu6000.ay + mpu6000.az*mpu6000.az));

    // // ─────────────────────────────────────────────────────────────────────────
    // // 2. Filtre complémentaire (fusion gyro + accel)
    // // ─────────────────────────────────────────────────────────────────────────
    // roll_angle  = ALPHA * (roll_angle  + mpu6000.gx * DT) + (1.0f - ALPHA) * roll_accel;
    // pitch_angle = ALPHA * (pitch_angle + mpu6000.gy * DT) + (1.0f - ALPHA) * pitch_accel;


    // ----------- MAHONY FILTER -----------

    // ─────────────────────────────────────────────────────────────────────────
    // 1. Normalisation vecteur accéléromètre
    // ─────────────────────────────────────────────────────────────────────────
    float recipNorm = 1.0f / sqrtf(mpu6000.ax * mpu6000.ax + 
                                    mpu6000.ay * mpu6000.ay + 
                                    mpu6000.az * mpu6000.az);
    float ax = mpu6000.ax * recipNorm;
    float ay = mpu6000.ay * recipNorm;
    float az = mpu6000.az * recipNorm;

    // ─────────────────────────────────────────────────────────────────────────
    // 2. Gravité estimée dans le repère corps (depuis quaternion)
    // ─────────────────────────────────────────────────────────────────────────
    // Rotation inverse de [0, 0, 1] (gravité terrestre NED) par q
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // ─────────────────────────────────────────────────────────────────────────
    // 3. Erreur vectorielle = cross product (a_mes × g_est)
    // ─────────────────────────────────────────────────────────────────────────
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // ─────────────────────────────────────────────────────────────────────────
    // 4. Correction PI sur le gyroscope
    // ─────────────────────────────────────────────────────────────────────────
    if (MAHONY_KI > 0.0f) {
        integralFBx += MAHONY_KI * ex * DT;
        integralFBy += MAHONY_KI * ey * DT;
        integralFBz += MAHONY_KI * ez * DT;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Gyroscope corrigé
    float gx_corr = mpu6000.gx + MAHONY_KP * ex + integralFBx;
    float gy_corr = mpu6000.gy + MAHONY_KP * ey + integralFBy;
    float gz_corr = mpu6000.gz + MAHONY_KP * ez + integralFBz;

    // ─────────────────────────────────────────────────────────────────────────
    // 5. Intégration du quaternion
    // ─────────────────────────────────────────────────────────────────────────
    // q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += 0.5f * (-qb * gx_corr - qc * gy_corr - q3 * gz_corr) * DT;
    q1 += 0.5f * ( qa * gx_corr + qc * gz_corr - q3 * gy_corr) * DT;
    q2 += 0.5f * ( qa * gy_corr - qb * gz_corr + q3 * gx_corr) * DT;
    q3 += 0.5f * ( qa * gz_corr + qb * gy_corr - qc * gx_corr) * DT;

    // ─────────────────────────────────────────────────────────────────────────
    // 6. Normalisation du quaternion
    // ─────────────────────────────────────────────────────────────────────────
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // ─────────────────────────────────────────────────────────────────────────
    // 7. Extraction angles d'Euler (convention NED, séquence ZYX)
    // ─────────────────────────────────────────────────────────────────────────
    // Pitch (rotation autour de X)
    pitch_angle = atan2f(2.0f * (q0 * q1 + q2 * q3), 
                        1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // Roll (rotation autour de Y)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        roll_angle = copysignf(1.57079632f, sinp); // ±90° (gimbal lock)
    else
        roll_angle = asinf(sinp);

    // Yaw (rotation autour de Z) — optionnel si pas de mag
    // yaw_angle = atan2f(2.0f * (q0 * q3 + q1 * q2), 
    //                    1.0f - 2.0f * (q2 * q2 + q3 * q3));

    /* ══════════════════════════════════════════════════════════════════════════
    * FIN MAHONY FILTER
    * roll_angle et pitch_angle sont maintenant à jour → utiliser pour la suite
    * ══════════════════════════════════════════════════════════════════════════ */


    // ─────────────────────────────────────────────────────────────────────────
    // 3. Détection décrochage (|roll| > 80° OU |pitch| > 80°)
    // ─────────────────────────────────────────────────────────────────────────
    if (fabsf(roll_angle) > STALL_ANGLE_RAD || fabsf(pitch_angle) > STALL_ANGLE_RAD) {
        motors_armed = false;  // Désarmer
    } else if (!motors_armed && fabsf(roll_angle) < 0.1f && fabsf(pitch_angle) < 0.1f) {
        motors_armed = true;   // Réarmer si revenu quasi horizontal (<6°)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 4. Commande moteurs (si armé) — Loi P simple sur angles
    // ─────────────────────────────────────────────────────────────────────────
    if (motors_armed) {
        // Corrections proportionnelles (négatif car on veut corriger l'erreur)
        float roll_correction  = -KP_DEMO * roll_angle;   // roll > 0 (droite penche) → baisse droite
        float pitch_correction = -KP_DEMO * pitch_angle;  // pitch > 0 (nez monte) → baisse avant
        
        // Mixer Quad-X simplifié (sans yaw, throttle constant)
        // Convention : FL=M1, FR=M2, RR=M3, RL=M4 (comme dans esc_driver)
        //
        //     FL(M1) ●         ● FR(M2)
        //              ╲     ╱
        //                ╲ ╱
        //                ╱ ╲
        //              ╱     ╲
        //     RL(M4) ●         ● RR(M3)
        //
        // Roll positif (penche gauche) → monter gauche (M1,M4), baisser droite (M2,M3)
        // Pitch positif (nez descend) → monter avant (M1,M2), baisser arrière (M3,M4)
        
        float m1_cmd = THROTTLE_BASE - roll_correction - pitch_correction;  // FL
        float m2_cmd = THROTTLE_BASE + roll_correction - pitch_correction;  // FR
        float m3_cmd = THROTTLE_BASE + roll_correction + pitch_correction;  // RR
        float m4_cmd = THROTTLE_BASE - roll_correction + pitch_correction;  // RL
        
        // Clipping [0, 3000]
        m1_cmd = (m1_cmd < 0.0f) ? 0.0f : ((m1_cmd > 3000.0f) ? 3000.0f : m1_cmd);
        m2_cmd = (m2_cmd < 0.0f) ? 0.0f : ((m2_cmd > 3000.0f) ? 3000.0f : m2_cmd);
        m3_cmd = (m3_cmd < 0.0f) ? 0.0f : ((m3_cmd > 3000.0f) ? 3000.0f : m3_cmd);
        m4_cmd = (m4_cmd < 0.0f) ? 0.0f : ((m4_cmd > 3000.0f) ? 3000.0f : m4_cmd);
        
        // Application aux ESC
        ESC_Set_Values((uint16_t)m1_cmd, (uint16_t)m2_cmd, (uint16_t)m3_cmd, (uint16_t)m4_cmd);
        
    } else {
        // Décrochage détecté → moteurs à 0
        ESC_Set_Values(0, 0, 0, 0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Debug print (fusion gyro + accel)
    // ─────────────────────────────────────────────────────────────────────────
    UART_Debug_Transmit_Buffer_LL((uint8_t*)"\n\rR:", 4);
    print_roll_deg(roll_angle);
    UART_Debug_Transmit_Buffer_LL((uint8_t*)" P:", 3);
    print_pitch_deg(pitch_angle);

    /* ══════════════════════════════════════════════════════════════════════════
    * FIN DU CODE DE DÉMONSTRATION
    * ══════════════════════════════════════════════════════════════════════════ */

    /*
     * Signaler à la fast_loop que les données sont prêtes.
     * La fast_loop consomme gx/gy/gz puis remet data_ready = false.
     * Si la fast_loop est plus lente que 1kHz (ne devrait pas arriver),
     * data_ready reste true et la prochaine ISR l'écrasera → overwrite
     * silencieux acceptable en rate loop (on ne bufferize pas, on prend le
     * plus récent).
     */
    mpu6000.data_ready = true;
}
 
/**
 * @brief  Callback EXTI3 — flanc montant MPU6000 DATA_RDY @ 1kHz.
 *         Appelé depuis EXTI3_IRQHandler dans stm32f4xx_it.c.
 */
void MPU6000_EXTI3_Callback(void)
{
    MPU6000_Start_DMA_Read();
}
void MPU6000_DMA2_Stream0_Callback(void)
{
    MPU6000_DMA_RX_Complete_Callback();
}