/* mpu6000.h
 * Driver LL (Low-Layer) pour MPU6000 sur STM32F405
 * SPI1 + DMA2, fast loop 1kHz — GYRO ONLY
 *
 * Câblage :
 *   PC2  → MPU6000_SPI_CS   (GPIO Output)
 *   PC3  → MPU6000_INT      (EXTI3, rising edge)
 *   PA5  → SPI1_SCLK        (AF5)
 *   PA6  → SPI1_MISO        (AF5)
 *   PA7  → SPI1_MOSI        (AF5)
 *   DMA2 Stream0 Ch3        → SPI1_RX
 *   DMA2 Stream3 Ch3        → SPI1_TX
 */
 
#ifndef MPU6000_H
#define MPU6000_H
 
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "debug.h"
#include <stdint.h>
#include <stdbool.h>
#include "global.h"
#include <string.h>
#include "esc_driver.h"
#include <math.h>
 
 
/* ─── defines pour le test fast loop ─────────────────────────────────────────────────── */
#define DT 0.001f                  // 1 kHz → dt = 1ms
#define STALL_ANGLE_RAD 1.4f       // 80° en radians
#define KP_DEMO 800.0f             // Gain proportionnel (ajustable, valeur demo)
#define THROTTLE_BASE 1500.0f      // Throttle de base (milieu de la plage 0-3000)

// Gains Mahony (à tuner selon les vibrations)
#define MAHONY_KP  1.5f    // Gain proportionnel (1.0-2.0 pour porteur stable)
#define MAHONY_KI  0.05f   // Gain intégral (0.0-0.1, optionnel pour corriger dérive gyro)

/* ─── Registres MPU6000 ─────────────────────────────────────────────────── */
#define MPU6000_REG_SMPLRT_DIV      0x19U
#define MPU6000_REG_CONFIG          0x1AU   /* DLPF_CFG */
#define MPU6000_REG_GYRO_CONFIG     0x1BU
#define MPU6000_REG_ACCEL_CONFIG    0x1CU
#define MPU6000_REG_ACCEL_XOUT_H    0x3BU   /* Adresse de départ du burst accel+gyro */
#define MPU6000_REG_INT_PIN_CFG     0x37U
#define MPU6000_REG_INT_ENABLE      0x38U
#define MPU6000_REG_GYRO_XOUT_H     0x43U
#define MPU6000_REG_USER_CTRL       0x6AU
#define MPU6000_REG_PWR_MGMT_1      0x6BU
#define MPU6000_REG_WHO_AM_I        0x75U
 
#define MPU6000_WHO_AM_I_VAL        0x68U
 
/* ─── SPI ───────────────────────────────────────────────────────────────── */
#define MPU6000_READ_FLAG           0x80U   /* Bit 7 = 1 en lecture SPI */
 
/* ─── GPIO CS ───────────────────────────────────────────────────────────── */
#define MPU6000_CS_PORT             GPIOC
#define MPU6000_CS_PIN              LL_GPIO_PIN_2
 
/* ─── DMA ───────────────────────────────────────────────────────────────── */
#define MPU6000_DMA                 DMA2
#define MPU6000_DMA_RX_STREAM       LL_DMA_STREAM_0
#define MPU6000_DMA_TX_STREAM       LL_DMA_STREAM_3
#define MPU6000_DMA_CHANNEL         LL_DMA_CHANNEL_3
 
/* ─── Config gyro ───────────────────────────────────────────────────────── */
/*
 * DLPF_CFG = 0  →  BW gyro = 256Hz, Fs interne = 8kHz
 * SMPLRT_DIV = 7  →  ODR = 8000 / (1+7) = 1000 Hz  ✓
 * FSR ±2000°/s  →  sensibilité = 16.4 LSB/°/s
 */
#define MPU6000_GYRO_FSR_2000_DPS   0x18U
#define MPU6000_GYRO_SCALE_RAD_S    (1.0f / 16.4f * 0.01745329252f)
 
/* ─── Config accel ──────────────────────────────────────────────────────── */
/*
 * AFS_SEL = 2 → ±8g, sensibilité = 4096 LSB/g
 * Choix ±8g : un quadrotor peut encaisser 4-6g en sortie de virage serré,
 * ±2g (reset default) saturerait. ±16g est surdimensionné pour ce chassis.
 * Conversion debug : raw × 10 / 4096 → g×10 (ex: 4096 → 10 → "1.0g")
 */
#define MPU6000_ACCEL_FSR_8G        0x10U
#define MPU6000_ACCEL_SCALE_MPS2    (9.80665f / 4096.0f)   /* raw → m/s² */
 
/*
 * Burst démarrant à ACCEL_XOUT_H (0x3B) — registres consécutifs :
 *   1 adresse + 6 accel + 2 temp (ignoré) + 6 gyro = 15 octets
 *   Temps @ 10.5MHz : 15×8 / 10.5e6 ≈ 11.4µs — toujours dans le budget 1kHz
 */
#define MPU6000_BURST_LEN           15U
 
/* ─── Structure de données ──────────────────────────────────────────────── */
typedef struct {
    /* Valeurs converties (rad/s) — consommées directement par le rate PID */
    float    gx;
    float    gy;
    float    gz;
 
    /* Accélérations converties (m/s²) — debug / fusion attitude slow loop */
    float    ax;
    float    ay;
    float    az;
 
    /* Bruts signés — utiles pour calibration / debug / logging */
    int16_t  raw_gx;
    int16_t  raw_gy;
    int16_t  raw_gz;
 
    int16_t  raw_ax;
    int16_t  raw_ay;
    int16_t  raw_az;
 
    volatile bool data_ready;
} mpu6000_data_t;
 
/* Instance globale — accédée directement par la fast_loop */
extern mpu6000_data_t mpu6000;
 
/* ─── API publique ──────────────────────────────────────────────────────── */
 
/**
 * @brief  Initialise le MPU6000 : reset, WHO_AM_I, config registres, prep DMA.
 *         À appeler une fois dans main() après MX_SPI1_Init() et MX_DMA_Init().
 * @retval true  si WHO_AM_I == 0x68 (MPU6000 détecté et configuré)
 *         false si non détecté — stopper le firmware, câblage/alim KO
 */
void MPU6000_Init(void);
 
/**
 * @brief  Lance une transaction DMA SPI1 non-bloquante (7 octets).
 *         Relance le burst RX en réarmant les streams DMA (mode Normal).
 *         Appelé depuis mpu6000_exti_callback().
 *         Peut également être appelé depuis un TIMx ISR pour un déclenchement
 *         timer-based (si l'on préfère une loop synchrone timer plutôt que DRY).
 */
void MPU6000_Start_DMA_Read(void);
 
/**
 * @brief  Callback Transfer Complete DMA RX (Stream0).
 *         À appeler depuis DMA2_Stream0_IRQHandler dans stm32f4xx_it.c.
 *         Désactive les DMA req SPI → CS HIGH → parse raw → rad/s → data_ready.
 */
void MPU6000_DMA_RX_Complete_Callback(void);
 
/**
 * @brief  Callback EXTI3 (flanc montant DATA_RDY du MPU6000 @ 1kHz).
 *         À appeler depuis EXTI3_IRQHandler dans stm32f4xx_it.c.
 */
void MPU6000_EXTI3_Callback(void);
 
void MPU6000_DMA2_Stream0_Callback(void);
 
#endif /* MPU6000_H */