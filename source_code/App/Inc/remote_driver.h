#ifndef __REMOTE_DRIVER_H__
#define __REMOTE_DRIVER_H__

#include "global.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * CRSF — Crossfire Serial Protocol (couche série utilisée par ELRS)
 * ========================================================================== */

/* --- Adresses CRSF --- */
#define CRSF_ADDR_BROADCAST             0x00
#define CRSF_ADDR_FLIGHT_CONTROLLER     0xC8
#define CRSF_ADDR_RECEIVER              0xEA
#define CRSF_ADDR_TX_MODULE             0xEE

/* --- Types de frame gérés --- */
#define CRSF_FRAMETYPE_GPS               0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR    0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS   0x14
#define CRSF_FRAMETYPE_RC_CHANNELS       0x16
#define CRSF_FRAMETYPE_LINK_STATS_RX     0x1C
#define CRSF_FRAMETYPE_LINK_STATS_TX     0x1D
#define CRSF_FRAMETYPE_FLIGHT_MODE       0x21
#define CRSF_FRAMETYPE_DEVICE_PING       0x28
#define CRSF_FRAMETYPE_DEVICE_INFO       0x29
#define CRSF_FRAMETYPE_PARAM_ENTRY       0x2B
#define CRSF_FRAMETYPE_PARAM_READ        0x2C
#define CRSF_FRAMETYPE_PARAM_WRITE       0x2D

/* --- Constantes protocole / driver --- */
#define CRSF_CRC_POLY               0xD5
#define CRSF_FRAME_MAX_SIZE         64     // ADDR+LEN+TYPE+payload(<=60)+CRC
#define CRSF_DMA_BUF_SIZE           128    // puissance de 2 obligatoire (masque AND)
#define CRSF_CHANNEL_VALUE_MIN      172
#define CRSF_CHANNEL_VALUE_MID      992
#define CRSF_CHANNEL_VALUE_MAX      1811
#define CRSF_FAILSAFE_TIMEOUT_MS    100    // ~10 paquets manqués @100Hz

/* --- Tuning traitement RC (application layer) --- */
#define CRSF_RC_DEADZONE             0.03f
#define CRSF_RC_EXPO_RATE            0.30f
#define CRSF_RC_LPF_HZ                30.0f

/* ==========================================================================
 * Structures
 * ========================================================================== */

typedef enum {
    CRSF_STATE_WAIT_SYNC,
    CRSF_STATE_LEN,
    CRSF_STATE_PAYLOAD,
} crsf_parse_state_t;

typedef struct {
    crsf_parse_state_t state;
    uint8_t  frame[CRSF_FRAME_MAX_SIZE];
    uint8_t  frame_len;     // valeur du champ LEN
    uint8_t  idx;           // octets déjà accumulés dans frame[]
    uint32_t last_rx_ms;    // horodatage dernier octet -> timeout resync
} crsf_parser_t;

typedef struct {
    uint16_t    ch[16];               // 16 canaux, valeurs 172-1811
    uint32_t    last_update_ms;       // horodatage dernière trame 0x16 valide
    bool        first_frame_received; // handshake initial OK
    bool        valid;
} crsf_rc_t;

typedef struct {
    uint8_t     uplink_rssi_ant1;
    uint8_t     uplink_rssi_ant2;
    uint8_t     uplink_lq;      // 0-100 %
    int8_t      uplink_snr;     // dB
    uint8_t     active_antenna;
    uint8_t     rf_mode;
    uint8_t     uplink_tx_power;
    uint8_t     downlink_rssi;
    uint8_t     downlink_lq;
    int8_t      downlink_snr;
    uint32_t    last_update_ms;
    bool        valid;
} crsf_link_stats_t;

typedef struct {
    char        name[16];   // tronqué si besoin
    bool        received;
} crsf_device_info_t;

typedef enum {
    FLIGHT_MODE_ACRO    = 0,
    FLIGHT_MODE_ANGLE   = 1,
    FLIGHT_MODE_ALTHOLD = 2,
} flight_mode_t;

typedef struct {
    float roll, pitch, throttle, yaw;                       // normalisés post deadzone+expo
    float roll_filt, pitch_filt, throttle_filt, yaw_filt;   // post LPF PT1
} rc_axes_t;

typedef struct {
    bool            armed;          // CH5
    flight_mode_t   flight_mode;    // CH6
    bool            beeper;         // CH7
    bool            blackbox;       // CH8
    uint16_t        aux[8];         // ch[8..15] bruts, extensions futures
} rc_switches_t;

typedef enum {
    FAILSAFE_OK,
    FAILSAFE_WARN,
    FAILSAFE_ACTIVE,
} failsafe_state_t;

typedef struct {
    failsafe_state_t state;
    uint32_t          active_since_ms;
    uint32_t          count;         // nb d'entrées en failsafe depuis boot
} rc_failsafe_t;

/* ==========================================================================
 * Variables globales — définies dans remote_driver.c, extern ici
 * (ne JAMAIS redéfinir ces variables dans un autre .c, sinon erreur linker)
 * ========================================================================== */

extern crsf_parser_t       crsf_parser;
extern crsf_rc_t           crsf_rc;
extern crsf_link_stats_t   crsf_link;
extern crsf_device_info_t  crsf_device;

extern rc_axes_t           rc_axes;
extern rc_switches_t       rc_switches;
extern rc_failsafe_t       rc_failsafe;

/* ==========================================================================
 * API publique
 * ========================================================================== */

void Remote_Init(void);

/* À appeler depuis USART2_IRQHandler() dans stm32f4xx_it.c, sur détection IDLE.
 * Ce fichier étant régénéré par CubeMX, ne pas modifier son contenu directement
 * au-delà de cet appel unique. */
void Remote_UART_IdleHandler(void);

/* À appeler périodiquement (ta slow loop, ex 400 Hz) avec le dt réel en secondes.
 * Calcule failsafe, décodage switches, normalisation + filtrage des axes. */
void Remote_Process(float dt);

bool rc_is_valid(void);

/* Exposées si besoin en dehors du driver (tests, telemetry custom, etc.) */
void    crsf_unpack_channels(const uint8_t *payload, uint16_t out[16]);
uint8_t crsf_crc8(const uint8_t *data, uint8_t len);
void    crsf_parse_byte(crsf_parser_t *p, uint8_t byte);
void    crsf_dispatch_frame(const uint8_t *frame);

#endif // REMOTE_DRIVER_H