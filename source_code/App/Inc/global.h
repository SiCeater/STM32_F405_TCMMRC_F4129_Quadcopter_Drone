
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern uint8_t elrs_buffer[128];

extern bool debug_init;
extern bool debug_rc;
extern bool debug_imu;
extern bool debug_warn;

extern int16_t AccData[3];
extern int16_t MagData[3];
extern int16_t GyroData[3];

extern uint16_t m1;
extern uint16_t m2;
extern uint16_t m3;
extern uint16_t m4;
extern uint16_t m5;
extern uint16_t m6;
extern uint16_t m7;
extern uint16_t m8;

uint32_t get_ms(void);

#endif
    
