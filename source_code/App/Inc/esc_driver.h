#ifndef __ESC_DRIVER_H__
#define __ESC_DRIVER_H__

#include "main.h"
#include "global.h"
#include "debug.h"
#include "led_driver.h"

void ESC_Init();
void ESC_Set_Values(uint16_t front_left_motor, uint16_t front_right_motor, uint16_t rear_right_motor, uint16_t rear_left_motor);
void ESC_Set_Global_Values();
void ESC_FL_MS(uint16_t x);
void ESC_FR_MS(uint16_t x);
void ESC_RR_MS(uint16_t x);
void ESC_RL_MS(uint16_t x);
void ESC_Test();

#endif