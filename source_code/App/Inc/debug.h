#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "main.h"
#include "global.h"

void UART_Debug_Transmit_Char_LL(uint8_t c);
void UART_Debug_Transmit_Buffer_LL(uint8_t *data, uint16_t size);
void print_remote_data();
void print_gyro_data(int16_t dps_x10);
void print_accel_data(int16_t g_x10);
void print_gyro_rads(int32_t rad_x1000);
void print_accel_mps2(int16_t mps2_x100);
void print_roll_deg(float roll_rad);
void print_pitch_deg(float pitch_rad);
void print_yaw_deg(float yaw_rad);
void print_to_console(char *buffer, uint16_t buffer_size);

#endif