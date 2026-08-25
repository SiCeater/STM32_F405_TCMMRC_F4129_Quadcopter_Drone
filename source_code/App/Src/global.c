#include "global.h"

uint8_t elrs_buffer[128] = {0};

bool debug_init = false;
bool debug_rc = false;
bool debug_imu = false;
bool debug_warn = true;

int16_t AccData[3] = {0};
int16_t MagData[3] = {0};
int16_t GyroData[3] = {0};

uint16_t m1=3000;
uint16_t m2=3000;
uint16_t m3=3000;
uint16_t m4=3000;
uint16_t m5=3000;
uint16_t m6=3000;
uint16_t m7=3000;
uint16_t m8=3000;

volatile uint32_t ms_counter = 0;

uint32_t get_ms(void)
{
    return ms_counter;
}