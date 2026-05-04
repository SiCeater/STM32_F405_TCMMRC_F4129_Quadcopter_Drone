#include "esc_driver.h"
#include "global.h"
#include <stdint.h>

/**
 * @brief initialisation des 4 ESC
 * ils sont commandé par des signaux PWM en sortie du timer 8 de 1000Hz
 * plage de 125 µs-> 250 µs => 3000->6000
 */
void ESC_Init()
{
    // Configurer les rapports cycliques initiaux des canaux (125µs = 3000)
    LL_TIM_OC_SetCompareCH1(TIM8, 3000); //top left motor
    LL_TIM_OC_SetCompareCH2(TIM8, 3000); //top right motor
    LL_TIM_OC_SetCompareCH3(TIM8, 3000); //bottom right motor
    LL_TIM_OC_SetCompareCH4(TIM8, 3000); //bottom left motor

    LL_TIM_EnableAllOutputs(TIM8);

    // Démarrer le compteur du Timer 8
    LL_TIM_EnableCounter(TIM8);
    
    // Remettre le compteur du Timer 8 à 0
    LL_TIM_SetCounter(TIM8, 0);

    LL_TIM_EnableAllOutputs(TIM8);
    
    // Activer les sorties PWM des canaux du Timer 8
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH4);

    // calibration des ESCs : on les met à 3000 (250µs) pendant 2 secondes pour qu'ils reconnaissent le signal minimum
    // et ensuite on les met à 0 (125µs) pendant 2 secondes pour qu'ils reconnaissent le signal de stop
    LED_On();
    ESC_Set_Values(3000,3000,3000,3000);
    LL_mDelay(2000);
    ESC_Set_Values(0,0,0,0);
    LL_mDelay(2000);
    LED_Off();

    // Debug optionnel
    if (debug)
    {
        print_to_console("\n\rESCs : initialized and calibrated", 36);
    }
}

/**
 * @brief Set the motors rotation speed (0->3000)
 *
 * @param front_left_motor
 * @param front_right_motor
 * @param rear_left_motor
 * @param rear_right_motor
 */
void ESC_Set_Values(uint16_t front_left_motor, uint16_t front_right_motor, uint16_t rear_right_motor, uint16_t rear_left_motor)
{
    if (front_left_motor>3000) {
        front_left_motor=3000;
    }
    else if (front_left_motor<0) {
        front_left_motor=0;
    }
    ESC_FL_MS(front_left_motor);

    if (front_right_motor>3000) {
        front_right_motor=3000;
    }
    else if (front_right_motor<0) {
        front_right_motor=0;
    }
    ESC_FR_MS(front_right_motor);

    if (rear_right_motor>3000) {
        rear_right_motor=3000;
    }
    else if (rear_right_motor<0) {
        rear_right_motor=0;
    }
    ESC_RR_MS(rear_right_motor);

    if (rear_left_motor>3000) {
        rear_left_motor=3000;
    }
    else if (rear_left_motor<0) {
        rear_left_motor=0;
    }
    ESC_RL_MS(rear_left_motor);
}

void ESC_Set_Global_Values()
{
    ESC_Set_Values(m1, m2, m3, m4);
}

void ESC_RR_MS(uint16_t x)
{
    if (x>3000) {
        x=3000;
    }
    else if (x<0) {
        x=0;
    }
    LL_TIM_OC_SetCompareCH1(TIM8, 3000 + x);
}

void ESC_RL_MS(uint16_t x)
{
    if (x>3000) {
        x=3000;
    }
    else if (x<0) {
        x=0;
    }
    LL_TIM_OC_SetCompareCH3(TIM8, 3000 + x);
}

void ESC_FL_MS(uint16_t x)
{
    if (x>3000) {
        x=3000;
    }
    else if (x<0) {
        x=0;
    }
    LL_TIM_OC_SetCompareCH4(TIM8, 3000 + x);
}

void ESC_FR_MS(uint16_t x)
{
    if (x>3000) {
        x=3000;
    }
    else if (x<0) {
        x=0;
    }
    LL_TIM_OC_SetCompareCH2(TIM8, 3000 + x);
}

void ESC_Test() // WARNING : this function is blocking, make sure to remove propeller before using it
{
    ESC_FL_MS(1500);
    LL_mDelay(1000);
    ESC_FL_MS(3000);
    LL_mDelay(1000);
    ESC_FL_MS(0);

    ESC_FR_MS(1500);
    LL_mDelay(1000);
    ESC_FR_MS(3000);
    LL_mDelay(1000);
    ESC_FR_MS(0);

    ESC_RR_MS(1500);
    LL_mDelay(1000);
    ESC_RR_MS(3000);
    LL_mDelay(1000);
    ESC_RR_MS(0);

    ESC_RL_MS(1500);
    LL_mDelay(1000);
    ESC_RL_MS(3000);
    LL_mDelay(1000);
    ESC_RL_MS(0);
}