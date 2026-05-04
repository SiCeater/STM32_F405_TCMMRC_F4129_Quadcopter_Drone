#include "led_driver.h"

void LED_On()
{
    // Écrire un RESET (0) pour allumer la LED
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
}

void LED_Off()
{
    // Écrire un SET (1) pour éteindre la LED
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
}

void LED_Toggle()
{
    // Basculer l'état actuel de la broche (SET <-> RESET)
    LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
