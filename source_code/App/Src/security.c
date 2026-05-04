#include "security.h"

void connection_lost_routine()
{

    if (debug)
        print_to_console("\nremote connection lost !\n", 26);

    // ICI DESACTIVER LA LOOP CONTROL ET SECURITY
    LL_TIM_DisableIT_UPDATE(TIM8);
    LL_TIM_DisableIT_UPDATE(TIM6);
    ESC_Set_Values(0,0,0,0);
    LED_On();

    do
    {
        if (LL_USART_IsActiveFlag_IDLE(USART1))// on teste si ya eu une nouvelle reconnexion
            missed_transfers=0;
        LL_USART_ClearFlag_IDLE(USART1); // Efface le drapeau IDLE
    }
    while(missed_transfers==10);

    LED_Off();

    // ICI REACTIVER LA LOOP CONTROL ET SECURITY
    LL_TIM_EnableIT_UPDATE(TIM8);
    LL_TIM_EnableIT_UPDATE(TIM6);

    if(debug)
        print_to_console("\nremote connection re-etablished !\n", 35);
}