#include "security.h"

void connection_lost_routine()
{

    if (debug_warn)
        print_to_console("\nremote connection lost !\n", 26);

}