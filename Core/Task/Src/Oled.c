#include <stdio.h>
#include "cmsis_os2.h"
#include "oled.h"

static char message[25] = "";
uint8_t show_target = 40;

void StartOledTask(void *argument)
{

    for (;;)
    {
        OLED_NewFrame();
        sprintf(message, "Real:%.2fC", realTemp);
        OLED_PrintString(1, 1, message, &font13x13, OLED_COLOR_NORMAL);
        sprintf(message, "Target:%dC", show_target);
        OLED_PrintString(1, 20, message, &font13x13, OLED_COLOR_NORMAL);
        sprintf(message, "Target:%dC", targetTemp);
        OLED_PrintString(1, 40, message, &font13x13, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        osDelay(10);

    }
}
