#include <stdio.h>
#include "cmsis_os2.h"
#include "oled.h"

static char message[25] = "";

void StartOledTask(void *argument)
{
    for (;;)
    {
        OLED_NewFrame();
        printf(message, "Real temperature: %.2fC", realTemp);
        OLED_PrintString(1, 1, message, &font16x16, OLED_COLOR_NORMAL);
        printf(message, "Target temperature: %dC", targetTemp);
        OLED_PrintString(1, 20, message, &font16x16, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        osDelay(200);
    }
}
