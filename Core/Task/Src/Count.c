#include <stdlib.h>

#include "cmsis_os2.h"
#include "main.h"
#include "stm32f1xx_hal_tim.h"

extern TIM_HandleTypeDef htim1;

void StartCountTask(void *argument)
{
    static int16_t last_counter = 0;
    static uint8_t flag = 0;

    for (;;)
    {
        const int16_t current_counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        const int16_t diff = current_counter - last_counter;
        uint8_t target = targetTemp;
        uint8_t last_target = targetTemp;

        if (diff >= 4)
        {
            target += 1;
            last_counter = current_counter;
        }
        else if (diff <= -4)
        {
            target -= 1;
            last_counter = current_counter;
        }

        // 限幅
        target = target > 70 ? 70 : (target < 40 ? 40 : target);

        if (HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin) == GPIO_PIN_RESET && flag == 0)
        {
            osDelay(20);
            if (HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin) == GPIO_PIN_RESET)
            {
                flag = 1;
                targetTemp = target;                // 确认设定目标温度

                if (abs(targetTemp - last_target) > 10)
                {
                    PID_Reset();                       // 突变时重置 PID，清除误差
                }
            }
        }
        else if (HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin) == GPIO_PIN_SET)
        {
            flag = 0;
        }

        osDelay(10);
    }
}
