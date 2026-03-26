#include <math.h>
#include <stdlib.h>

#include "cmsis_os2.h"
#include "main.h"
#include "stm32f1xx_hal_tim.h"

extern TIM_HandleTypeDef htim1;
extern uint8_t show_target;

void StartCountTask(void *argument)
{
    static int16_t last_counter = 0;
    static uint8_t flag = 0;

    uint8_t target = (uint8_t)targetTemp;
    float last_target = targetTemp;

    for (;;)
    {
        const int16_t current_counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        const int16_t diff = current_counter - last_counter;

        if (diff >= 4)
        {
            target += 2;
            last_counter = current_counter;
        }
        else if (diff <= -4)
        {
            target -= 2;
            last_counter = current_counter;
        }

        target = target > 70 ? 70 : (target < 40 ? 40 : target);

        show_target = target;

        if (HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET && flag == 0)
        {
            osDelay(20);
            if (HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET)
            {
                flag = 1;
                last_target = targetTemp;
                targetTemp = (float)target;

                if (fabsf(targetTemp - last_target) > 10.0f)
                {
                    PID_Reset();
                }
            }
        }
        else if (HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_SET)
        {
            flag = 0;
        }

        osDelay(10);
    }
}
