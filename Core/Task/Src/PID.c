#include "cmsis_os2.h"
#include "../Inc/MAX31855.h"
#include "PID.h"
#include <math.h>

#include <FreeRTOS.h>
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"

#define MAX 1000
#define MIN (-1000)

extern TIM_HandleTypeDef htim2;

void ReadData()
{
   MAX31855_ReadData(&MAX31855_Handle);
   if(!MAX31855_GetFault(&MAX31855_Handle))
   {
       realTemp = MAX31855_GetTemperature(&MAX31855_Handle);
   }
}

void SetPwm(int16_t power)
{
    if (power > MAX) power = MAX;
    if (power < MIN) power = MIN;

    if (power > 0)
    {
        // 加热
        // 把另一端置 0，防止 H 桥上下桥臂直通短路烧毁
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, power);
    }
    else if (power < 0)
    {
        // 制冷
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -power);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

void PID_Init()
{
    pid.Kp = P;
    pid.Ki = I;
    pid.Kd = D;

    pid.outputMax = output_Max;
    pid.integralMax = i_Max;
    pid.integralAdd = i_Add;

    PID_Reset();
}

// 当设定温度突变时，调用一次，清除误差
void PID_Reset()
{
    pid.error = 0.0f;
    pid.last_error = 0.0f;
    pid.integral = 0.0f;
}

float PID_Output(const float target, const double real)
{
    pid.target = target;
    pid.real = real;

    pid.error = pid.target - pid.real;

    float p_out = pid.Kp * pid.error;

    // 当设定温度突变时，先去除积分，防止积分累计过大导致超调
    if (fabsf(pid.error) <= pid.integralAdd)
    {
        pid.integral += pid.error;

        if (pid.integral > pid.integralMax)
        {
            pid.integral = pid.integralMax;
        } else if (pid.integral < -pid.integralMax)
        {
            pid.integral = -pid.integralMax;
        }
    }

    float i_out = pid.Ki * pid.integral;

    float d_out = pid.Kd * (pid.error - pid.last_error);
    pid.last_error = pid.error;

    float total_out = p_out + i_out + d_out;

    if (total_out > pid.outputMax)
    {
        total_out = pid.outputMax;
    } else if (total_out < -pid.outputMax)
    {
        total_out = -pid.outputMax;
    }

    return total_out;
}

void StartPIDTask(void *argument)
{
    // 等待传感器硬件上电完毕
    vTaskDelay(pdMS_TO_TICKS(300));

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        ReadData();
        const float pwm_cmd = PID_Output((float)targetTemp, realTemp);
        SetPwm((int16_t)pwm_cmd);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
