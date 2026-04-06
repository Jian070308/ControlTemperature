#include "cmsis_os2.h"
#include "../Inc/MAX31855.h"
#include "PID.h"
#include <math.h>

#include <FreeRTOS.h>
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"

#define MAX 1000
#define MIN (-1000)
#define TEMP_MEDIAN_WINDOW 3U
#define TEMP_MAX_STEP_C 3.0f
#define TEMP_FILTER_ALPHA 0.20f

extern TIM_HandleTypeDef htim2;

static float temp_samples[TEMP_MEDIAN_WINDOW] = {0.0f};
static uint8_t temp_sample_count = 0U;
static uint8_t temp_sample_index = 0U;
static uint8_t temp_filter_ready = 0U;
static float filtered_temp = 0.0f;

static float clampf(const float value, const float min_value, const float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static void sort3(float *a, float *b, float *c)
{
    float temp = 0.0f;

    if (*a > *b)
    {
        temp = *a;
        *a = *b;
        *b = temp;
    }
    if (*b > *c)
    {
        temp = *b;
        *b = *c;
        *c = temp;
    }
    if (*a > *b)
    {
        temp = *a;
        *a = *b;
        *b = temp;
    }
}

static float filter_temperature_sample(const float sample)
{
    temp_samples[temp_sample_index] = sample;
    temp_sample_index = (uint8_t)((temp_sample_index + 1U) % TEMP_MEDIAN_WINDOW);
    if (temp_sample_count < TEMP_MEDIAN_WINDOW)
    {
        temp_sample_count++;
    }

    if (temp_sample_count < TEMP_MEDIAN_WINDOW)
    {
        filtered_temp = sample;
        return filtered_temp;
    }

    {
        float a = temp_samples[0];
        float b = temp_samples[1];
        float c = temp_samples[2];
        float median = 0.0f;

        sort3(&a, &b, &c);
        median = b;

        if (temp_filter_ready == 0U)
        {
            filtered_temp = median;
            temp_filter_ready = 1U;
            return filtered_temp;
        }

        if (fabsf(median - filtered_temp) > TEMP_MAX_STEP_C)
        {
            return filtered_temp;
        }

        filtered_temp += TEMP_FILTER_ALPHA * (median - filtered_temp);
        return filtered_temp;
    }
}

static float get_output_limit(const float abs_error)
{
    if (abs_error <= pid.smallErrorBand)
    {
        return pid.smallOutputMax;
    }

    if (abs_error <= pid.mediumErrorBand)
    {
        return pid.mediumOutputMax;
    }

    return pid.outputMax;
}

void ReadData()
{
    MAX31855_ReadData(&MAX31855_Handle);
    if (!MAX31855_GetFault(&MAX31855_Handle))
    {
        const float sample = MAX31855_GetTemperature(&MAX31855_Handle);

        if ((sample > -100.0f) && (sample < 400.0f))
        {
            realTemp = filter_temperature_sample(sample);
        }
    }
}

void SetPwm(int16_t power)
{
    if (power > MAX) power = MAX;
    if (power < MIN) power = MIN;

    if (power < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -power);
    }
    else if (power > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, power);
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
    pid.sampleTime = PID_SAMPLE_TIME_S;
    pid.dFilterAlpha = PID_D_FILTER_ALPHA;
    pid.deadBand = PID_DEADBAND;
    pid.smallErrorBand = PID_SMALL_ERROR_BAND;
    pid.mediumErrorBand = PID_MEDIUM_ERROR_BAND;
    pid.smallOutputMax = PID_SMALL_OUTPUT_MAX;
    pid.mediumOutputMax = PID_MEDIUM_OUTPUT_MAX;

    PID_Reset();
}

void PID_Reset()
{
    pid.target = 0.0f;
    pid.real = 0.0f;
    pid.error = 0.0f;
    pid.last_error = 0.0f;
    pid.last_real = 0.0f;
    pid.integral = 0.0f;
    pid.derivative = 0.0f;
    temp_sample_count = 0U;
    temp_sample_index = 0U;
    temp_filter_ready = 0U;
    filtered_temp = (float)realTemp;
}

float PID_Output(const float target, const double real)
{
    const float real_value = (float)real;
    const float output_max = (pid.outputMax > 0.0f) ? pid.outputMax : output_Max;
    const float sample_time = (pid.sampleTime > 0.0f) ? pid.sampleTime : PID_SAMPLE_TIME_S;

    pid.target = target;
    pid.real = real_value;
    pid.error = pid.target - pid.real;

    const float abs_error = fabsf(pid.error);
    const float dynamic_output_limit = get_output_limit(abs_error);

    float working_error = pid.error;
    if (abs_error <= pid.deadBand)
    {
        working_error = 0.0f;
    }
    const float p_out = pid.Kp * working_error;

    // 微分先行逻辑保持不变，但只有在获取到真实变化时才有意义（通过延长 Task 周期解决）
    const float measurement_rate = (pid.real - pid.last_real) / sample_time;
    pid.derivative = pid.dFilterAlpha * pid.derivative + (1.0f - pid.dFilterAlpha) * measurement_rate;
    const float d_out = -pid.Kd * pid.derivative;

    const float total_without_i = p_out + d_out;
    float next_integral = pid.integral;

    // 积分分离逻辑
    if (fabsf(working_error) <= pid.integralAdd && fabsf(working_error) > 0.0f) // 【修改说明：在死区内(working_error == 0)不累加积分，防止死区内积分漂移】
    {
        // 【修改说明：积分累加必须乘以 dt (sample_time)，使得 Ki 具备标准时间物理意义】
        const float candidate_integral = clampf(pid.integral + (working_error * sample_time), -pid.integralMax, pid.integralMax);
        const float candidate_i_out = pid.Ki * candidate_integral;
        const float candidate_total = total_without_i + candidate_i_out;

        // 积分抗饱和逻辑 (Anti-Windup)
        const uint8_t saturating_high = candidate_total > dynamic_output_limit;
        const uint8_t saturating_low = candidate_total < -dynamic_output_limit;

        if ((!saturating_high || working_error < 0.0f) && (!saturating_low || working_error > 0.0f))
        {
            next_integral = candidate_integral;
        }
    }
    pid.integral = next_integral;

    const float i_out = pid.Ki * pid.integral;
    float total_out = total_without_i + i_out;

    total_out = clampf(total_out, -dynamic_output_limit, dynamic_output_limit);
    total_out = clampf(total_out, -output_max, output_max);

    pid.last_error = pid.error;
    pid.last_real = pid.real;

    return total_out;
}

void StartPIDTask(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(300));

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        ReadData();
        // const float pwm_cmd = PID_Output((float)targetTemp, realTemp);
        // SetPwm((int16_t)pwm_cmd);
        SetPwm(-900);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));

    }
}


