#ifndef CONTROLTEMPERATURE_PID_H
#define CONTROLTEMPERATURE_PID_H

#define P 0
#define I 0
#define D 0
#define i_Max 400.0f
#define i_Add 3.0f
#define output_Max 1000
#define PID_SAMPLE_TIME_S 0.1f
#define PID_D_FILTER_ALPHA 0.85f
#define PID_DEADBAND 0.20f
#define PID_SMALL_ERROR_BAND 0.80f
#define PID_MEDIUM_ERROR_BAND 2.00f
#define PID_SMALL_OUTPUT_MAX 180.0f
#define PID_MEDIUM_OUTPUT_MAX 400.0f

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;
    float real;

    float error;
    float last_error;
    float last_real;
    float integral;
    float derivative;

    float integralMax;
    float outputMax;
    float integralAdd;
    float sampleTime;
    float dFilterAlpha;
    float deadBand;
    float smallErrorBand;
    float mediumErrorBand;
    float smallOutputMax;
    float mediumOutputMax;
} PID_TypeDef;

void PID_Init();

float PID_Output(float target, double real);

void PID_Reset();

#endif //CONTROLTEMPERATURE_PID_H
