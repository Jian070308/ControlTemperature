#ifndef CONTROLTEMPERATURE_PID_H
#define CONTROLTEMPERATURE_PID_H

#define P 0
#define I 0
#define D 0
#define i_Max 400.0f
#define i_Add 3.0f
#define output_Max 1000

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;
    float real;

    float error;
    float last_error;
    float integral;

    float integralMax;
    float outputMax;
    float integralAdd;                  // 决定何时加入积分
} PID_TypeDef;

void PID_Init();

float PID_Output(float target, double real);

void PID_Reset();

#endif //CONTROLTEMPERATURE_PID_H