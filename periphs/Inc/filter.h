#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

// 低通滤波器结构体
typedef struct
{
    float prev_value;
    float coefficient;
} LowPassFilter_t;

// 变化率限制器结构体
typedef struct
{
    float prev_value;
    float max_rate;
} RateLimiter_t;

// 移动平均滤波器结构体
#define WINDOW_SIZE 5
typedef struct
{
    float buffer[WINDOW_SIZE];
    int index;
    float sum;
} MovingAverage_t;

// 函数声明
void LowPass_Filter_Init(LowPassFilter_t *filter, float coefficient);
float LowPass_Filter(LowPassFilter_t *filter, float value);

void Rate_Limiter_Init(RateLimiter_t *limiter, float max_rate);
float Rate_Limiter(RateLimiter_t *limiter, float value);

void MA_Init(MovingAverage_t *ma);
float Moving_Average(MovingAverage_t *ma, float value);

float Hybrid_Filter(float value, float coefficient, float max_rate);

#endif