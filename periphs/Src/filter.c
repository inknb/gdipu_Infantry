#include "filter.h"

// 低通滤波器初始化
void LowPass_Filter_Init(LowPassFilter_t *filter, float coefficient)
{
    filter->prev_value = 0.0f;
    filter->coefficient = (coefficient > 1.0f) ? 1.0f : (coefficient < 0.0f) ? 0.0f
                                                                             : coefficient;
}

// 低通滤波
float LowPass_Filter(LowPassFilter_t *filter, float value)
{
    // 如果是第一次运行，直接返回当前值
    if (filter->prev_value == 0.0f && value != 0.0f)
    {
        filter->prev_value = value;
        return value;
    }

    // 计算滤波后的值
    filter->prev_value = filter->coefficient * value +
                         (1.0f - filter->coefficient) * filter->prev_value;

    return filter->prev_value;
}

// 变化率限制器初始化
void Rate_Limiter_Init(RateLimiter_t *limiter, float max_rate)
{
    limiter->prev_value = 0.0f;
    limiter->max_rate = max_rate;
}

// 变化率限制
float Rate_Limiter(RateLimiter_t *limiter, float value)
{
    float change = value - limiter->prev_value;

    // 第一次运行
    if (limiter->prev_value == 0.0f && value != 0.0f)
    {
        limiter->prev_value = value;
        return value;
    }

    // 限制变化率
    if (change > limiter->max_rate)
        limiter->prev_value += limiter->max_rate;
    else if (change < -limiter->max_rate)
        limiter->prev_value -= limiter->max_rate;
    else
        limiter->prev_value = value;

    return limiter->prev_value;
}

// 移动平均滤波器初始化
void MA_Init(MovingAverage_t *ma)
{
    ma->index = 0;
    ma->sum = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        ma->buffer[i] = 0.0f;
    }
}

// 移动平均滤波
float Moving_Average(MovingAverage_t *ma, float value)
{
    // 更新和维护滑动总和
    ma->sum -= ma->buffer[ma->index];
    ma->sum += value;
    ma->buffer[ma->index] = value;
    ma->index = (ma->index + 1) % WINDOW_SIZE;

    return ma->sum / WINDOW_SIZE;
}

// 混合滤波
float Hybrid_Filter(float value, float coefficient, float max_rate)
{
    static LowPassFilter_t lp_filter = {0};
    static RateLimiter_t rate_limiter = {0};
    static MovingAverage_t ma = {0};
    static uint8_t init_flag = 0;

    if (!init_flag)
    {
        LowPass_Filter_Init(&lp_filter, coefficient);
        Rate_Limiter_Init(&rate_limiter, max_rate);
        MA_Init(&ma);
        init_flag = 1;
    }

    // 依次进行三重滤波
    float rate_limited = Rate_Limiter(&rate_limiter, value);
    float lp_filtered = LowPass_Filter(&lp_filter, rate_limited);
    return Moving_Average(&ma, lp_filtered);
}