#include "math.h"

#ifndef PID_H
#define PID_H

/// @brief PID 状态枚举，用于指示 PID 控制器是启用还是禁用
typedef enum
{
    PID_DISABLE, ///< 禁用 PID 控制器
    PID_ENABLE   ///< 启用 PID 控制器
} PID_STATE;

/// @brief PID 控制器结构体定义
typedef struct pid_controler
{
    // 误差项
    float err;       // 当前误差值
    float err_dz;    // 误差死区阈值
    float err_limit; // 最大允许误差

    // 积分和微分误差
    float d_err;       // 误差的微分项
    float i_err;       // 误差的积分项
    float i_err_limit; // 最大允许积分误差

    // PID 系数
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数

    // 输出项
    float output;       // PID 控制器输出
    float output_limit; // 最大允许输出
    float p_output;     // 比例输出部分
    float i_output;     // 积分输出部分
    float d_output;     // 微分输出部分

    // PID 控制器状态
    PID_STATE state; // PID 控制器状态，启用或禁用
} pid_controler;

/// @brief 全局 PID 控制器指针
extern pid_controler *PID;

void PID_Init(pid_controler *pid);
void PID_Set_Config(pid_controler *pid, float kp, float ki, float kd, float i_err_limit, float err_dz, float output_limit);
float PID_Set_Err(pid_controler *pid, float err);

#endif // PID_H
