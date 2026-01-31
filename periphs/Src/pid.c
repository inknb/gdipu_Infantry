#include "pid.h"

/**
 * @brief 清空 PID 控制器状态
 * @param pid 指向 PID 控制器结构体的指针
 */
static void PID_Reset(pid_controler *pid)
{
    pid->err = 0.0f;
    pid->d_err = 0.0f;
    pid->i_err = 0.0f;
    pid->output = 0.0f;
    pid->p_output = 0.0f;
    pid->i_output = 0.0f;
    pid->d_output = 0.0f;
}

/**
 * @brief 设置 PID 控制器增益
 * @param pid 指向 PID 控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static void PID_SetGains(pid_controler *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 初始化 PID 控制器
 * @param pid 指向 PID 控制器结构体的指针
 */
void PID_Init(pid_controler *pid)
{
    PID_Reset(pid);
    PID_SetGains(pid, 0.0f, 0.0f, 0.0f);
    pid->err_dz = 0.0f;
    pid->i_err_limit = 0.0f;
    pid->output_limit = 0.0f;
    pid->state = PID_DISABLE;
}

/**
 * @brief 设置 PID 控制器参数并重置状态
 * @param pid 指向 PID 控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param i_err_limit 最大积分误差
 * @param err_dz 误差死区
 * @param output_limit 输出限幅
 */
void PID_Set_Config(pid_controler *pid, float kp, float ki, float kd, float i_err_limit, float err_dz, float output_limit)
{
    PID_SetGains(pid, kp, ki, kd);
    pid->i_err_limit = i_err_limit;
    pid->err_dz = err_dz;
    pid->output_limit = output_limit;
    pid->state = PID_ENABLE;
    PID_Reset(pid);
}

/**
 * @brief 设置误差并计算 PID 输出
 * @param pid 指向 PID 控制器结构体的指针
 * @param err 当前误差
 * @return 计算后的 PID 输出值
 */
float PID_Set_Err(pid_controler *pid, float err)
{
    // if (pid->state == PID_DISABLE) // 如果 PID 被禁用，返回 0
    //     return 0.0f;

    if (fabsf(err) < pid->err_dz) // 处理误差死区
        err = 0.0f;

    // 更新误差
    pid->i_err += err;
    pid->d_err = err - pid->err;
    pid->err = err;

    // 积分误差限幅
    if (pid->i_err > pid->i_err_limit)
        pid->i_err = pid->i_err_limit;
    else if (pid->i_err < -pid->i_err_limit)
        pid->i_err = -pid->i_err_limit;

    // 计算各部分输出
    pid->p_output = pid->kp * pid->err;
    pid->i_output = pid->ki * pid->i_err;
    pid->d_output = pid->kd * pid->d_err;

    // 合成输出并限幅
    pid->output = pid->p_output + pid->i_output + pid->d_output;

    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;

    return pid->output;
}
