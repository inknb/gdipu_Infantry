
#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include "main.h"
#include "usart.h"
#include "robot_status.h"
#include "can_tx.h"
#include "pid.h"

#define SQRT_2_OVER_2 0.70710678f // 斜对角速度分解系数（用于轮子计算）
#define PI 3.141592653589793238462643383279f

#define CHASSIS_MOTION_CURVE_COEFFICIENT 0.89f
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


typedef struct
{
    float current_speed;    // 当前实际速度
    float max_acceleration; // 最大加速度（用于加速）
    float max_deceleration; // 最大减速度（用于减速）
    float max_speed;        // 最大速度限制
} SpeedRampController;

// 运动学矩阵
static const float MECANUM_MATRIX[4][3] = {
    {-SQRT_2_OVER_2, SQRT_2_OVER_2, 1},  // 左前轮
    {SQRT_2_OVER_2, SQRT_2_OVER_2, 1},   // 右前轮
    {-SQRT_2_OVER_2, -SQRT_2_OVER_2, 1}, // 左后轮
    {SQRT_2_OVER_2, -SQRT_2_OVER_2, 1}   // 右后轮
};

typedef enum
{
    CHASSIS_INIT,      // 初始化状态
    CHASSIS_IDLE,      // 空闲状态
    CHASSIS_SENDING,   // 发送状态
    CHASSIS_RECEIVING, // 接收数据状态
    CHASSIS_COMPLETE   // 发送完成状态
} Chassis_Status_t;

// 底盘运动结构体
typedef struct
{
    float x;     // x方向速度
    float y;     // y方向速度
    float omega; // z方向角速度
} ChassisSpeed_t;

// 底盘运动结构体
typedef struct
{
    ChassisSpeed_t speed; // 底盘运动速度
    float angle;          // 底盘运动角度
} ChassisMotion_t;

// 电容系统结构体
typedef struct
{
    float input_voltage; // 输入电压
    float cap_voltage;   // 电容电压
    float input_current; // 输入电流
    float set_power;     // 设定的功率

    pid_controler cap_pid; // 电容PID控制器
} Capacitor_t;

// 底盘电机结构体
typedef struct
{
    int16_t speed;          // 当前速度
    int16_t chassis_target; // 目标速度

    pid_controler speed_pid; // 速度环PID控制器
} ChassisMotor_t;

typedef struct
{
    float target_angle;  // 目标角度
    float current_angle; // 当前角度

    pid_controler angle_pid; // 角度环PID控制器
} ChassisFollow_t;

// 底盘系统结构体
typedef struct
{
    ChassisMotor_t chassis_motors[4]; // 四个底盘电机
    Capacitor_t capacitor;            // 电容
    ChassisFollow_t follow_target;    // 底盘跟随云台的目标角度

    uint8_t chassis_id;
    ChassisMotion_t motion;
    int16_t chassis_force[4];
    int16_t speed_limit;
} Chassis_t;

extern Chassis_t chassis;

void Chassis_Init(void);

void Handle_Chassis_Motors();
void Handle_Supercap(uint8_t *data);

void Chassis_Control_Calculate(int16_t vx, int16_t vy, int16_t omega);
void Chassis_Receive(void);

#endif /* __CHASSIS_CONTROL_H__ */
