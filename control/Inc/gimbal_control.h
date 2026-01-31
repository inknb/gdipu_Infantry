
#ifndef __GIMBAL_CONTROL_H__
#define __GIMBAL_CONTROL_H__

#include "main.h"

#include "dbus.h"
#include "filter.h"
#include "hwt906.h"
#include "can_tx.h"
#include "pid.h"
#include "rc_control.h"


#define FRICTION_SPEED_TOLERANCE 100 // 摩擦轮误差

#define FRICTION_SPEED_TARGET 6000 // 摩擦轮目标值

#define HEAT_PER_SHOT 10     // 单发子弹热量增量（单位：热量/发）
#define SAFE_MARGIN 10       // 安全余量
#define N_BULLETS_PER_REV 12 // 拨盘每转输送的子弹数

#define FEEDER_ACTIVE_THRESHOLD 10 // 拨盘激活转速阈值（RPM）
#define HEAT_INCREASE_THRESHOLD 5  // 热量增长阈值（连续N次未增长视为空仓）
#define DETECTION_WINDOW_MS 200    // 检测窗口时间（毫秒）
#define ENCODER_TO_ANGLE(x) ((float)(x) / 8192.0f * 360.0f)

#define YAW_ANGLE_LIMIT(min, max)      \
    const float YAW_ANGLE_MIN = (min); \
    const float YAW_ANGLE_MAX = (max)

// 发射电机状态
typedef enum
{
    MOTOR_STOPPED, // 停止
    MOTOR_RUNNING  // 运行
} MotorState_t;

typedef struct
{
    MotorState_t state; // 当前状态
    int16_t speed;      // 当前速度
    int16_t torque;
    int16_t targetSpeed;     // 目标速度
    pid_controler speed_pid; // 速度环PID控制器
} Motor_t;

// Yaw电机结构体
typedef struct
{
    int16_t current; // 当前电流
    float speed;     // 当前速度
    float angle;     // 当前角度

    float targetCurrent; // 目标电流
    float targetSpeed;   // 目标速度
    float targetAngle;   // 目标角度

    pid_controler current_pid; // 电流环PID控制器
    pid_controler speed_pid;   // 速度环PID控制器
    pid_controler angle_pid;   // 角度环PID控制
} YawMotor_t;

typedef struct
{
    Motor_t shooter_pid[3];   // 拨弹电机、左摩擦轮、右摩擦轮的PID数据
    int16_t shooter_force[3]; // 拨弹电机、左摩擦轮、右摩擦轮电流值
    uint8_t shooting_enabled; // 发射使能标志

    YawMotor_t yaw_motor; // Yaw电机
    int16_t yaw_force;    // 输出Yaw电机电流
} Gimbal_t;

#define FEEDER_MOTOR_INDEX 0
#define FRICTION_LEFT_MOTOR_INDEX 1
#define FRICTION_RIGHT_MOTOR_INDEX 2

extern Gimbal_t gimbal;
extern int16_t feeder_speed_target;

void Gimbal_Init(void);
void Handle_Shooter_Motors(uint32_t id, uint8_t *data);
void Update_Shooter_State(void);
void update_fire_rate_strategy(void);

#endif /*__ GIMBAL_CONTROL_H__ */
