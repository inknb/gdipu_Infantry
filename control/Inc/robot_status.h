#ifndef __ROBOT_STATUS_H__
#define __ROBOT_STATUS_H__

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "hwt906.h"
#include "dbus.h"

#define DEBUG_OUTPUT_MODE 0 // 0使用SEGGER RTT打印 1使用串口打印
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define INCREMENT_FLAG(flag, threshold) \
    do                                  \
    {                                   \
        (flag)++;                       \
        if ((flag) >= (threshold))      \
            (flag) = 0;                 \
    } while (0)


#define PITCH_MIN              (-1.1f)     // 最小俯仰角（rad）
#define PITCH_MAX              (0.0f)      // 最大俯仰角（rad）
#define PITCH_SPEED            (55.0f)     // 电机转速
#define PITCH_INCREMENT        (0.00005f)  // 摇杆控制步长
#define MOUSE_SENSITIVITY      (0.00016f)  // 鼠标灵敏度
#define TARGET_CHANGE_THRESH   (0.001f)    // 目标值变化阈值

// 云台状态
typedef enum
{
    YAW_INIT,    // 初始化状态
    YAW_IDLE,    // 空闲状态
    YAW_SENDING, // 发送状态
    YAW_COMPLETE // 发送完成状态
} Yaw_Status_t;

// 俯仰状态
typedef enum
{
    PITCH_INIT,    // 初始化状态
    PITCH_IDLE,    // 空闲状态
    PITCH_SENDING, // 发送状态
    PITCH_COMPLETE // 发送完成状态
} Pitch_Status_t;

// 射击状态
typedef enum
{
    SHOOTER_INIT,    // 初始化状态
    SHOOTER_IDLE,    // 空闲状态
    SHOOTER_SENDING, // 发送状态
    SHOOTER_COMPLETE // 发送完成状态
} Shooter_Status_t;

// 遥控器状态
typedef enum
{
    RC_STATE_INIT,         // 系统初始化
    RC_STATE_IDLE,         // 空闲状态，等待新数据
    DMA_STARTUP_FAILURE,   // DMA 初始化失败
    RC_STATE_ERROR,        // 数据解析错误
    RC_STATE_OK,           // 数据正常接收
    RC_STATE_DISCONNECTED, // 遥控器信号丢失
    RC_STATE_PROCESSING,   // 正在处理数据
    RC_STATE_COMPLETE      // 数据处理完成
} RC_State_t;

extern RC_State_t rc_state;

extern uint32_t robot_rc_flag;
extern uint32_t robot_pitch_flag;
extern uint32_t robot_print_flag;
extern uint32_t robot_shooter_flag;
extern uint32_t robot_chassis_flag;


void robot_init(void);
void Set_Shooter_Data_Updated(void);
void Update_Robot_Status(void);

#endif
