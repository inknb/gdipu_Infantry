
#ifndef __RC_CONTROL_H__
#define __RC_CONTROL_H__

#include "chassis_control.h"


// 机器人控制模式枚举
typedef enum
{
    RC_CONTROL,    // 遥控器控制模式
    KB_CONTROL,    // 键盘控制模式
    SPIN_CONTROL,  // 小陀螺控制模式
    SHOOTING_MODE, // 发射模式
    NO_SHOOT_MODE  // 不发射模式
} Robot_Control_Mode_t;

// 机器人控制状态
typedef struct
{
    Robot_Control_Mode_t control_mode; // 当前控制模式
} Robot_State_t;

extern Robot_State_t robot_state;

void Set_robot_Control(const _RC_Ctl_t *rc_input);

#endif /*__ RC_CONTROL_H__ */
