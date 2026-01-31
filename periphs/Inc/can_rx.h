#ifndef __CAN_RX_H__
#define __CAN_RX_H__

#include "can.h"

typedef enum
{
    // 底盘电机 ID(CAN1)
    MOTOR_CHASSIS_1 = 0x201, // 左前电机
    MOTOR_CHASSIS_2 = 0x202, // 右前电机
    MOTOR_CHASSIS_3 = 0x203, // 左后电机
    MOTOR_CHASSIS_4 = 0x204, // 右后电机

    // 哨兵0x208， 步兵0x209
    MOTOR_YAW = 0x209, // yaw
    SUPERCAP = 0x211,  // 超电

    MOTOR_FEEDER = 0x205,         // 拨弹
    MOTOR_FRICTION_LEFT = 0x206,  // 左摩擦轮
    MOTOR_FRICTION_RIGHT = 0x207, // 右摩擦轮

} MotorID_t;

void Can_Init(void);
static void Configure_CAN_Filter(CAN_HandleTypeDef *hcan, uint32_t filterBank, uint32_t id, uint32_t mask, uint32_t fifo);
void Can_Handle_RxMessage(CAN_HandleTypeDef *can, uint16_t fifo);

#endif
