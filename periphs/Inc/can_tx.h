#ifndef __CAN_TX_H__
#define __CAN_TX_H__

#include "can.h"
#include "string.h"

float Update_Yaw_Angle(float current_yaw, int yaw_mode);

//===============================================DJI电机===============================================//
HAL_StatusTypeDef Send_Yaw_Motor_Current(int16_t currents);
HAL_StatusTypeDef Send_Motor_Currents(CAN_HandleTypeDef *hcan, uint32_t can_id, int16_t currents[4]);
HAL_StatusTypeDef Set_Power_Limit(uint16_t power);

//===============================================达秒电机===============================================//
HAL_StatusTypeDef Send_DM_Motor_Command(float p_des, float v_des);
HAL_StatusTypeDef DM_Motor_Enable(void);
HAL_StatusTypeDef DM_Motor_Disable(void);
HAL_StatusTypeDef DM_Motor_Clear_error(void);

//===============================================MG电机===============================================//
HAL_StatusTypeDef Send_MultiTurn_Position_Control_Command(int32_t angleControl, int16_t maxSpeed);
HAL_StatusTypeDef MG_Motor_Shutdown(void);
HAL_StatusTypeDef MG_Motor_Stop(void);
HAL_StatusTypeDef MG_Motor_Run(void);


HAL_StatusTypeDef Send_Robot_Status(uint8_t robot_level, uint16_t heat_energy);

#endif
