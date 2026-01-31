#include "can_rx.h"
#include "chassis_control.h"
#include "gimbal_control.h"

void Can_Init(void)
{
    // 配置 CAN1 过滤器
    Configure_CAN_Filter(&hcan1, 0, MOTOR_CHASSIS_1, 0x7FF, CAN_FILTER_FIFO0);
    Configure_CAN_Filter(&hcan1, 1, MOTOR_CHASSIS_2, 0x7FF, CAN_FILTER_FIFO0);
    Configure_CAN_Filter(&hcan1, 2, MOTOR_CHASSIS_3, 0x7FF, CAN_FILTER_FIFO0);
    Configure_CAN_Filter(&hcan1, 3, MOTOR_CHASSIS_4, 0x7FF, CAN_FILTER_FIFO0);
    Configure_CAN_Filter(&hcan1, 4, SUPERCAP, 0x7FF, CAN_FILTER_FIFO0);

    // 配置 CAN2 过滤器
    Configure_CAN_Filter(&hcan2, 14, MOTOR_YAW, 0x7FF, CAN_FILTER_FIFO0);
    Configure_CAN_Filter(&hcan2, 15, MOTOR_FEEDER, 0x7FF, CAN_FILTER_FIFO0);
    Configure_CAN_Filter(&hcan2, 16, MOTOR_FRICTION_LEFT, 0x7FF, CAN_FILTER_FIFO0);
    Configure_CAN_Filter(&hcan2, 17, MOTOR_FRICTION_RIGHT, 0x7FF, CAN_FILTER_FIFO0);

    // 启动 CAN1
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    // 启动 CAN2
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    // 激活中断通知
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
}

static void Configure_CAN_Filter(CAN_HandleTypeDef *hcan, uint32_t filterBank, uint32_t id, uint32_t mask, uint32_t fifo)
{
    CAN_FilterTypeDef filterConfig = {0};

    filterConfig.FilterBank = filterBank;             // 过滤器银行
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // ID掩码模式
    filterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // 16位过滤器
    filterConfig.FilterIdHigh = id << 5;              // ID 高16位
    filterConfig.FilterMaskIdHigh = mask << 5;        // 掩码高16位
    filterConfig.FilterIdLow = 0x0000;                // ID 低16位
    filterConfig.FilterMaskIdLow = 0x0000;            // 掩码低16位
    filterConfig.FilterFIFOAssignment = fifo;         // 指定 FIFO
    filterConfig.FilterActivation = ENABLE;           // 启用过滤器

    if (HAL_CAN_ConfigFilter(hcan, &filterConfig) != HAL_OK)
    {
        Error_Handler(); // 错误处理
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Can_Handle_RxMessage(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Can_Handle_RxMessage(hcan, CAN_RX_FIFO1);
}

void Can_Handle_RxMessage(CAN_HandleTypeDef *hcan, uint16_t fifo)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, fifo, &rxHeader, data) != HAL_OK)
    {
        return;
    }

    if (hcan->Instance == CAN1)
    {
        if (fifo == CAN_RX_FIFO0)
        {
            if (rxHeader.StdId == MOTOR_CHASSIS_1 || rxHeader.StdId == MOTOR_CHASSIS_2 ||
                rxHeader.StdId == MOTOR_CHASSIS_3 || rxHeader.StdId == MOTOR_CHASSIS_4)
            {
                chassis.chassis_id = rxHeader.StdId - 0x201;
                chassis.chassis_motors[chassis.chassis_id].speed = (data[2] << 8) | data[3];

                chassis.chassis_force[chassis.chassis_id] = PID_Set_Err(
                    &chassis.chassis_motors[chassis.chassis_id].speed_pid,
                    chassis.chassis_motors[chassis.chassis_id].chassis_target - chassis.chassis_motors[chassis.chassis_id].speed);

                Chassis_Receive();
            }
            if (rxHeader.StdId == SUPERCAP)
            {
                Handle_Supercap(data);
            }
        }
        else if (fifo == CAN_RX_FIFO1)
        {
        }
    }

    else if (hcan->Instance == CAN2)
    {
        if (fifo == CAN_RX_FIFO0)
        {
            if (rxHeader.StdId == MOTOR_YAW)
            {
                chassis.motion.angle = ENCODER_TO_ANGLE((data[0] << 8) | data[1]);
                // Handle_Yaw_Motor(data);
            }
            if (rxHeader.StdId == MOTOR_FRICTION_LEFT ||
                rxHeader.StdId == MOTOR_FRICTION_RIGHT ||
                rxHeader.StdId == MOTOR_FEEDER)
            {
                Handle_Shooter_Motors(rxHeader.StdId, data);
            }
        }
        else if (fifo == CAN_RX_FIFO1)
        {
        }
    }
}
