#include "can_tx.h"

HAL_StatusTypeDef Bsp_CAN_Transmit(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *pHeader,
                                   const uint8_t aData[], uint32_t *pTxMailbox)
{
    uint32_t TxMailboxX = CAN_TX_MAILBOX0; // CAN发送邮箱
    // 找到空的发送邮箱 把数据发送出去
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
    }; // 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲

    if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET)
    {
        // 检查发送邮箱0状态 如果邮箱0空闲
        TxMailboxX = CAN_TX_MAILBOX0;
    }
    else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET)
    {
        TxMailboxX = CAN_TX_MAILBOX1;
    }
    else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET)
    {
        TxMailboxX = CAN_TX_MAILBOX2;
    }
    return HAL_CAN_AddTxMessage(hcan, pHeader, aData, (uint32_t *)TxMailboxX);
}

HAL_StatusTypeDef Send_Yaw_Motor_Current(int16_t currents)
{
    uint8_t data[2];

    data[0] = (uint8_t)(currents >> 8);
    data[1] = (uint8_t)(currents & 0xFF);

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x2FF,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 2,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

HAL_StatusTypeDef Send_Robot_Status(uint8_t robot_level, uint16_t heat_energy)
{
    uint8_t data[3];

    data[0] = robot_level;
    data[1] = (uint8_t)(heat_energy >> 8);
    data[2] = (uint8_t)(heat_energy & 0xFF);

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x300,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 3,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

HAL_StatusTypeDef Send_Motor_Currents(CAN_HandleTypeDef *hcan, uint32_t can_id, int16_t currents[4])
{
    uint8_t data[8] = {0};

    for (uint8_t i = 0; i < 4; i++)
    {
        data[i * 2] = (uint8_t)(currents[i] >> 8);
        data[i * 2 + 1] = (uint8_t)(currents[i] & 0xFF);
    }

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = can_id,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(hcan, &tx_header, data, NULL);
}

HAL_StatusTypeDef Set_Power_Limit(uint16_t power)
{
    uint8_t data[2];
    data[0] = (uint8_t)(power >> 8);
    data[1] = (uint8_t)(power & 0xFF);

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x210,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 2,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan1, &tx_header, data, NULL);
}

//===============================================达秒电机===============================================//

HAL_StatusTypeDef Send_DM_Motor_Command(float p_des, float v_des)
{
    uint8_t data[8];

    // 将浮点数转换为字节数组，低位在前，高位在后
    memcpy(&data[0], &p_des, sizeof(float));
    memcpy(&data[4], &v_des, sizeof(float));

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x10C,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

HAL_StatusTypeDef DM_Motor_Enable(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x10C,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

HAL_StatusTypeDef DM_Motor_Clear_error(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x10C,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}


HAL_StatusTypeDef DM_Motor_Disable(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x10C,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

//===============================================MG电机===============================================//

HAL_StatusTypeDef Send_MultiTurn_Position_Control_Command(int32_t angleControl, int16_t maxSpeed)
{
    uint8_t data[8] = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    data[2] = (uint8_t)(maxSpeed & 0xFF);
    data[3] = (uint8_t)((maxSpeed >> 8) & 0xFF);

    data[4] = (uint8_t)(angleControl & 0xFF);
    data[5] = (uint8_t)((angleControl >> 8) & 0xFF);
    data[6] = (uint8_t)((angleControl >> 16) & 0xFF);
    data[7] = (uint8_t)((angleControl >> 24) & 0xFF);

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x141,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

HAL_StatusTypeDef MG_Motor_Shutdown(void)
{
    uint8_t data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x141,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

HAL_StatusTypeDef MG_Motor_Stop(void)
{
    uint8_t data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x141,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

HAL_StatusTypeDef MG_Motor_Run(void)
{
    uint8_t data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x141,
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

    return Bsp_CAN_Transmit(&hcan2, &tx_header, data, NULL);
}

//==============================================================================================//

float Update_Yaw_Angle(float current_yaw, int yaw_mode)
{
    static float previous_yaw = 0;
    static int rotation_count = 0;
    static float absolute_angle = 0;

    if (current_yaw - previous_yaw > 180)
    {
        rotation_count--;
    }
    else if (current_yaw - previous_yaw < -180)
    {
        rotation_count++;
    }

    absolute_angle = ((float)rotation_count * 360) + current_yaw;
    previous_yaw = current_yaw;

    float offset;
    switch (yaw_mode)
    {
    case 1: // 两基准点模式 (0°, 180°)
    {
        int base_point = (int)(absolute_angle / 180.0f) * 180;
        offset = absolute_angle - base_point;

        if (offset > 90.0f)
        {
            offset -= 180.0f;
        }
        else if (offset < -90.0f)
        {
            offset += 180.0f;
        }
        break;
    }
    case 2: // 四基准点模式 (0°, 90°, 180°, 270°)
    {
        int base_point = (int)(absolute_angle / 90.0f) * 90;
        offset = absolute_angle - base_point;
        if (offset > 45.0f)
        {
            offset -= 90.0f;
        }
        else if (offset < -45.0f)
        {
            offset += 90.0f;
        }
        break;
    }
    default: // 单一基准点模式 (0°)
        offset = absolute_angle;
        break;
    }

    return offset;
}