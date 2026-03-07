/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2025-02-26 20:45:46
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-02 21:24:58
 * @FilePath: \RM_Hero_UP_Board\modules\referee\rm_referee.c
 */
#include "string.h"
#include "crc_ref.h"
#include "usart.h"
#include "rm_referee.h"
#include "custom_client.h" // [新增] 引入自定义客户端解析头文件

#define RE_RX_BUFFER_SIZE 255u			 
uint8_t RE_RX_Buffer[RE_RX_BUFFER_SIZE]; 
referee_info_t referee_info; 

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 */
static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; 
	if (buff == NULL)	   
		return;

	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	if (buff[SOF] == REFEREE_SOF)
	{
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				switch (referee_info.CmdID)
				{
				case ID_game_state: 
					memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
					break;
				case ID_game_result: 
					memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
					break;
				case ID_game_robot_survivors: 
					memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
					break;
				case ID_event_data: 
					memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
					break;
				case ID_supply_projectile_action: 
					memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
					break;
				case ID_game_robot_state: 
					memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
					break;
				case ID_power_heat_data: 
					memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
					break;
				case ID_game_robot_pos: 
					memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
					break;
				case ID_buff_musk: 
					memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
					break;
				case ID_aerial_robot_energy: 
					memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
					break;
				case ID_robot_hurt: 
					memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
					break;
				case ID_shoot_data: 
					memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
					break;
				case ID_student_interactive: 
					memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
					break;
                case 0x0311: // [新增] 自定义客户端发送给机器人的自定义指令 
                    // 把去掉帧头和CMD_ID的数据段传给硬解函数
                    parse_keyboard_mouse_control((buff + DATA_Offset), referee_info.FrameHeader.DataLength);
                    break;
				}
			}
		}
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ 
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

void RefereeInit(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RE_RX_Buffer, sizeof(RE_RX_Buffer)); 
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);						   
}

uint8_t *Referee_Get_Buffer(void)
{
	return RE_RX_Buffer;
}

referee_info_t *Referee_Get_Instance(void)
{
	return &referee_info;
}

void Referee_RX_Handle(uint8_t *Buffer, uint8_t size) 
{
	JudgeReadData(Buffer);
	RefereeInit();
}

void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	HAL_UART_Transmit_DMA(&huart6, send, tx_len);
}