#include "referee.h"
#include "gimbal_control.h"

// uint8_t reference_buffer[128] = {};
// ext_game_robot_status_t reference_game_robot_status;
// ext_power_heat_data_t reference_power_heat_data;

void reference_init()
{
	// __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
	// __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	// HAL_UART_Receive_DMA(&huart6, reference_buffer, sizeof(reference_buffer));
}

void reference_callback()
{
	// if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
	// {
	// 	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	// 	HAL_UART_DMAStop(&huart6);
	// 	extern DMA_HandleTypeDef hdma_usart6_rx;

	// 	uint16_t cmd_id = (reference_buffer[6] << 8) | reference_buffer[5];
	// 	uint16_t data_length = sizeof(reference_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
	// 	for (int i = 0; i < data_length; i++)
	// 	{
	// 		if (reference_buffer[i] == 0xA5)
	// 		{
	// 			cmd_id = (reference_buffer[i + 6] << 8) | reference_buffer[i + 5];
	// 			switch (cmd_id)
	// 			{
	// 			case 0x0201:
	// 				memcpy(&reference_game_robot_status, reference_buffer + i + 7, 13);
	// 				break;
	// 			case 0x0202:
	// 				memcpy(&reference_power_heat_data, reference_buffer + i + 7, 16);
	// 				break;
	// 			}
	// 		}
	// 	}

	// 	HAL_UART_Receive_DMA(&huart6, reference_buffer, sizeof(reference_buffer));
	// }
}
