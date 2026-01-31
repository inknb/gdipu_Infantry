#include "hwt906.h"
#include "usart.h"
#include "rm_referee.h"

// HWT606 维特标准协议常量
#define WIT_HEADER      0x55
#define WIT_ANG_RATE    0x52  // 角速度包标识
#define WIT_ANGLE       0x53  // 角度包标识
#define RX_BUFFER_SIZE  256   // DMA缓冲区大小

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

Gyro gyro = {0};                   // 全局姿态数据
uint8_t rx_buffer[RX_BUFFER_SIZE];  // DMA接收缓冲区

/**
 * @brief 初始化陀螺仪 (函数名保持不变)
 */
void hwt906_init(void)
{
    memset(&gyro, 0, sizeof(Gyro));

    // 启用带IDLE中断的DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
    // 禁用半传输中断
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/**
 * @brief 数据解析函数 (针对HWT606维特协议修改)
 */
void ParseGyroData(uint8_t* data, uint16_t size) {
    // HWT606每个数据包固定11字节
    for (uint16_t i = 0; i <= (size >= 11 ? size - 11 : 0); i++) {
        
        // 1. 搜索帧头 0x55
        if (data[i] == WIT_HEADER) {
            
            // 2. 累加和校验：前10字节之和的低8位应等于第11字节
            uint8_t sum = 0;
            for (uint8_t j = 0; j < 10; j++) {
                sum += data[i + j];
            }
            
            if (sum != data[i + 10]) {
                continue; // 校验失败，寻找下一个字节
            }

            // 3. 根据包类型解析数据
            switch (data[i + 1]) {
                case WIT_ANG_RATE: // 角速度包
                {
                    // 转换公式: 速度 = ((short)(High<<8 | Low) / 32768.0) * 2000
                    short roll = (short)((data[i+3] << 8) | data[i+2]);
                    short pitch = (short)((data[i+5] << 8) | data[i+4]);
                    short yaw = (short)((data[i+7] << 8) | data[i+6]);
                    
                    gyro.roll_rate  = roll / 32768.0f * 2000.0f;
                    gyro.pitch_rate = pitch / 32768.0f * 2000.0f;
                    gyro.yaw_rate   = yaw / 32768.0f * 2000.0f;
                    break;
                }

                case WIT_ANGLE: // 角度包
                {
                    // 转换公式: 角度 = ((short)(High<<8 | Low) / 32768.0) * 180
                    short wx  = (short)((data[i+3] << 8) | data[i+2]);
                    short wy = (short)((data[i+5] << 8) | data[i+4]);
                    short wz   = (short)((data[i+7] << 8) | data[i+6]);
                    
                    gyro.roll  = wx  / 32768.0f * 180.0f;
                    gyro.pitch = wy / 32768.0f * 180.0f;
                    gyro.yaw   = wz   / 32768.0f * 180.0f;
                    break;
                }
                
                default:
                    break;
            }
            
            i += 10; // 解析成功，跳过这11个字节
        }
    }
}

/**
 * @brief 串口接收完成回调函数
 * @param huart 串口句柄
 * @param Size 接收到的数据大小
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        ParseGyroData(rx_buffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    else if (huart->Instance == USART6)
    {
        Referee_RX_Handle(Referee_Get_Buffer(), Size);
    }
}