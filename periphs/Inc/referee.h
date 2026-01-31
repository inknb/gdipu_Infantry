#ifndef __REFERENCE_H__
#define __REFERENCE_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart.h"
#include <string.h>
#include "main.h"
#include "pid.h"

// #pragma pack(push, 1) // 强制1字节对齐，禁用编译器填充

// /**
//  * @brief 游戏状态数据结构体
//  * 协议命令码 0x0001，数据长度11字节
//  */
// typedef struct
// {
//     uint8_t game_type : 4;      // 偏移0，bit0-3
//     uint8_t game_progress : 4;  // 偏移0，bit4-7
//     uint16_t stage_remain_time; // 偏移1-2，单位秒
//     uint64_t SyncTimeStamp;     // 偏移3-10，UNIX时间戳（协议定义8字节）
// } ext_game_status_t;

// /**
//  * @brief 机器人状态数据结构体
//  * 协议命令码 0x0201，数据长度13字节
//  */
// typedef struct
// {
//     uint8_t robot_id;              // 偏移0，1字节
//     uint8_t robot_level;           // 偏移1，1字节
//     uint16_t remain_HP;            // 偏移2-3，当前血量
//     uint16_t max_HP;               // 偏移4-5，最大血量
//     uint16_t shooter_cooling_rate; // 偏移6-7，17mm枪口冷却速率
//     uint16_t shooter_heat_limit;   // 偏移8-9，17mm枪口热量上限
//     uint16_t chassis_power_limit;  // 偏移10-11，底盘功率限制
//     uint8_t gimbal_power : 1;      // 偏移12，bit0: 云台电源
//     uint8_t chassis_power : 1;     // 偏移12，bit1: 底盘电源
//     uint8_t shooter_power : 1;     // 偏移12，bit2: 射击电源
//     uint8_t reserved_bits : 5;     // 偏移12，bit3-7: 保留位（协议未定义）
// } ext_game_robot_status_t;

// /**
//  * @brief 底盘功率和枪口热量数据结构体
//  * 协议命令码 0x0202，数据长度16字节
//  */
// typedef struct
// {
//     uint16_t chassis_volt;         // 偏移0-1，电压（单位mV）
//     uint16_t chassis_current;      // 偏移2-3，电流（单位mA）
//     float chassis_power;           // 偏移4-7，功率（单位W）
//     uint16_t chassis_power_buffer; // 偏移8-9，缓冲能量（单位J）
//     uint16_t shooter_17mm_1_heat;  // 偏移10-11，17mm枪口1热量
//     uint16_t shooter_17mm_2_heat;  // 偏移12-13，17mm枪口2热量
//     uint16_t shooter_42mm_heat;    // 偏移14-15，42mm枪口热量
// } ext_power_heat_data_t;

// #pragma pack(pop) // 恢复默认对齐

// // 外部变量声明
// extern ext_game_robot_status_t reference_game_robot_status;
// extern ext_power_heat_data_t reference_power_heat_data;
// extern ext_game_status_t reference_game_status;

void reference_init();
void reference_callback();

#endif