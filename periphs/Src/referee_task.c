/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
// #include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "chassis_control.h"
#include <stdio.h>

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
Referee_Interactive_info_t ui_data;                  // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
UI_Task_Struct UI_Instance;

// @todo 不应该使用全局变量
static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID(void)
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;

    referee_info.referee_id.Robot_ID = referee_info.GameRobotState.robot_id;
    referee_info.referee_id.Cilent_ID = 0x0100 + referee_info.referee_id.Robot_ID; // 计算客户端ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}

void UITaskInit(void)
{
    referee_recv_info = Referee_Get_Instance();
    // referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = &ui_data; // 获取UI绘制需要的机器人状态数据
    referee_info.init_flag = 1;
}

void UI_Task_Timing(void)
{
    if (UI_Instance.Count > 0)
    {
        UI_Instance.Count--;
    }
}

void UI_Task(void)
{

    if (UI_Instance.Count > 0)
    {
        return;
    }

    switch (UI_Instance.Flag)
    {
    case 0:
        UI_Instance.Flag = 1;
        break;

    case 1:
        MyUIRefresh(referee_recv_info, Interactive_data);
        UI_Instance.Count = 500;
        break;

    default:
        break;
    }
}

// 全局图形数组
static Graph_Data_t UI_shoot_line[5];

// 绘制并调试十字的函数
// 参数：center_x, center_y 为十字中心坐标
void DrawCross(referee_id_t *referee_id, uint32_t center_x, uint32_t center_y, uint8_t first_draw)
{
    if (first_draw)
    {
        // 十字（4 条线）
        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 9, UI_Color_Green, 3, center_x - 15, center_y, center_x - 4, center_y); // 水平左
        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 9, UI_Color_Green, 3, center_x + 4, center_y, center_x + 15, center_y); // 水平右
        UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 9, UI_Color_Green, 3, center_x, center_y - 15, center_x, center_y - 4); // 垂直上
        UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 9, UI_Color_Green, 3, center_x, center_y + 4, center_x, center_y + 15); // 垂直下
    }
    // 十字无需动态更新，仅在首次绘制时推送
}

// 绘制并调试能量条的函数（7-24V 范围，长度 400 像素，动态颜色）
void DrawEnergyBar(referee_id_t *referee_id, uint32_t center_x, uint32_t start_y, uint32_t width, float voltage, uint8_t first_draw)
{
    // 电压范围 7-24V，映射到能量条长度 0-400 像素
    float voltage_normalized = voltage - 7.0f; // 将 7V 作为起点
    if (voltage_normalized < 0.0f)
        voltage_normalized = 0.0f;                                              // 小于 7V 时长度为 0
    uint32_t energy_length = (uint32_t)(voltage_normalized * (400.0f / 17.0f)); // 17V 范围 (24-7) 对应 400 像素
    if (energy_length > 400)
        energy_length = 400; // 上限 400 像素

    // 居中对齐：起点 x = 十字中心 x - 能量条最大长度/2
    uint32_t start_x = center_x - 200; // 400/2 = 200，中心点为 center_x

    // 根据电压值设置颜色
    uint32_t color;
    if (voltage > 20.0f)
    {
        color = UI_Color_Green; // 20V 以上绿色
    }
    else if (voltage >= 12.0f && voltage <= 20.0f)
    {
        color = UI_Color_Yellow; // 12-20V 黄色
    }
    else if (voltage >= 8.0f && voltage < 12.0f)
    {
        color = UI_Color_Purplish_red; // 8-12V 红色
    }
    else
    {
        color = UI_Color_Purplish_red; // 低于 8V 主色
    }

    if (first_draw)
    {
        UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 9, color, width, start_x, start_y, start_x, start_y);
    }
    else
    {
        UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_Change, 9, color, width, start_x, start_y, start_x + energy_length, start_y);
    }
}

void MyUIInit(void)
{
    // if (!referee_info.init_flag)
    //     vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务

    //     while (1)
    //     {
    //         if ((referee_info.GameRobotState.robot_id)!= 0x00)
    //				 {
    //					  break;
    //				 }
    //
    //     };
    while (((referee_info.GameRobotState.robot_id) == 0x00))
    {
        HAL_Delay(200);
    }

    DeterminRobotID(); // 确定ui要发送到的目标客户端
    HAL_Delay(200);

    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 0); // 清空UI

    HAL_Delay(100);

    // referee_id_t *referee_id = &referee_recv_info->referee_id; // 需确认字段名

    // 绘制并推送文字
    // UpdateUIDisplay(referee_id, chassis.capacitor.cap_voltage);
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    //     count++;
    //     if (count >= 50)
    //     {
    //         count = 0;
    //         count1++;
    //     }
    //     switch (count1 % 4)
    //     {
    //     case 0:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
    //         _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
    //         _Interactive_data->shoot_mode = SHOOT_ON;
    //         _Interactive_data->friction_mode = FRICTION_ON;
    //         _Interactive_data->lid_mode = LID_OPEN;
    //         _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
    //         if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
    //             _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
    //         break;
    //     }
    //     case 1:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_ROTATE;
    //         _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
    //         _Interactive_data->shoot_mode = SHOOT_OFF;
    //         _Interactive_data->friction_mode = FRICTION_OFF;
    //         _Interactive_data->lid_mode = LID_CLOSE;
    //         break;
    //     }
    //     case 2:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
    //         _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
    //         _Interactive_data->shoot_mode = SHOOT_ON;
    //         _Interactive_data->friction_mode = FRICTION_ON;
    //         _Interactive_data->lid_mode = LID_OPEN;
    //         break;
    //     }
    //     case 3:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //         _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
    //         _Interactive_data->shoot_mode = SHOOT_OFF;
    //         _Interactive_data->friction_mode = FRICTION_OFF;
    //         _Interactive_data->lid_mode = LID_CLOSE;
    //         break;
    //     }
    //     default:
    //         break;
    //     }
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);

    referee_id_t *referee_id = &referee_recv_info->referee_id; // 需确认字段名
    static uint8_t first_draw = 1;

    DrawCross(referee_id, 960, 460, first_draw);

    DrawEnergyBar(referee_id, 960, 250, 15, chassis.capacitor.cap_voltage, first_draw);
    // 推送 5 个图形
    UIGraphRefresh(referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    if (first_draw)
        first_draw = 0;
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
    //     _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    // }

    // if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
    //     _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    // }

    // if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    //     _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    // }

    // if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
    //     _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    // }

    // if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
    //     _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    // }

    // if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
    //     _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    // }
}
