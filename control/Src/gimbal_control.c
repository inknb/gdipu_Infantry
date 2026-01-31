#include "gimbal_control.h"
#include "referee.h"
#include <math.h>
#include "rm_referee.h"

Gimbal_t gimbal;

volatile uint16_t current_heat; // 当前枪口热量（从0x0202解析）
volatile uint16_t cooling_rate; // 枪口冷却速率（从0x0201解析）
volatile uint16_t heat_max;     // 枪口热量上限

volatile uint16_t last_heat = 0;              // 上一次热量值
volatile uint16_t no_heat_increase_count = 0; // 热量未增长计数
volatile uint32_t last_detection_time = 0;    // 最后检测时间

// 控制参数
float fire_rate_target = 0.0f; // 目标射速（发/秒）
int16_t feeder_speed_target = 0;
float burst_time_sec = 0; // 爆发阶段时间（单位：秒）

/**
 * @brief 弹仓空检测（基于热量变化）
 * @return 1-无弹，0-有弹
 */
uint8_t check_ammo_empty(void)
{
    if (abs(gimbal.shooter_pid[0].speed) < 30 && no_heat_increase_count != 0)
    {
        return 0;
    }

    // 定时检测（避免频繁调用）
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_detection_time < DETECTION_WINDOW_MS)
    {
        return 0;
    }
    last_detection_time = current_time;

    // 计算热量变化
    int16_t delta_heat = current_heat - last_heat;
    last_heat = current_heat;

    // 热量未增长时计数
    if (delta_heat <= 0)
    {
        no_heat_increase_count++;
    }
    else
    {

        no_heat_increase_count = 0; // 重置计数器
    }

    // 连续N次未增长判定为无弹
    return (no_heat_increase_count >= 3);
}

/**
 * @brief 分阶段射速控制策略
 */
void update_fire_rate_strategy(void)
{
    //    // 弹仓空检测（优先级最高）
    //    if (check_ammo_empty())
    //    {
    //        fire_rate_target = 0.0f;
    //        feeder_speed_target = 0;
    //        no_heat_increase_count = 0; // 重置计数器

    //        return;
    //    }

    // 计算可用热量空间
    int32_t usable_heat = heat_max - current_heat - SAFE_MARGIN;

    // 过热保护
    if (usable_heat < 0)
    {
        fire_rate_target = 0.0f;
        feeder_speed_target = 0.0f;
        return;
    }

    // 爆发阶段策略
    if (usable_heat >= HEAT_PER_SHOT)
    {
        // 计算最大可爆发次数
        uint16_t max_burst_shots = usable_heat / HEAT_PER_SHOT;

        switch (referee_info.GameRobotState.robot_level)
        {
        case 1:
            burst_time_sec = 0.8;
            break;
        case 2:
            burst_time_sec = 0.8;
            break;
        case 3:
            burst_time_sec = 0.8;
            break;
        case 4:
            burst_time_sec = 0.8;
            break;
        case 5:
            burst_time_sec = 0.9;
            break;
        case 6:
            burst_time_sec = 0.9;
            break;
        case 7:
            burst_time_sec = 0.9;
            break;
        case 8:
            burst_time_sec = 1;
            break;
        case 9:
            burst_time_sec = 1;
            break;
        case 10:
            burst_time_sec = 1;
            break;
        default:
            burst_time_sec = 2;
            break;
        }
        // 计算爆发射速
        fire_rate_target = (float)max_burst_shots / burst_time_sec;
    }
    // 维持阶段策略
    else
    {
        // 可持续射速 = 冷却速率 / 单发热量增量
        fire_rate_target = (float)cooling_rate / HEAT_PER_SHOT;
        // fire_rate_target = 0;
    }

    // 1v1 100
    // 转换为拨盘转速（RPM = 发/秒 * 60秒 / 每转发数）
    feeder_speed_target = (uint16_t)((fire_rate_target * 60) / N_BULLETS_PER_REV) * 100;

    feeder_speed_target = (feeder_speed_target > 5000) ? 5000 : feeder_speed_target;
}

void Gimbal_Init(void)
{
    PID_Set_Config(&gimbal.shooter_pid[0].speed_pid, 7, 0.5, 0, 5000, 0, 10000);
    PID_Set_Config(&gimbal.shooter_pid[1].speed_pid, 8, 0.3, 0, 4000, 0.0f, 15000);
    PID_Set_Config(&gimbal.shooter_pid[2].speed_pid, 8, 0.3, 0, 4000, 0.0f, 15000);

    PID_Set_Config(&gimbal.yaw_motor.current_pid, 20, 0.5, 0, 5000, 0, 6000);

    PID_Set_Config(&gimbal.yaw_motor.speed_pid, 3000, 0, 0, 0, 0, 25000);
    PID_Set_Config(&gimbal.yaw_motor.angle_pid, 0.5, 0, 0, 0, 0, 2500);
}

void Handle_Shooter_Motors(uint32_t id, uint8_t *data)
{
    static uint8_t update_flag = 0;

    uint8_t motorIndex = id - 0x205;

    gimbal.shooter_pid[motorIndex].speed = (data[2] << 8) | data[3];
    gimbal.shooter_pid[motorIndex].torque = (data[4] << 8) | data[5];

    gimbal.shooter_force[motorIndex] = PID_Set_Err(
        &gimbal.shooter_pid[motorIndex].speed_pid,
        gimbal.shooter_pid[motorIndex].targetSpeed - gimbal.shooter_pid[motorIndex].speed);

    update_flag |= (1 << motorIndex);
    if ((update_flag & 0b0111) == 0b00111)
    {
        Set_Shooter_Data_Updated();
        update_flag = 0;
    }
}

static _Bool Check_Friction_Ready(void)
{
    return (abs(gimbal.shooter_pid[FRICTION_LEFT_MOTOR_INDEX].speed -
                gimbal.shooter_pid[FRICTION_LEFT_MOTOR_INDEX].targetSpeed) < FRICTION_SPEED_TOLERANCE) &&
           (abs(gimbal.shooter_pid[FRICTION_RIGHT_MOTOR_INDEX].speed -
                gimbal.shooter_pid[FRICTION_RIGHT_MOTOR_INDEX].targetSpeed) < FRICTION_SPEED_TOLERANCE);
}

void Set_Friction_Speed(int16_t speed)
{
    gimbal.shooter_pid[FRICTION_LEFT_MOTOR_INDEX].targetSpeed = speed;
    gimbal.shooter_pid[FRICTION_RIGHT_MOTOR_INDEX].targetSpeed = -speed;

    uint8_t motor_state = (speed == 0) ? MOTOR_STOPPED : MOTOR_RUNNING;
    gimbal.shooter_pid[FRICTION_LEFT_MOTOR_INDEX].state = motor_state;
    gimbal.shooter_pid[FRICTION_RIGHT_MOTOR_INDEX].state = motor_state;

    if (speed == 0)
    {
        gimbal.shooting_enabled = 0;
    }
}

void Set_Feeder_Speed(int16_t speed)
{
    gimbal.shooter_pid[FEEDER_MOTOR_INDEX].targetSpeed = -speed;

    if (!gimbal.shooting_enabled)
    {
        gimbal.shooter_pid[FEEDER_MOTOR_INDEX].state = MOTOR_STOPPED;
        gimbal.shooter_pid[FEEDER_MOTOR_INDEX].speed = 0;
        return;
    }

    gimbal.shooter_pid[FEEDER_MOTOR_INDEX].state =
        (speed == 0) ? MOTOR_STOPPED : MOTOR_RUNNING;
}

void Update_Shooter_State(void)
{
    if (robot_state.control_mode == SHOOTING_MODE)
    {
        Set_Friction_Speed(FRICTION_SPEED_TARGET);
    }
    else
    {
        Set_Friction_Speed(0);
    }

    current_heat = referee_info.PowerHeatData.shooter_heat0_17mm;
    cooling_rate = referee_info.GameRobotState.shooter_barrel_cooling_value;
    heat_max = referee_info.GameRobotState.shooter_barrel_heat_limit;

    update_fire_rate_strategy();

    gimbal.shooting_enabled = Check_Friction_Ready() &&
                              gimbal.shooter_pid[FRICTION_LEFT_MOTOR_INDEX].targetSpeed != 0;

    if (gimbal.shooting_enabled && robot_state.control_mode == SHOOTING_MODE)
    {
        if (RC_Ctl.rc.s1 == 1)
        {
            if (RC_Ctl.mouse.left_key)
            {
                Set_Feeder_Speed(feeder_speed_target);
            }
            else if (RC_Ctl.mouse.right_key)
            {
                Set_Feeder_Speed(-feeder_speed_target * 0.1);
            }
        }
        else
        {

            Set_Feeder_Speed(RC_Ctl.rc.s2 == 3 ? feeder_speed_target : (-feeder_speed_target * 0.1));
        }
    }
    else
    {
        Set_Feeder_Speed(0);
        gimbal.shooter_force[0] = 0;
    }
}
