#include "robot_status.h"
#include "gimbal_control.h"
#include "chassis_control.h"
#include "rc_control.h"
#include "referee.h"
#include "can_rx.h"
#include "can_tx.h"
#include "hwt906.h"
#include "rm_referee.h"
#include "referee_task.h"
// 阈值定义
#define RC_FLAG_THRESHOLD 10
#define YAW_FLAG_THRESHOLD 30
#define PITCH_FLAG_THRESHOLD 30

#define SHOOTER_FLAG_THRESHOLD 70
#define CHASSIS_FLAG_THRESHOLD 200
#define CHASSIS_POWER_FLAG_THRESHOLD 100

#define PRINTF_FLAG_THRESHOLD 20

// 系统状态枚举变量

Pitch_Status_t pitch_status = PITCH_INIT;
RC_State_t rc_state = RC_STATE_INIT;
Shooter_Status_t shooter_status = SHOOTER_INIT;

// 计数标志位
uint32_t robot_rc_flag = 0;
uint32_t robot_yaw_flag = 0;
uint32_t robot_pitch_flag = 0;
uint32_t robot_print_flag = 0;
uint32_t robot_shooter_flag = 0;
uint32_t robot_chassis_flag = 0;
int32_t robot_chassis_power_flag = 0;

void robot_init(void)
{
    Can_Init();

    DM_Motor_Enable();
    Send_DM_Motor_Command(-0.4, 20);

    Dbus_Init();
    hwt906_init();
    // reference_init();

    Gimbal_Init();
    Chassis_Init();

    // SEGGER_RTT_Init();

    RefereeInit();
    UITaskInit();
    //MyUIInit();

    Set_Power_Limit(4500);

    MX_WWDG_Init();
}

//=========================================定时器=========================================//

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        INCREMENT_FLAG(robot_rc_flag, RC_FLAG_THRESHOLD);
        INCREMENT_FLAG(robot_pitch_flag, PITCH_FLAG_THRESHOLD);
        INCREMENT_FLAG(robot_yaw_flag, YAW_FLAG_THRESHOLD);
        INCREMENT_FLAG(robot_shooter_flag, SHOOTER_FLAG_THRESHOLD);
        INCREMENT_FLAG(robot_print_flag, PRINTF_FLAG_THRESHOLD);
        INCREMENT_FLAG(robot_chassis_flag, CHASSIS_FLAG_THRESHOLD);
        INCREMENT_FLAG(robot_chassis_power_flag, CHASSIS_POWER_FLAG_THRESHOLD);
        UI_Task_Timing();
    }
}
//=========================================遥控器=========================================//

static void Update_rc_state(void)
{
    if (robot_rc_flag)
    {
        return;
    }
    robot_rc_flag = RC_FLAG_THRESHOLD;

    switch (rc_state)
    {
    case RC_STATE_PROCESSING:
        if (_Parse_RC_Data())
        {
            Set_robot_Control(&RC_Ctl);
            rc_state = RC_STATE_COMPLETE;
            HAL_WWDG_Refresh(&hwwdg);
        }
        break;
    default:
        break;
    }
}

//=========================================pitch云台=========================================//

static void Update_pitch_status(void)
{

    static float pitch_target = -0.4f;      // 目标俯仰角（初始中间位置）
    static float last_pitch_target = -0.4f; // 上一次目标值（用于变化检测）
    static uint8_t prev_state = 0;          // 上一次电源管理状态

    //------------------------ 电源管理触发逻辑 ------------------------
    // 当 power_management_gimbal_output 从 0→1 时触发电机使能
    const uint8_t current_state = referee_info.GameRobotState.power_management_gimbal_output;
    if (current_state == 1 && prev_state == 0)
    {
        DM_Motor_Enable();
    }
    prev_state = current_state; // 更新状态记录

    //------------------------ 键盘强制复位逻辑 ------------------------
    if (RC_Ctl.keyboard.q)
    { // 按下 Q 键时复位到中间位置
        DM_Motor_Enable();
        DM_Motor_Clear_error();
        pitch_target = -0.4f;
    }

    //------------------------ 输入控制逻辑 ----------------------------
    // 根据输入设备更新目标俯仰角
    if (RC_Ctl.rc.s1 == 1)
    {
        // 鼠标控制模式
        pitch_target += RC_Ctl.mouse.y * MOUSE_SENSITIVITY;
    }
    else
    {
        // 摇杆控制模式
        pitch_target += -RC_Ctl.rc.ch3 * PITCH_INCREMENT;
    }

    //------------------------ 目标值限幅 ------------------------------
    pitch_target = LIMIT(pitch_target, PITCH_MIN, PITCH_MAX);

    //------------------------ 发送电机命令 ----------------------------
    // 仅当目标值变化超过阈值时发送命令（减少通信负载）
    if (fabsf(pitch_target - last_pitch_target) > TARGET_CHANGE_THRESH)
    {
        Send_DM_Motor_Command(pitch_target, PITCH_SPEED);
        last_pitch_target = pitch_target; // 更新记录值
    }
}
//=========================================yaw云台=============================================//

void Update_yaw_status(void)
{
    if (robot_yaw_flag)
    {
        return;
    }
    static uint8_t first_call = 1;
    static int16_t yaw_rotation_count = 0;
    static float previous_yaw = 0;
    static float continuous_yaw = 0;

    float yaw_difference = gyro.yaw - previous_yaw;
    if (yaw_difference > 180.0f)
    {
        yaw_rotation_count -= 1;
    }
    else if (yaw_difference < -180.0f)
    {
        yaw_rotation_count += 1;
    }
    continuous_yaw = yaw_rotation_count * 360.0f + gyro.yaw;
    previous_yaw = gyro.yaw;

    gimbal.yaw_motor.angle = continuous_yaw;
    robot_yaw_flag = YAW_FLAG_THRESHOLD;

    if (first_call)
    {
        gimbal.yaw_motor.targetAngle = gimbal.yaw_motor.angle;
        first_call = 0;
    }
    // 计算目标角度增量
    float target_angle_increment = (RC_Ctl.rc.s1 == 1) ? -RC_Ctl.mouse.x : -RC_Ctl.rc.ch2 * 0.4;
    gimbal.yaw_motor.targetAngle += target_angle_increment;

    PID_Set_Err(&gimbal.yaw_motor.angle_pid,
                gimbal.yaw_motor.targetAngle - gimbal.yaw_motor.angle);

    gimbal.yaw_motor.speed = 0.1f * gyro.yaw_rate;
    gimbal.yaw_force = PID_Set_Err(&gimbal.yaw_motor.speed_pid,
                                   gimbal.yaw_motor.angle_pid.output - gimbal.yaw_motor.speed);

    // 发送电流命令
    Send_Yaw_Motor_Current(gimbal.yaw_force);
}
//=========================================射击器=============================================//

static void Update_shooter_status(void)
{
    if (robot_shooter_flag)
    {
        return;
    }
    robot_shooter_flag = SHOOTER_FLAG_THRESHOLD;

    switch (shooter_status)
    {
    case SHOOTER_INIT:

        shooter_status = SHOOTER_IDLE;
        break;

    case SHOOTER_IDLE:
        break;

    case SHOOTER_SENDING:

        Update_Shooter_State();
        if (Send_Motor_Currents(&hcan2, 0x1ff, gimbal.shooter_force) == HAL_OK)
        {
            shooter_status = SHOOTER_COMPLETE;
        }
        break;

    case SHOOTER_COMPLETE:
        shooter_status = SHOOTER_IDLE;
        break;
    }
}

//=========================================底盘功率=========================================//

void Control_Chassis_Power(void)
{

    if (robot_chassis_power_flag)
    {
        robot_chassis_power_flag = CHASSIS_POWER_FLAG_THRESHOLD;

        int power_limit = 10000; // 默认功率限制

        if (referee_info.GameRobotState.chassis_power_limit != chassis.capacitor.set_power)
        {
            Set_Power_Limit(referee_info.GameRobotState.chassis_power_limit * 100);
        }
    }
}

//=========================================打印=============================================//

static void Update_print_status(void)
{
    if (robot_print_flag)
    {
        return;
    }

    // Safe_Printf("%d,%d\r\n", gimbal.shooter_pid[0].speed, -feeder_speed_target);
}

//=========================================主函数===========================================//

void Update_Robot_Status(void)
{

    Update_rc_state();

    Update_print_status();
    if (rc_state == RC_STATE_COMPLETE)
    {
        Handle_Chassis_Motors();
        Update_pitch_status();
        Update_yaw_status();
        Update_shooter_status();
        Control_Chassis_Power();
        UI_Task();
    }
}

void Set_Shooter_Data_Updated(void)
{
    shooter_status = SHOOTER_SENDING;
}
