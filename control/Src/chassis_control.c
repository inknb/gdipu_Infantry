#include "chassis_control.h"
#include "gimbal_control.h"
#include "rc_control.h"
#include "filter.h"
#include "math.h"

Chassis_t chassis;
Chassis_Status_t chassis_status = CHASSIS_INIT;

SpeedRampController x_ramp = {0};
SpeedRampController y_ramp = {0};
SpeedRampController gyro_ramp = {0};

/**
 * @brief 应用速度渐变控制
 * @param ramp 速度渐变控制器
 * @param target_speed 目标速度
 * @param dt 时间间隔（秒）
 * @return 更新后的当前速度
 */
float Apply_Speed_Ramp(SpeedRampController *ramp, float target_speed, float dt)
{
    // 计算速度差值
    float speed_diff = target_speed - ramp->current_speed;

    // 判断是加速还是减速
    float max_accel = (speed_diff > 0) ? ramp->max_acceleration : ramp->max_deceleration;
  
    // 应用加速度限制
    float allowed_diff = copysignf(
        fminf(fabsf(speed_diff), max_accel * dt),
        speed_diff);

    // 更新当前速度
    ramp->current_speed += allowed_diff;

    // 应用速度限制
    ramp->current_speed = CLAMP(ramp->current_speed, -ramp->max_speed, ramp->max_speed);

    return ramp->current_speed;
}

void Chassis_Init(void)
{
    PID_Set_Config(&chassis.chassis_motors[0].speed_pid, 15, 0.8, 0, 8000, 0, 16000);
    PID_Set_Config(&chassis.chassis_motors[1].speed_pid, 15, 0.8, 0, 8000, 0, 16000);
    PID_Set_Config(&chassis.chassis_motors[2].speed_pid, 15, 0.8, 0, 8000, 0, 16000);
    PID_Set_Config(&chassis.chassis_motors[3].speed_pid, 15, 0.8, 0, 8000, 0, 16000);

    PID_Set_Config(&chassis.capacitor.cap_pid, 500, 0.5, 0, 4000, 0, 10000);
    PID_Set_Config(&chassis.follow_target.angle_pid, 30, 0, 0, 0, 2, 8000);

    x_ramp.max_acceleration = 15000.0f; // 加速度
    x_ramp.max_deceleration = 10000.0f; // 减速度
    x_ramp.max_speed = 2000.0f;         // 总速度

    y_ramp.max_acceleration = 15000.0f; // 加速度
    y_ramp.max_deceleration = 10000.0f; // 减速度
    y_ramp.max_speed = 2000.0f;         // 总速度

    gyro_ramp.max_acceleration = 10000; // 加速度
    gyro_ramp.max_deceleration = 15000; // 减速度
    gyro_ramp.max_speed = 6000.0f;      // 总速度 (小陀螺模式下可能需要更高上限)
}

void Handle_Chassis_Motors(void)
{
    /* 底盘使能检查 */
    if (!robot_chassis_flag)
        return;
    robot_chassis_flag = 20;

    /* 计算时间差 */
    static uint32_t last_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - last_tick) / 1000.0f; // 转换为秒
    last_tick = current_tick;

    /* 底盘状态检查 */
    if (chassis_status != CHASSIS_RECEIVING)
        return;

    /* 角度计算模块 & 45度偏置修正 [泺泺修改点 1] */
    // 这里的 45.0f 是修正麦轮安装或坐标系定义导致的偏转
    // 如果发现改成往左偏了，请尝试改成 -45.0f
    #define GIMBAL_MOUNT_OFFSET 45.0f 

    // 叠加偏置角
    float angle_offset = chassis.motion.angle + GIMBAL_MOUNT_OFFSET; 
    
    float yaw = Update_Yaw_Angle(angle_offset, 1);
    float normalized_yaw = fmodf(angle_offset, 360.0f);
    if (normalized_yaw < 0)
        normalized_yaw += 360.0f;

    const float current_yaw_rad = normalized_yaw * PI / 180.0f;
    const float cos_yaw = cosf(current_yaw_rad);
    const float sin_yaw = sinf(current_yaw_rad);

    PID_Set_Err(&chassis.follow_target.angle_pid, yaw);
    
    /* 输入信号处理 */
    float target_x = 0.0f, target_y = 0.0f;

    /* 模式切换 */
    switch (RC_Ctl.rc.s1)
    {
    case 1: // 键盘模式
        target_x = (RC_Ctl.keyboard.d - RC_Ctl.keyboard.a) * x_ramp.max_speed;
        target_y = (RC_Ctl.keyboard.s - RC_Ctl.keyboard.w) * y_ramp.max_speed;

        if (RC_Ctl.keyboard.shift)
        {
            chassis.motion.speed.omega = Apply_Speed_Ramp(&gyro_ramp, 2500, dt);
        }
        else if (RC_Ctl.keyboard.ctrl)
        {
            chassis.motion.speed.omega = Apply_Speed_Ramp(&gyro_ramp, 4000, dt);
        }
        else
        {
            chassis.motion.speed.omega = Apply_Speed_Ramp(&gyro_ramp, chassis.follow_target.angle_pid.output, dt);
        }
        break;

    case 2: // 云台跟随模式
    {
        target_x = RC_Ctl.rc.ch0 * x_ramp.max_speed;
        target_y = -RC_Ctl.rc.ch1 * y_ramp.max_speed;
        
        // 设定小陀螺旋转速度，这里给 4000 (约中速)
        // 使用 Apply_Speed_Ramp 防止启动电流过大
        float spin_speed = 4000.0f; 
        chassis.motion.speed.omega = Apply_Speed_Ramp(&gyro_ramp, spin_speed, dt);
        break;
    }

    case 3: // 小陀螺模式
        target_x = RC_Ctl.rc.ch0 * x_ramp.max_speed;
        target_y = -RC_Ctl.rc.ch1 * y_ramp.max_speed;
        chassis.motion.speed.omega = Apply_Speed_Ramp(&gyro_ramp, chassis.follow_target.angle_pid.output, dt);
        break;

    default:
        return; // 无效模式，不做处理
    }

    target_x = Apply_Speed_Ramp(&x_ramp, target_x, dt);
    target_y = Apply_Speed_Ramp(&y_ramp, target_y, dt);

    /* 运动学解算 (Vector Decomposition) */
    // 使用修正后的 yaw 角进行解算，确保车头方向与云台对齐
    chassis.motion.speed.x = cos_yaw * target_y - sin_yaw * target_x;
    chassis.motion.speed.y = sin_yaw * target_y + cos_yaw * target_x;

    /* 电机控制计算 */
    Chassis_Control_Calculate(chassis.motion.speed.x, chassis.motion.speed.y, chassis.motion.speed.omega);

    static uint8_t update_flag = 0;
    update_flag |= 1 << chassis.chassis_id;
    if ((update_flag & 0b1111) == 0b1111)
    {
        if (Send_Motor_Currents(&hcan1, 0x200, chassis.chassis_force) == HAL_OK)
        {
            chassis_status = CHASSIS_COMPLETE;
        }
        update_flag = 0;
    }
}

void Handle_Supercap(uint8_t *data)
{
    // chassis.capacitor.input_voltage = (float)((data[1] << 8) | data[0]) / 100.0f;
    chassis.capacitor.cap_voltage = (float)((data[3] << 8) | data[2]) / 100.0f;
    // chassis.capacitor.input_current = (float)((data[5] << 8) | data[4]) / 100.0f;
    chassis.capacitor.set_power = (float)((data[7] << 8) | data[6]) / 100.0f;

    PID_Set_Err(&chassis.capacitor.cap_pid,
                12 - chassis.capacitor.cap_voltage);

    x_ramp.max_speed = -chassis.capacitor.cap_pid.output;
    y_ramp.max_speed = -chassis.capacitor.cap_pid.output;
}

void Chassis_Receive(void)
{
    chassis_status = CHASSIS_RECEIVING;
}

void Chassis_Control_Calculate(int16_t vx, int16_t vy, int16_t omega)
{
    for (int i = 0; i < 4; i++)
    {
        chassis.chassis_motors[i].chassis_target = (int16_t)(vx * MECANUM_MATRIX[i][0] +
                                                             vy * MECANUM_MATRIX[i][1] +
                                                             omega * MECANUM_MATRIX[i][2]);
    }
}