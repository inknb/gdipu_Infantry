#include "rc_control.h"
#include "custom_client.h" // [新增] 引入新的键鼠数据

Robot_State_t robot_state;
int16_t chassis_force[4];

// 为了不改变头文件声明，保留了原参，但在函数内部使用 custom_client_data
void Set_robot_Control(const _RC_Ctl_t *rc_input)
{
    // 默认设置为键鼠控制模式
    robot_state.control_mode = KB_CONTROL;

    // 按下 F 键 (bit 9) 进入小陀螺模式 [cite: 610]
    // 注意：如果是保持按下才转，用下面的逻辑；如果要按一下切换，需要做边沿检测
    if (custom_client_data.keyboard_value & (1 << 9)) 
    {
        robot_state.control_mode = SPIN_CONTROL;
    }

    // 默认不射击
    if (robot_state.control_mode == RC_CONTROL || robot_state.control_mode == SPIN_CONTROL || robot_state.control_mode == KB_CONTROL)
    {
        robot_state.control_mode = NO_SHOOT_MODE;
    }

    // 鼠标左键或右键按下时，触发射击模式 [cite: 621-622]
    if (custom_client_data.left_button_down || custom_client_data.right_button_down)
    {
        robot_state.control_mode = SHOOTING_MODE;
    }
}