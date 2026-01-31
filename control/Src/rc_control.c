#include "rc_control.h"

Robot_State_t robot_state;

int16_t chassis_force[4];

void Set_robot_Control(const _RC_Ctl_t *rc_input)
{

    switch (rc_input->rc.s1)
    {
    case 1:
        robot_state.control_mode = KB_CONTROL;
        break;
    case 3:
        robot_state.control_mode = RC_CONTROL;
        break;
    case 2:
        robot_state.control_mode = SPIN_CONTROL;
        break;
    default:
        break;
    }

    if (robot_state.control_mode == RC_CONTROL || robot_state.control_mode == SPIN_CONTROL)
    {
        switch (rc_input->rc.s2)
        {
        case 1:
            robot_state.control_mode = NO_SHOOT_MODE;
            break;
        case 3:
            robot_state.control_mode = SHOOTING_MODE;
            break;
        case 2:
            robot_state.control_mode = SHOOTING_MODE;
        default:
            break;
        }
    }

    if (RC_Ctl.mouse.left_key || RC_Ctl.mouse.right_key)
    {
        robot_state.control_mode = SHOOTING_MODE;
    }
}
