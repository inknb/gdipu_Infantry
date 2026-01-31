#include "dbus.h"
#include "hwt906.h"
#include "referee.h"

_RC_Ctl_t RC_Ctl;
uint8_t dbus_buffer[18];

volatile uint8_t gyro_data_ready;

void Dbus_Init(void)
{
    memset(&RC_Ctl, 0, sizeof(_RC_Ctl_t));

    rc_state = RC_STATE_INIT;

    if (HAL_UART_Receive_DMA(&huart3, dbus_buffer, sizeof(dbus_buffer)) != HAL_OK)
    {
        rc_state = DMA_STARTUP_FAILURE;
        uint8_t retry_count = 0;
        while (retry_count++ < 3)
        {
            if (HAL_UART_Receive_DMA(&huart3, dbus_buffer, sizeof(dbus_buffer)) == HAL_OK)
            {
                rc_state = RC_STATE_INIT;
                break;
            }
            HAL_Delay(10);
        }
    }
}

bool _Parse_RC_Data(void)
{
    const float scale_factor = 1.0f / 660.0f;
    const float mouse_scale_factor = 1.0f / 300.0f;

    uint16_t rx = (dbus_buffer[0] | (dbus_buffer[1] << 8)) & 0x7FF;
    uint16_t ry = ((dbus_buffer[1] >> 3) | (dbus_buffer[2] << 5)) & 0x7FF;
    uint16_t lx = ((dbus_buffer[2] >> 6) | (dbus_buffer[3] << 2) | (dbus_buffer[4] << 10)) & 0x7FF;
    uint16_t ly = ((dbus_buffer[4] >> 1) | (dbus_buffer[5] << 7)) & 0x7FF;
    uint16_t dial_wheel = (dbus_buffer[17] << 8) | dbus_buffer[16];
    int16_t x = (int16_t)((dbus_buffer[6]) | (dbus_buffer[7] << 8));
    int16_t y = (int16_t)((dbus_buffer[8]) | (dbus_buffer[9] << 8));

    if (rx < 364 || rx > 1684 ||
        ry < 364 || ry > 1684 ||
        lx < 364 || lx > 1684 ||
        ly < 364 || ly > 1684)
    {
        rc_state = RC_STATE_ERROR;
        memset(&RC_Ctl, 0, sizeof(_RC_Ctl_t));
        HAL_UART_DMAStop(&huart3);                                       // 停止DMA传输
        __HAL_UART_CLEAR_OREFLAG(&huart3);                               // 清除溢出错误标志
        HAL_UART_Receive_DMA(&huart3, dbus_buffer, sizeof(dbus_buffer)); // 重新启动DMA接收
        return false;
    }

    RC_Ctl.rc.ch0 = (lx - 1024.0f) * scale_factor;
    RC_Ctl.rc.ch1 = (ly - 1024.0f) * scale_factor;
    RC_Ctl.rc.ch2 = (rx - 1024.0f) * scale_factor;
    RC_Ctl.rc.ch3 = (ry - 1024.0f) * scale_factor;
    RC_Ctl.rc.dial_wheel = (dial_wheel - 1024.0f) * scale_factor;

    RC_Ctl.rc.s1 = ((dbus_buffer[5] >> 4) & 0x0C) >> 2;
    RC_Ctl.rc.s2 = (dbus_buffer[5] >> 4) & 0x03;

    RC_Ctl.mouse.x = x * mouse_scale_factor;
    RC_Ctl.mouse.y = y * mouse_scale_factor;
    RC_Ctl.mouse.z = ((dbus_buffer[10]) | (dbus_buffer[11] << 8));
    RC_Ctl.mouse.left_key = dbus_buffer[12];
    RC_Ctl.mouse.right_key = dbus_buffer[13];

    RC_Ctl.keyboard.w = dbus_buffer[14] & 0x01;
    RC_Ctl.keyboard.s = (dbus_buffer[14] & 0x02) >> 1;
    RC_Ctl.keyboard.a = (dbus_buffer[14] & 0x04) >> 2;
    RC_Ctl.keyboard.d = (dbus_buffer[14] & 0x08) >> 3;
    RC_Ctl.keyboard.shift = (dbus_buffer[14] & 0x10) >> 4;
    RC_Ctl.keyboard.ctrl = (dbus_buffer[14] & 0x20) >> 5;
    RC_Ctl.keyboard.q = (dbus_buffer[14] & 0x40) >> 6;
    RC_Ctl.keyboard.e = (dbus_buffer[14] & 0x80) >> 7;

    rc_state = RC_STATE_OK;
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle == &huart3)
    {
        rc_state = RC_STATE_PROCESSING;
    }
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    memset(&RC_Ctl, 0, sizeof(_RC_Ctl_t));
    rc_state = RC_STATE_DISCONNECTED;
}

const char *Get_RC_State_Name(RC_State_t state)
{
    switch (state)
    {
    case RC_STATE_INIT:
        return "RC_STATE_INIT";
    case DMA_STARTUP_FAILURE:
        return "DMA_STARTUP_FAILURE";
    case RC_STATE_ERROR:
        return "RC_STATE_ERROR";
    case RC_STATE_OK:
        return "RC_STATE_OK";
    case RC_STATE_DISCONNECTED:
        return "RC_STATE_DISCONNECTED";
    default:
        return "UNKNOWN_STATE";
    }
}

void Print_RC_Info(void)
{
#if DEBUG_OUTPUT_MODE == 0

    char buffer[320];
    int len = 0;

    len += snprintf(buffer + len, sizeof(buffer) - len,
                    "RC State: %s\n", Get_RC_State_Name(rc_state));

    len += snprintf(buffer + len, sizeof(buffer) - len,
                    "RC[%6.2f,%6.2f,%6.2f,%6.2f] "
                    "SW[%d,%d] "
                    "M[X:%6.3f Y:%6.3f Z:%4d L:%1d R:%1d] "
                    "KB[W:%1d S:%1d A:%1d D:%1d SHIFT:%1d CTRL:%1d Q:%1d E:%1d]\n",
                    RC_Ctl.rc.ch0, RC_Ctl.rc.ch1, RC_Ctl.rc.ch2, RC_Ctl.rc.ch3,
                    RC_Ctl.rc.s1, RC_Ctl.rc.s2,
                    RC_Ctl.mouse.x, RC_Ctl.mouse.y, RC_Ctl.mouse.z,
                    RC_Ctl.mouse.left_key, RC_Ctl.mouse.right_key,
                    RC_Ctl.keyboard.w, RC_Ctl.keyboard.s, RC_Ctl.keyboard.a,
                    RC_Ctl.keyboard.d, RC_Ctl.keyboard.shift, RC_Ctl.keyboard.ctrl,
                    RC_Ctl.keyboard.q, RC_Ctl.keyboard.e);

    SEGGER_RTT_Write(0, buffer, len);

#elif DEBUG_OUTPUT_MODE == 1

    //    char buf[64];
    //    int len = sprintf(buf, "Ch0: %.2f, Ch1: %.2f, Ch2: %.2f, Ch3: %.2f\r\n",
    //                      RC_Ctl.rc.ch0, RC_Ctl.rc.ch1, RC_Ctl.rc.ch2, RC_Ctl.rc.ch3);
    //    Safe_Printf(buf);

#endif
}