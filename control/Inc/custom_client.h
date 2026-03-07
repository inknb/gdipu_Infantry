#ifndef CUSTOM_CLIENT_H
#define CUSTOM_CLIENT_H

#include <stdint.h>

// 根据官方协议定义的键鼠数据结构体 [cite: 618-624]
typedef struct {
    int32_t mouse_x;            // 鼠标 x 轴移动速度
    int32_t mouse_y;            // 鼠标 y 轴移动速度
    int32_t mouse_z;            // 鼠标滚轮移动速度
    uint8_t left_button_down;   // 左键是否按下 (1=按下, 0=抬起)
    uint8_t right_button_down;  // 右键是否按下 (1=按下, 0=抬起)
    uint32_t keyboard_value;    // 键盘按键位掩码
    uint8_t mid_button_down;    // 中键是否按下 (1=按下, 0=抬起)
} Custom_Client_KB_t;

// 供全局调用的外部变量
extern Custom_Client_KB_t custom_client_data;

// 解析函数声明
void parse_keyboard_mouse_control(const uint8_t *payload, uint16_t payload_len);

#endif