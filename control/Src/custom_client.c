#include "custom_client.h"

// 全局结构体实例，存放最新解析的数据
Custom_Client_KB_t custom_client_data;

/**
 * @brief  轻量级 Protobuf 变长整数 (Varint) 解码器
 */
static uint8_t pb_decode_varint(const uint8_t *buf, uint16_t len, uint64_t *value) {
    uint8_t count = 0;
    uint64_t result = 0;
    uint8_t shift = 0;
    
    while (count < len) {
        uint8_t byte = buf[count++];
        result |= (uint64_t)(byte & 0x7F) << shift; 
        shift += 7;
        
        if (!(byte & 0x80)) { 
            *value = result;
            return count;
        }
    }
    return 0; 
}

/**
 * @brief  解析 KeyboardMouseControl 数据流 
 * @param  payload: 去掉 0xA5 帧头和校验后的核心 Protobuf 数据区
 * @param  payload_len: 核心数据长度
 */
void parse_keyboard_mouse_control(const uint8_t *payload, uint16_t payload_len) {
    uint16_t offset = 0;
    uint64_t varint_val = 0;

    while (offset < payload_len) {
        // 1. 读取 Tag
        uint8_t tag_bytes = pb_decode_varint(&payload[offset], payload_len - offset, &varint_val);
        if (tag_bytes == 0) break;
        offset += tag_bytes;

        uint32_t field_num = varint_val >> 3;   
        uint32_t wire_type = varint_val & 0x07; 

        // 2. 提取数据
        if (wire_type == 0) { 
            uint8_t val_bytes = pb_decode_varint(&payload[offset], payload_len - offset, &varint_val);
            if (val_bytes == 0) break;
            offset += val_bytes;

            // 映射到官方协议定义的编号 [cite: 618-624]
            switch (field_num) {
                case 1: custom_client_data.mouse_x = (int32_t)varint_val; break;
                case 2: custom_client_data.mouse_y = (int32_t)varint_val; break;
                case 3: custom_client_data.mouse_z = (int32_t)varint_val; break;
                case 4: custom_client_data.left_button_down = (uint8_t)varint_val; break;
                case 5: custom_client_data.right_button_down = (uint8_t)varint_val; break;
                case 6: custom_client_data.keyboard_value = (uint32_t)varint_val; break;
                case 7: custom_client_data.mid_button_down = (uint8_t)varint_val; break;
                default: break; 
            }
        }
        else if (wire_type == 1 || wire_type == 5) { // 跳过固定长度数据
            offset += (wire_type == 1) ? 8 : 4;
        }
        else if (wire_type == 2) { // 跳过变长数据
            uint8_t len_bytes = pb_decode_varint(&payload[offset], payload_len - offset, &varint_val);
            offset += len_bytes + (uint16_t)varint_val; 
        }
    }
}