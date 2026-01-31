#ifndef __HWT906_H__
#define __HWT906_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

// 陀螺仪数据结构体
typedef struct
{
	float roll;       // 横滚角度(°)
	float pitch;      // 俯仰角度(°)
	float yaw;        // 偏航角度(°)
	float roll_rate;  // 横滚角速度(°/s)
	float pitch_rate; // 俯仰角速度(°/s)
	float yaw_rate;   // 偏航角速度(°/s)
} Gyro;

// 全局陀螺仪数据
extern Gyro gyro;

// 函数声明
void hwt906_init(void);
void ParseGyroData(uint8_t* data, uint16_t size);
   
#ifdef __cplusplus
}
#endif

#endif /* __HWT906_H__ */