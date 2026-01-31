
#ifndef __DBUS_H__
#define __DBUS_H__

#include "main.h"


#include "usart.h"
#include "wwdg.h"

#include "robot_status.h"

typedef struct
{
    struct
    {
        float ch0;//◊Û“°∏Àx÷·
        float ch1;//◊Û“°∏Ày÷·
        float ch2;//”““°∏Àx÷·
        float ch3;//”““°∏Ày÷·
        float dial_wheel;
        uint16_t s1;
        uint16_t s2;
    } rc;

    struct
    {
        float x;
        float y;
        int16_t z;
        uint8_t left_key : 1;
        uint8_t right_key : 1;
    } mouse;

    struct
    {
        uint8_t w : 1;
        uint8_t s : 1;
        uint8_t a : 1;
        uint8_t d : 1;
        uint8_t q : 1;
        uint8_t e : 1;
        uint8_t shift : 1;
        uint8_t ctrl : 1;
    } keyboard;

} _RC_Ctl_t;

extern _RC_Ctl_t RC_Ctl;
extern uint8_t dbus_buffer[18];
extern volatile uint8_t gyro_data_ready;

void Dbus_Init(void);
bool _Parse_RC_Data(void);
void Print_RC_Info(void);
#endif /*__ DBUS_H__ */
