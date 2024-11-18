#ifndef __JY901USE_H
#define __JY901USE_H

#include "applications/RM_BSP/bsp_system.h"

#define RXBUFFER_LEN 33


typedef struct
{

    float angle[3];

}Angle;


typedef struct
{

    float a[3];

}Acc;


typedef struct
{

    float w[3];

}SGyro;


typedef struct User_USART
{

    uint8_t Rx_flag;

    uint8_t Rx_len;

    uint8_t frame_head;

    uint8_t RxBuffer[RXBUFFER_LEN];

    Angle angle;

    Acc acc;

    SGyro w;

}User_USART;

void JY901_Process(void);

void User_USART_Init(User_USART *Data);

extern User_USART JY901_data;

extern float error_angle_calc(float target_angle, float now_angle);

extern float ffabs(float number);

#endif
