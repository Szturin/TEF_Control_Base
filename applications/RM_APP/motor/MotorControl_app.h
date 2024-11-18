#ifndef MotorControl_APP_H
#define MotorControl_APP_H

#include "applications/RM_BSP/bsp_system.h"

/*
***************************************************
备注：电机pid的参数初始化
***************************************************
*/

/*******************↓底盘功率PID参数↓********************//*1000 210 1200*/
#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
    .Kp = 1.2,  /*8.5*/ \
    .Ki = 0.2, /*0.2*/ \
    .Kd = 0.0,    /*18*/  \
    .Output_Max = 16384,\
    .DeadZone = 50,\
    .EIS_Max = 2000,\
    .EAIS_Max = 2000,\
    .Integral_Max = 1500,\
    .Error = {0,0,0},\
    .Integral = 0,\
    .Output = 0,\
    .Output_Last = 0,\
    .Calc = &Position_PID,\
    .RST  = &PID_Reset,\
}

void CMSpeedAdjust(float speed1_target, float speed2_target, float speed3_target, float speed4_target, uint8_t flag_reset);

void CMControl(CANSend_TypeDef *RM3510_CANSend);

#endif




