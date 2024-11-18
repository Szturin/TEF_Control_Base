#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "applications/RM_BSP/bsp_system.h"

//电机参数
typedef struct
{
    fp32 vx_set; //底盘在x方向上的期望速度
    fp32 vy_set; //底盘在y方向上的期望速度
    fp32 wz_set; //底盘在z方向上的期望速度
    fp32 last_set; //上一次的角速度设定值
    fp32 wheel_speed[4]; //底盘四个电机的期望速度
    fp32 angle; //底盘当前角度
}chassis_ctrl_info_t;

//底盘参数
typedef struct
{
    fp32 vx; //底盘在x方向上的当前速度
    fp32 vy; //底盘在y方向上的当前速度
    fp32 wz; //底盘在z方向上的当前速度
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 chassis_relative_angle; //底盘相对于某个方向的角度
    fp32 chassis_relative_angle_set; //底盘相对于某个方向的期望角度
}chassis_motor_parameter_t;

void chassistask(void);

void controlmode1_solution(void);

void chassis_data_send(void);

void gimbal_axis_control(void);

void chassis_axis_control(void);

void gimbal_follow_control(void);

void PID_Adjust(void);

void chassis_data_calc(void);

void mackenaham_calc(fp32 vx_set, fp32 vy_set, fp32 wz_set);

void keyboard_control(void);


#endif

