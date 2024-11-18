#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "applications/RM_BSP/bsp_system.h"

extern int16_t shoot_mode;//拨弹模式

void shoottask(void);

void shoot_init(void);

void shoot_calc_task(void);

void shoot_data_refresh(void);

void shoot_data_send(void);

void shoot_remote_calc(void);

void trigger_motor_calc(void);


void speed1_feed(void);

void speed2_feed(void);

void back_feed(void);

void stop_feed(void);

typedef struct
{
    fp32 target_speed;
    fp32 speed;
    fp32 speed_rpm;

    short send_data;
}SHO_Motor_t;


typedef struct
{
    int16_t circle;//圈数
    int32_t ecd_sum;//积累值
}trigger_change_calc_info_t;

#endif
