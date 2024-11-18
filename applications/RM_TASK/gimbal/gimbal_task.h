#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "applications/RM_BSP/bsp_system.h"

void gimbaltask(void);                     //��̨������
void gimbal_data_send(void);             //��̨������������Ƶ��ת��
void gimbal_init(void);

void gimbal_data_refresh(void);

void gimbal_remote_calc(void);

void gimbal_calc_task(void);

void gimbal_initial_angle(void);        //��̨��ʼ���ǶȻ�ȡ
void gimbal_calc(void);                    //��̨PID����

void imu_calc(void);                //�������涯����
void imu_space_angle(void);            //�����ǿռ�Ƕ�
void imu_deflective_angle(void);    //������ƫת�Ƕ�
void imu_accumulate_angle(void);    //�������ۼӽǶ�

float angle_calc(float angle1, float angle2);

static float target_angle_calc(float angle, float add);

static float relative_angle_calc(unsigned int angle, unsigned int initial_angle);

static float filtering(float last_angle, float angle);

double map(double x, double min, double max, double min_t, double max_t);

void servo_move(void);          //���ת��

typedef struct
{
    int16_t initial_ecd;
    int16_t now_ecd;
    int16_t target_ecd;
    int16_t last_target_ecd;
    int16_t error_ecd;

    float initial_angle;
    float now_angle;
    float target_angle;
    float last_target_angle;
    float error_angle;
    float last_error_angle;

    float now_speed;
    float target_speed;

    short send_data;

    float add_angle;
}Gim_Motor_t;

typedef struct
{
    short pit_info;
    short yaw_info;
    unsigned char gim_mode_info;
    short save_info;
}gim_info_t;

typedef struct
{
    short shoot_speed;
    short friction_CCR;
    short LASER_FLAG;
}shoot_info_t;

typedef struct
{
    float space_initial_angle;
    float space_angle;
    float deflective_angle;
}calc_info_t;



extern shoot_info_t shoot_info;
extern uint8_t keyboard_flag;

#endif
