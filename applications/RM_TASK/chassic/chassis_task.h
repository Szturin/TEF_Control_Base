#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "applications/RM_BSP/bsp_system.h"

//�������
typedef struct
{
    fp32 vx_set; //������x�����ϵ������ٶ�
    fp32 vy_set; //������y�����ϵ������ٶ�
    fp32 wz_set; //������z�����ϵ������ٶ�
    fp32 last_set; //��һ�εĽ��ٶ��趨ֵ
    fp32 wheel_speed[4]; //�����ĸ�����������ٶ�
    fp32 angle; //���̵�ǰ�Ƕ�
}chassis_ctrl_info_t;

//���̲���
typedef struct
{
    fp32 vx; //������x�����ϵĵ�ǰ�ٶ�
    fp32 vy; //������y�����ϵĵ�ǰ�ٶ�
    fp32 wz; //������z�����ϵĵ�ǰ�ٶ�
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 chassis_relative_angle; //���������ĳ������ĽǶ�
    fp32 chassis_relative_angle_set; //���������ĳ������������Ƕ�
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

