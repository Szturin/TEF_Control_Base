#ifndef __RM_DEFINE_H__  //��ͷ�ļ�������һЩRM�ĺ궨��   
#define __RM_DEFINE_H__

#include "applications/RM_BSP/bsp_system.h"
/***************************************��������*********************************************/
#define PI 3.141592653f

/***************************************ң����ض���*********************************************/
//ң����ͨ��1��ͨ��4�ı��� ���̵������̨��������ֵ��ͬ�����Ը�����Ҫ�ı��ʼ���
#define R_ch1 3
#define R_ch2 1
#define R_ch3 1
#define R_ch4 1
//�˴��������õ�����MAX_Motor_SPeed����ֵ�����ʣ�ֵԽ��Խ�쵽��MAX_Motor_SPeed���˴����ֵΪ24.8242422
//�м�660*R_chX(X=1��2��3��4����֮һ)�����MAX_Motor_SPeed ��������ٶȽ���660*R_chX����

/***************************************������ض���*********************************************/
//Chassis ���̵�һЩ�궨��
#define ROLL_FORWARD 1
#define ROLL_BACK    -1

//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.002f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���

#define CHASSIS_WZ_RC_SEN 0.01f
//rpm��ÿ���ӵ���ת����
//m3508ת���ɵ����ٶ�(m/s)�ı���
//����ϵ��=�ܳ�/���ٱ�/60 ע��/60��Ϊ�˻����s��λ
//�ٸ�����
//��֪������ٶ�Ϊ400rpm�����ӵ�ֱ��Ϊ185mm������ļ��ٱ�Ϊ1��19����ǰ���ӵ��ٶȣ�

//�⣺ ��������Ĺ�ʽ��
//����ϵ��=(0.1852*PI/2)/19/60=0.0005098;
//���ԣ����ӵ��ٶ�=400* (0.1852��/2)/19/60=0.203927(m/s)��
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f//0.0002098115409482972879038f

#define CHASSISMOTOR_TO_CENTER 0.2723967694   //��Ҫ���������ķ���뼸�����ĵ�ˮƽ����ʹ�ֱ���룬���ù��ɶ�����ø���ֵ ��λ��m��
#define CHASSIS_VX_SPEED  0.25f  //Ϊʲô��0.25,VX���ٶ�Ϊ�ĸ��ֵ��ٶȵ��ӣ���4��Ϊ�����ٶ�
#define CHASSIS_VY_SPEED  0.25f
#define CHASSIS_VW_SPEED  0.25f

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.0f

#define MAX_Motor_SPeed 3.0f //�ٶ�����  0.5m/s
//�����λ�õ��Ƹ˵��������λ��֮��ľ���ΪD�������ٶȴﵽMAX_Motor_SPeed��ֵʱ����ʱ�Ƹ�λ��Ϊ(MAX_Motor_SPeed/660*R_chX)*D��
//�ٸ�����MAX_Motor_SPeed=2000��660*R_chX=6000�������ٶȴﵽMAX_Motor_SPeed��ֵʱ���Ƹ�λ��λ��2000/6000Ҳ���ǣ�1/3��D��
//2022-12-22 ������

#define CHASSIS_CAN hcan1//���̲���can1��ͨѶ

/***************************************��̨��ض���*********************************************/
#define Motor_Ecd_to_Rad   0.000766990394f //      2*  PI  /8192  �Ƕ�ת��

#define Half_ecd_range 4096   //�������ֵ��ֵ
#define ecd_range 8191                //�������ֵ���ֵ

//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define GIMBAL_YAW_RC_SEN 0.00300f
#define GIMBAL_PIT_RC_SEN 0.00115f

#define GIMBAL_CAN hcan2//��̨Ԥ����can2��ͨѶ

/*************************************���������ض���*******************************************/
//2006���ת���ɲ����������ٶ�
#define M2006_MOTOR_RPM_TO_VECTOR 0.000036361025462962f

//����Ħ��������
#define shoot_stop_speed 1000
#define shoot_basic_speed 1480
#define shoot_leveltwo_speed 1530
#define shoot_levelthree_speed 1680

//���ٲ����ٶ�
#define feed_target_speed -0.45
#define feed_target_second_speed -0.75

//������ģʽ�趨
#define STOP 0
#define BACK 1
#define SPEED_1 2
#define SPEED_2 3

//�ģʽ�趨
#define CLOSE 0
#define OPEN 1

#define SHOOT_CAN hcan2

/*************************************����ͨ��*******************************************/
//�����Ƿ񼤻�
#define NOT_READY 0
#define READY 1

//����ͨ���µ�С��ģʽ
#define BASIC 0//��ͨ�˶�
#define GLOBAL 1//��תС����
#define FOLLOW 2//�����涯zx

//�ƶ�ģʽ
#define SLOW_SPEED 0.6f
#define NORMAL_SPEED 0.95f
#define FAST_SPEED 1.2f

//������ģʽ
#define burst  0//����
#define strafe 1//ɨ��

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 }

#endif

