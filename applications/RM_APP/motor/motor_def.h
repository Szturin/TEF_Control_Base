/**
 * @file motor_def.h
 * @author neozng
 * @brief  ���ͨ�õ����ݽṹ����
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "controller.h"
#include "stdint.h"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief �ջ�����,�����Ҫ����ջ�,��ʹ�û�����
 *        ������Ҫ�ٶȻ��͵�����: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,

    // only for checking
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;

typedef enum
{
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,
    SPEED_FEEDFORWARD = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* ������Դ�趨,����ΪOTHER_FEED����Ҫָ��������Դָ��,���Motor_Controller_s*/
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e;

/* �������ת��־ */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/* ������������־ */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

/* �����������,�����ջ�����,��ת��־�ͷ�����Դ */
typedef struct
{
    Closeloop_Type_e outer_loop_type;              // �����ıջ�,δ����ʱĬ��Ϊ��߼��ıջ�
    Closeloop_Type_e close_loop_type;              // ʹ�ü����ջ�(����)
    Motor_Reverse_Flag_e motor_reverse_flag;       // �Ƿ�ת
    Feedback_Reverse_Flag_e feedback_reverse_flag; // �����Ƿ���
    Feedback_Source_e angle_feedback_source;       // �Ƕȷ�������
    Feedback_Source_e speed_feedback_source;       // �ٶȷ�������
    Feedfoward_Type_e feedforward_flag;            // ǰ����־

} Motor_Control_Setting_s;

/* ���������,����������Դ�ķ�������ָ��,3���������͵���Ĳο�����*/
// ��������ǰ������ָ��
typedef struct
{
    float *other_angle_feedback_ptr; // ����������Դ�ķ�������ָ��
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float pid_ref; // ������Ϊÿ��������������˳��ͨ�������ջ�
} Motor_Controller_s;

/* �������ö�� */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
} Motor_Type_e;

/**
 * @brief �����������ʼ���ṹ��,��������PID�������Լ���������������Դָ��
 *        �������Ҫĳ�����ƻ�,���Բ����ö�Ӧ��pid config
 *        ��Ҫ����������Դ���з����ջ�,����Ҫ���������ָ�뻹��Ҫ��Motor_Control_Setting_s��������������Դ��־
 */
typedef struct
{
    float *other_angle_feedback_ptr; // �Ƕȷ�������ָ��,ע����ʹ��total_angle
    float *other_speed_feedback_ptr; // �ٶȷ�������ָ��,��λΪangle per sec

    float *speed_feedforward_ptr;   // �ٶ�ǰ������ָ��
    float *current_feedforward_ptr; // ����ǰ������ָ��

    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
} Motor_Controller_Init_s;

/* ���ڳ�ʼ��CAN����Ľṹ��,������ͨ�� */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H