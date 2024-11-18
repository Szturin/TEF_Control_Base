/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // ������#pragma once����#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

//#include "ins_task.h"
#include "master_machine/master_process.h"
#include "stdint.h"

/* ���������Ͷ���,��¼ʱע�ⲻҪŪ���Ӧ����;�޸Ķ������Ҫ���±���,ֻ�ܴ���һ������! */
#define ONE_BOARD // �����������
// #define CHASSIS_BOARD //���̰�
// #define GIMBAL_BOARD  //��̨��

#define VISION_USE_VCP  // ʹ�����⴮�ڷ����Ӿ�����
// #define VISION_USE_UART // ʹ�ô��ڷ����Ӿ�����

/* ��������Ҫ��������,ע����ݲ�ͬ�����˽����޸�,��������Ҫ��.0��f��β,�޷�����u��β */
// ��̨����
#define YAW_CHASSIS_ALIGN_ECD 2711  // ��̨�͵��̶���ָ����ͬ����ʱ�ĵ��������ֵ,������̨�л�е�Ķ���Ҫ�޸�
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECDֵ�Ƿ����4096,��Ϊ1,��Ϊ0;���ڼ�����̨ƫת�Ƕ�
#define PITCH_HORIZON_ECD 3412      // ��̨����ˮƽλ��ʱ������ֵ,������̨�л�е�Ķ���Ҫ�޸�
#define PITCH_MAX_ANGLE 0           // ��̨��ֱ�������Ƕ� (ע�ⷴ������������ǣ�����д�����ǵĽǶ�)
#define PITCH_MIN_ANGLE 0           // ��̨��ֱ������С�Ƕ� (ע�ⷴ������������ǣ�����д�����ǵĽǶ�)
// �������
#define ONE_BULLET_DELTA_ANGLE 36    // ����һ�����貦��ת���ľ���,�ɻ�е���ͼֽ����
#define REDUCTION_RATIO_LOADER 36.0f // 2006���̵���ļ��ٱ�,Ӣ����Ҫ�޸�Ϊ3508��19.0f
#define NUM_PER_CIRCLE 10            // ����һȦ��װ����
// �����˵����޸ĵĲ���,��λΪmm(����)
#define WHEEL_BASE 350              // �������(ǰ�����˷���)
#define TRACK_WIDTH 300             // �����־�(����ƽ�Ʒ���)
#define CENTER_GIMBAL_OFFSET_X 0    // ��̨��ת���ľ���̼������ĵľ���,ǰ����,��̨λ��������ʱĬ����Ϊ0
#define CENTER_GIMBAL_OFFSET_Y 0    // ��̨��ת���ľ���̼������ĵľ���,���ҷ���,��̨λ��������ʱĬ����Ϊ0
#define RADIUS_WHEEL 60             // ���Ӱ뾶
#define REDUCTION_RATIO_WHEEL 19.0f // ������ٱ�,��Ϊ�������������ת�ӵ��ٶȶ������������ٶȹ������ת��

#define GYRO2GIMBAL_DIR_YAW 1   // �����������������̨��yaw�ķ���,1Ϊ��ͬ,-1Ϊ�෴
#define GYRO2GIMBAL_DIR_PITCH 1 // �����������������̨��pitch�ķ���,1Ϊ��ͬ,-1Ϊ�෴
#define GYRO2GIMBAL_DIR_ROLL 1  // �����������������̨��roll�ķ���,1Ϊ��ͬ,-1Ϊ�෴

// ����Ƿ�������ذ嶨���ͻ,ֻ����һ�������嶨�����,���������Զ�����
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // ѹ���ṹ��,ȡ���ֽڶ���,��������ݶ����ܱ�����
/* -------------------------��������ģʽ���������Ͷ���-------------------------*/
/**
 * @brief ��Щö�����ͺͽṹ�����ΪCMD�������ݺ͸�Ӧ�õķ������ݵ�һ����
 *
 */
// ������״̬
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// Ӧ��״̬
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// ����ģʽ����
/**
 * @brief ���������޸�Ϊ��̨�������,�������õ���ȥ׷��̨,��̨�Ĺ����ȵ���С.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // ����������
    CHASSIS_ROTATE,            // С����ģʽ
    CHASSIS_NO_FOLLOW,         // �����棬����ȫ��ƽ��
    CHASSIS_FOLLOW_GIMBAL_YAW, // ����ģʽ�����̵��ӽǶȻ�����
} chassis_mode_e;

// ��̨ģʽ����
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // ����������
    GIMBAL_FREE_MODE,      // ��̨�����˶�ģʽ,������̷���(���̴�ʱӦΪNO_FOLLOW)����ֵΪ���total_angle;�ƺ����Ը�Ϊȫ����IMU����?
    GIMBAL_GYRO_MODE,      // ��̨�����Ƿ���ģʽ,����ֵΪ������pitch,total_yaw_angle,���̿���ΪС���ݺ͸���ģʽ
} gimbal_mode_e;

// ����ģʽ����
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // Ħ���ֹر�
    FRICTION_ON,      // Ħ���ֿ���
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // ���ոǴ�
    LID_CLOSE,    // ���ոǹر�
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // ֹͣ����
    LOAD_REVERSE,   // ��ת
    LOAD_1_BULLET,  // ����
    LOAD_3_BULLET,  // ����
    LOAD_BURSTFIRE, // ����
} loader_mode_e;

// ��������,�Ӳ���ϵͳ��ȡ,�Ƿ��б�Ҫ����?
typedef struct
{ // ���ʿ���
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMDӦ�÷����Ŀ�������,Ӧ����gimbal/chassis/shoot����---------------- */
/**
 * @brief ����˫�����,ң������pc����̨,����ϵͳ�ڵ���
 *
 */
// cmd�����ĵ��̿�������,��chassis����
typedef struct
{
    // ���Ʋ���
    float vx;           // ǰ�������ٶ�
    float vy;           // ���Ʒ����ٶ�
    float wz;           // ��ת�ٶ�
    float offset_angle; // ���̺͹���λ�õļн�
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI����
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd��������̨��������,��gimbal����
typedef struct
{ // ��̨�Ƕȿ���
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd�����ķ����������,��shoot����
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // ����ö��
    uint8_t rest_heat;
    float shoot_rate; // �����������Ƶ,unit per s,��/��
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis�����ķ�������----------------*/
/**
 * @brief ��cmd����,����Ӧ��Ҳ���Ը�����Ҫ��ȡ.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // �ǵ����ʱ����̻���imu���ݻش�(���б�Ҫ)
    // attitude_t chassis_imu_data;
#endif
    // �������ӵ��̵���ʵ�ٶ�
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat;           // ʣ��ǹ������
    Bullet_Speed_e bullet_speed; // ��������
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;


typedef struct
{
    //attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;

#pragma pack() // �����ֽڶ���,����ǰ���#pragma pack(1)

#endif // !ROBOT_DEF_H