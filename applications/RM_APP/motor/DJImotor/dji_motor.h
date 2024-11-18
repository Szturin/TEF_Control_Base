/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI���ܵ��ͷ�ļ�
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. ����ͬ�ĵ�����ò�ͬ�ĵ�ͨ�˲�������ϵ��������ͳһʹ�ú�
          2. ΪM2006��M3508���ӿ�������λУ׼����,���ڳ�ʼ��ʱ����(�����û����þ����Ƿ����)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_system.h"
#include "daemon/daemon.h"
#include "bsp_can_demo.h"
#include "stdint.h"
#include "general_def.h"
#include "motor_def.h"
#include "bsp_dwt.h"
#include "controller.h"

#define DJI_MOTOR_CNT 12

/* �˲�ϵ������Ϊ1��ʱ�򼴹ر��˲� */
#define SPEED_SMOOTH_COEF 0.85f      // ��ô���0.85
#define CURRENT_SMOOTH_COEF 0.9f     // �������0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),��������ֵת��Ϊ�Ƕ���

/* DJI���CAN������Ϣ*/
typedef struct
{
    uint16_t last_ecd;        // ��һ�ζ�ȡ�ı�����ֵ
    uint16_t ecd;             // 0-8191,�̶��ܹ���8192��
    float angle_single_round; // ��Ȧ�Ƕ�
    float speed_aps;          // ���ٶ�,��λΪ:��/��
    int16_t real_current;     // ʵ�ʵ���
    uint8_t temperature;      // �¶� Celsius

    float total_angle;   // �ܽǶ�,ע�ⷽ��
    int32_t total_round; // ��Ȧ��,ע�ⷽ��
} DJI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    DJI_Motor_Measure_s measure;            // �������ֵ
    Motor_Control_Setting_s motor_settings; // �������
    Motor_Controller_s motor_controller;    // ���������

    CANInstance *motor_can_instance; // ���CANʵ��
    // ���鷢������
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;        // �������
    Motor_Working_Type_e stop_flag; // ��ͣ��־

    DaemonInstance* daemon;
    uint32_t feed_cnt;
    float dt;
} DJIMotorInstance;

/**
 * @brief ���ô˺���ע��һ��DJI���ܵ��,��Ҫ���ݽ϶�ĳ�ʼ������,����application��ʼ����ʱ����ô˺���
 *        �Ƽ�����ʱ���׼��һ������initStructureȻ����˺���.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        ��ע�ⲻҪ��һ�������Ϲ��ع���ĵ��(����6��),��һ��Ҫ��ô��,�뽵��ÿ������ķ���Ƶ��(��Ϊ500Hz),
 *        ����СDJIMotorControl()���������Ƶ��.
 *
 * @attention M3508��M2006�ķ������Ķ���0x200+id,��GM6020�ķ�����0x204+id,��ע��ǰ���ߺͺ��ߵ�id��Ҫ��ͻ.
 *            ���������ͻ,�ڳ�ʼ�������ʱ������IDcrash_Handler(),����ͨ��debug���ж��Ƿ���ֳ�ͻ.
 *
 * @param config �����ʼ���ṹ��,�����˵����������,���PID��������,��������Լ�������ص�CAN����
 *
 * @return DJIMotorInstance*
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);

/**
 * @brief ��application���Ӧ�õ���,������趨�ο�ֵ.
 *        ����Ӧ��,���Խ������Ϊ���ݺ���Ϊ1���豸,����Ҫ���ĵײ�ıջ�
 *
 * @param motor Ҫ���õĵ��
 * @param ref �趨�ο�ֵ
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/**
 * @brief �л�������Ŀ����Դ,�罫���ٶȺͽǶȵ���Դ��ΪIMU(С����ģʽ����)
 *
 * @param motor Ҫ�л�����������Դ�ĵ��
 * @param loop  Ҫ�л�����������Դ�Ŀ��Ʊջ�
 * @param type  Ŀ�귴��ģʽ
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief �ú�����motor_task����������rtos��,motor_stask��ͨ��osDelay()ȷ������Ƶ��
 */
void DJIMotorControl();

/**
 * @brief ֹͣ���,ע�ⲻ�ǽ��趨ֵ��Ϊ��,����ֱ�Ӹ�������͵ĵ���ֵ����
 *
 */
void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief �������,��ʱ�������Ӧ�趨ֵ
 *        ��ʼ��ʱ����Ҫ�˺���,��Ϊstop_flag��Ĭ��ֵΪ0
 *
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/**
 * @brief �޸ĵ���ջ�Ŀ��(���ջ�)
 *
 * @param motor  Ҫ�޸ĵĵ��ʵ��ָ��
 * @param outer_loop ���ջ�����
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

#endif // !DJI_MOTOR_H