//
// Created by 123 on 2024/11/18.
//

#ifndef RTTHREAD_BSP_CAN_DEMO_H
#define RTTHREAD_BSP_CAN_DEMO_H
#include "bsp_system.h"

#include <stdint.h>
#include "can.h"

// ����ܹ�֧�ֵ�CAN�豸��
#define CAN_MX_REGISTER_CNT 16     // �������ȡ����CAN���ߵĸ���
#define MX_CAN_FILTER_CNT (2 * 14) // ������ʹ�õ�CAN����������,ĿǰԶ�����õ���ô��
#define DEVICE_CAN_CNT 2           // ���ݰ����趨,F407IG��CAN1,CAN2,���Ϊ2;F334ֻ��һ��,����Ϊ1
// ���ֻ��1��CAN,����Ҫ��bsp_can.c�����е�hcan2������Ϊhcan1(����,��Ҫ�����ߺ�FIFO�ĸ��ؾ���,��Ӱ�칦��)

/* can instance typedef, every module registered to CAN should have this variable */
#pragma pack(1)
typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can���
    CAN_TxHeaderTypeDef txconf;    // CAN���ķ�������
    uint32_t tx_id;                // ����id
    uint32_t tx_mailbox;           // CAN��Ϣ����������
    uint8_t tx_buff[8];            // ���ͻ���,������Ϣ���ȿ���ͨ��CANSetDLC()�趨,���Ϊ8
    uint8_t rx_buff[8];            // ���ջ���,�����Ϣ����Ϊ8
    uint32_t rx_id;                // ����id
    uint8_t rx_len;                // ���ճ���,����Ϊ0-8
    // ���յĻص�����,���ڽ������յ�������
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // ʹ��can�����ģ��ָ��(��idָ���ģ��ӵ�д�canʵ��,�Ǹ��ӹ�ϵ)
} CANInstance;
#pragma pack()

/* CANʵ����ʼ���ṹ��,���˽ṹ��ָ�봫��ע�ắ�� */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can���
    uint32_t tx_id;                             // ����id
    uint32_t rx_id;                             // ����id
    void (*can_module_callback)(CANInstance *); // ����������ݵĻص�����
    void *id;                                   // ӵ��canʵ����ģ���ַ,�������ֲ�ͬ��ģ��(�������Ҫ�Ļ�),�������Ҫ���Բ�����
} CAN_Init_Config_s;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        ע��(��ʼ��)һ��canʵ��,��Ҫ�����ʼ�����õ�ָ��.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief �޸�CAN���ͱ��ĵ�����֡����;ע����󳤶�Ϊ8,��û�н����޸ĵ�ʱ��,Ĭ�ϳ���Ϊ8
 *
 * @param _instance Ҫ�޸ĳ��ȵ�canʵ��
 * @param length    �趨����
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief transmit mesg through CAN device,ͨ��canʵ��������Ϣ
 *        ����ǰ��Ҫ��CANʵ����tx_buffд�뷢������
 *
 * @attention ��ʱʱ�䲻Ӧ�ó������ô˺��������������,����ᵼ����������
 *
 * @param timeout ��ʱʱ��,��λΪms;������Ϊus,��ø���ȷ�Ŀ���
 * @param _instance* can instance owned by module
 */
uint8_t CANTransmit(CANInstance *_instance,float timeout);

#endif //RTTHREAD_BSP_CAN_DEMO_H
