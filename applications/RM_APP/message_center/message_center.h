/**
 * @file message_center.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief ����һ��αpubsub����,����Ӧ��֮���ͨ�Ž����˸���
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include "bsp_system.h"
#include "stdint.h"

#define MAX_TOPIC_NAME_LEN 32 // ���Ļ���������,ÿ�����ⶼ���ַ���������
#define MAX_TOPIC_COUNT 12    // ���֧�ֵĻ�������
#define QUEUE_SIZE 1

typedef struct mqt
{
    /* ������ģ��FIFO���� */
    void *queue[QUEUE_SIZE];
    uint8_t data_len;
    uint8_t front_idx;
    uint8_t back_idx;
    uint8_t temp_size; // ��ǰ���г���

    /* ָ����һ����������ͬ�Ļ���Ķ����ߵ�ָ�� */
    struct mqt *next_subs_queue; // ʹ�÷����߿���ͨ������������ж�������ͬ����Ķ�����
} Subscriber_t;

/**
 * @brief ����������.ÿ��������ӵ�з�����ʵ��,���ҿ���ͨ������������ж������Լ������Ļ���Ķ�����
 *
 */
typedef struct ent
{
    /* �������� */
    char topic_name[MAX_TOPIC_NAME_LEN + 1]; // 1���ֽ����ڴ���ַ��������� '\0'
    uint8_t data_len;                        // �û�������ݳ���
    /* ָ���һ�������˸û���Ķ�����,ͨ������������ж����� */
    Subscriber_t *first_subs;
    /* ָ����һ��Publisher��ָ�� */
    struct ent *next_topic_node;
    uint8_t pub_registered_flag; // ���ڱ�Ǹ÷������Ƿ��Ѿ�ע��
} Publisher_t;

/**
 * @brief ����name�Ļ�����Ϣ
 *
 * @param name ��������
 * @param data_len ��Ϣ����,ͨ��sizeof()��ȡ
 * @return Subscriber_t* ���ض�����ʵ��
 */
Subscriber_t *SubRegister(char *name, uint8_t data_len);

/**
 * @brief ע���Ϊ��Ϣ������
 *
 * @param name �����߷����Ļ�������(����)
 * @return Publisher_t* ���ط�����ʵ��
 */
Publisher_t *PubRegister(char *name, uint8_t data_len);

/**
 * @brief ��ȡ��Ϣ
 *
 * @param sub ������ʵ��ָ��
 * @param data_ptr ����ָ��,���յ���Ϣ����ŵ��˴�
 * @return uint8_t ����ֵΪ0˵��û���µ���Ϣ(��Ϣ����Ϊ��),Ϊ1˵����ȡ�����µ���Ϣ
 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr);

/**
 * @brief �����߸����ж����˻���Ķ�����������Ϣ
 *
 * @param pub ������ʵ��ָ��
 * @param data_ptr ָ��Ҫ���������ݵ�ָ��
 * @return uint8_t ����Ϣ�ɹ����͸�����������
 */
uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr);

#endif // !PUBSUB_H