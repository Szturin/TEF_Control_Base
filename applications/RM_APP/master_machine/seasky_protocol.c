/**
 * @file seasky_protocol.c
 * @author Liu Wei
 * @author modified by Neozng
 * @brief ���ϴ�ѧRoBoMatster����ͨ��Э��
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "seasky_protocol.h"
#include "algorithm/crc8.h"
#include "algorithm/crc16.h"
#include "memory.h"

/*��ȡCRC8У����*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*����CRC8���ݶ�*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*��ȡCRC16У����*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*����CRC16���ݶ�*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*��������֡ͷ*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        if (CRC8_Check_Sum(&rx_buf[0], 4))
        {
            pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
            pro->header.crc_check = rx_buf[3];
            pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
            return 1;
        }
    }
    return 0;
}

/*
    �˺������ݴ����͵����ݸ�������֡��ʽ�Լ����ݣ�ʵ�����ݵĴ������
    ��������ͨ�Žӿڵķ��ͺ�������tx_buf�еĶ�Ӧ����
*/
void get_protocol_send_data(uint16_t send_id,        // �ź�id
                            uint16_t flags_register, // 16λ�Ĵ���
                            float *tx_data,          // �����͵�float����
                            uint8_t float_length,    // float�����ݳ���
                            uint8_t *tx_buf,         // �����͵�����֡
                            uint16_t *tx_buf_len)    // �����͵�����֡����
{
    static uint16_t crc16;
    static uint16_t data_len;

    data_len = float_length * 4 + 2;
    /*֡ͷ����*/
    tx_buf[0] = PROTOCOL_CMD_ID;
    tx_buf[1] = data_len & 0xff;        // ��λ��ǰ
    tx_buf[2] = (data_len >> 8) & 0xff; // ��λ��ǰ
    tx_buf[3] = crc_8(&tx_buf[0], 3);   // ��ȡCRC8У��λ

    /*���ݵ��ź�id*/
    tx_buf[4] = send_id & 0xff;
    tx_buf[5] = (send_id >> 8) & 0xff;

    /*����16λ�Ĵ���*/
    tx_buf[6] = flags_register & 0xff;
    tx_buf[7] = (flags_register >> 8) & 0xff;

    /*float���ݶ�*/
    for (int i = 0; i < 4 * float_length; i++)
    {
        tx_buf[i + 8] = ((uint8_t *)(&tx_data[i / 4]))[i % 4];
    }

    /*����У��*/
    crc16 = crc_16(&tx_buf[0], data_len + 6);
    tx_buf[data_len + 6] = crc16 & 0xff;
    tx_buf[data_len + 7] = (crc16 >> 8) & 0xff;

    *tx_buf_len = data_len + 8;
}
/*
    �˺������ڴ���������ݣ�
    �����������ݵ�id
*/
uint16_t get_protocol_info(uint8_t *rx_buf,          // ���յ���ԭʼ����
                           uint16_t *flags_register, // �������ݵ�16λ�Ĵ�����ַ
                           uint8_t *rx_data)         // ���յ�float���ݴ洢��ַ
{
    // ���ھ�̬��,���ⷴ������ջ�Ͽռ�
    static protocol_rm_struct pro;
    static uint16_t date_length;

    if (protocol_heade_Check(&pro, rx_buf))
    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        if (CRC16_Check_Sum(&rx_buf[0], date_length))
        {
            *flags_register = (rx_buf[7] << 8) | rx_buf[6];
            memcpy(rx_data, rx_buf + 8, pro.header.data_length - 2);
            return pro.cmd_id;
        }
    }
    return 0;
}