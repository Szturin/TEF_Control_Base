/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo ���ӶԴ��ڵ�������Э���֧��,����vofa��serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon/daemon.h"
//#include "bsp_log.h"
#include "robot_def.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief ���߻ص�����,����daemon.c�б�daemon task����
 * @attention ����HAL����������,���ڿ���DMA����֮��ͬʱ�����и��ʳ���__HAL_LOCK()���µ�����,ʹ���޷�
 *            ��������ж�.ͨ��daemon�ж����ݸ���,���µ��÷������������Խ��������.
 *
 * @param id vision_usart_instance�ĵ�ַ,�˴�û��.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    //LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief ���ս���ص�����,����bsp_usart.c�б�usart rx callback����
 * @todo  1.��߿ɶ���,��get_protocol_info�ĵ��ĸ���������һ��float����buffer
 *        2.��ӱ�־λ����
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // ι��
    get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // Ϊmaster processע��daemon,�����ж��Ӿ�ͨ���Ƿ�����
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // ����ʱ���õĻص�����,���������ڽ���
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief ���ͺ���
 *
 * @param send ����������
 *
 */
void VisionSend()
{
    // buff��txlen����Ϊstatic,���ܱ�֤�ں����˳��󲻱��ͷ�,ʹ��DMA��ȷ��ɷ���
    // �������������Ҫ�ر�ע��!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // ������ת��ΪseaskyЭ������ݰ�
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // ���Ӿ�ͨ��ʹ��IT,��ֹ�ͽ���ʹ�õ�DMA��ͻ
    // �˴�ΪHAL��Ƶ�ȱ��,DMASTOP��ֹͣ���ͺͽ���,������Ҳ�޷���������ж�.
    // Ҳ���ڷ�������ж�����������DMA����,����Ϊ����.���,�˴�ʹ��IT����.
    // ��ʹ����daemon,��Ҳ����ʹ��DMA����.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

//#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    uint16_t flag_register;
    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

/* �Ӿ�ͨ�ų�ʼ�� */
/*
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // ��Ϊ����������
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // Ϊmaster processע��daemon,�����ж��Ӿ�ͨ���Ƿ�����
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // ����ʱ���õĻص�����,���������ڽ���
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}
*/

void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // ������ת��ΪseaskyЭ������ݰ�
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    //USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP