#include "bsp_remoter_uart.h"

static uint8_t dbus_buf[DBUS_BUFLEN];

/*ȫ�ֱ��� warning ������*/
rc_info_t rc;

//ʹ��dma����ʹ���жϵ�����´�uart��������
static int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
    uint32_t tmp1 = 0;

    tmp1 = huart->RxState;

    if (tmp1 == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        huart->pRxBuffPtr = pData; //��һ֡��������Ϣ���Ƶ����ջ���������ʼλ��
        //pRxBuffptr��ָ����ջ�������ָ��
        huart->RxXferSize = Size;  //������Ϣ���ֽ���
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        //����DMA���䣬�����ݴ�uart��DR�Ĵ��������˵����ջ�����
        HAL_DMA_Start(huart->hdmarx, (uint32_t) &huart->Instance->DR, (uint32_t) pData, Size);

        //����DMA����
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

//��ȡ��ǰdma�Ѵ�����ֽ�����
//DMA_stream��DMA�ļĴ����ṹ�壬NDTR�Ĵ������浱ǰ�Ѵ�����ֽ���
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
    return ((uint16_t) (dma_stream->NDTR));
}

//��Ϣ���뺯��
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
    my_printf(&huart2,"ok4");
    rc->remote.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->remote.ch1 -= 1024; //��1024��Ŀ������ҡ�˲���ʱ�����м�ֵ��Ϊ0������ҡ�˵ķ�Χ��Ϊ660 �� -660
    rc->remote.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->remote.ch2 -= 1024;
    rc->remote.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->remote.ch3 -= 1024;
    rc->remote.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;//ҡ�����ݽ���
    rc->remote.ch4 -= 1024;
    rc->remote.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->remote.sw2 = (buff[5] >> 4) & 0x0003;//��Ƭ���ݽ���
    rc->remote.wheel = (buff[16] | buff[17] << 8) - 1024;//�������ݽ���

    rc->mouse.x = (buff[6] | (buff[7] << 8));//����ƶ����ݽ���
    rc->mouse.y = (buff[8] | (buff[9] << 8));
    rc->mouse.z = (buff[10] | (buff[11] << 8));
    rc->mouse.press_l = buff[12];//������Ҽ�
    rc->mouse.press_r = buff[13];

    rc->key.val = (buff[14] | (buff[15] << 8));//����ֵ
    rc->key.w = (rc->key.val & key_w) >> 0;//���̰�������
    rc->key.s = (rc->key.val & key_s) >> 1;
    rc->key.a = (rc->key.val & key_a) >> 2;
    rc->key.d = (rc->key.val & key_d) >> 3;
    rc->key.shift = (rc->key.val & key_shift) >> 4;
    rc->key.ctrl = (rc->key.val & key_ctrl) >> 5;
    rc->key.q = (rc->key.val & key_q) >> 6;
    rc->key.e = (rc->key.val & key_e) >> 7;
    rc->key.r = (rc->key.val & key_r) >> 8;
    rc->key.f = (rc->key.val & key_f) >> 9;
    rc->key.g = (rc->key.val & key_g) >> 10;
    rc->key.z = (rc->key.val & key_z) >> 11;
    rc->key.x = (rc->key.val & key_x) >> 12;
    rc->key.c = (rc->key.val & key_c) >> 13;
    rc->key.v = (rc->key.val & key_v) >> 14;
    rc->key.b = (rc->key.val & key_b) >> 15;

    //ҡ�����ֵ��ȥ1024�󣬱�Ϊ660���������660����˵�������쳣
    if ((abs(rc->remote.ch1) > 660) || (abs(rc->remote.ch1) > 660) || (abs(rc->remote.ch3) > 660) || (abs(rc->remote.ch4) > 660))
    {
        memset(rc, 0, sizeof(rc_info_t));
    }

    switch (rc->remote.sw1)
    {
    case 1:
        rc->control_mode1 = rc_OFF;
        break;
    case 3:
        rc->control_mode1 = rc_CL;
        break;
    case 2:
        rc->control_mode1 = rc_HL;
        break;
    }//�������ģʽ���л�
    switch (rc->remote.sw2)
    {
    case 1:
        rc->control_mode2 = rc_GPS;
        break;
    case 3:
        rc->control_mode2 = rc_ATT1;
        break;
    case 2:
        rc->control_mode2 = rc_ATT2;
        break;
    }//�ұ�����ģʽ���л�
}

/*void keyboard_callback(rc_info_t *rc)
{
	rc->key.w = (rc->key.val & key_w) >> 0;
	rc->key.s = (rc->key.val & key_s) >> 1;
	rc->key.a = (rc->key.val & key_a) >> 2;
	rc->key.d = (rc->key.val & key_d) >> 3;
	rc->key.q = (rc->key.val & key_q) >> 4;
	rc->key.e = (rc->key.val & key_e) >> 5;	
	rc->key.shift = (rc->key.val & key_shift) >> 6;
	rc->key.ctrl = (rc->key.val & key_ctrl) >> 7;//����
}*/

//�ص���������uart��������⵽����״̬ʱ�Զ����á� 
//��uart�Ľ��ջ�������û���κ�����ʱ���������״̬��
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{

    __HAL_UART_CLEAR_IDLEFLAG(huart); //����������б�־


    if (huart == &DBUS_HUART)
    {
        my_printf(&huart2,"ok5:%d",DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance));
        __HAL_DMA_DISABLE(huart->hdmarx); //�ر�DMA���ݴ��书��

        //��uart���ջ���������ʱ��������Ϣ��������������Ϣ���д���
        if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
        {
            my_printf(&huart2,"ok3");
            rc_callback_handler(&rc, dbus_buf);
        }

        //�����д���������ǣ�����dma������������������ݴﵽ���ֵʱ������dma�Ĵ��书��
        //������һ�ε����ݴ���
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);

        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

//���uart�Ƿ���У����ǣ�����ý��տ��лص�����
void uart_receive_handler(UART_HandleTypeDef *huart)
{
    //my_printf(&huart2,"ok1");
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
       // my_printf(&huart2,"ok2");
        uart_rx_idle_callback(huart);
    }
}

void dbus_uart_init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART); //���uart�Ŀ��б�־λ���Ա��ڽ�����һ������
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//��uart�Ŀ����ж�

    uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}

int fputc(int ch, FILE *f)
{
    uint32_t temp = ch;

    HAL_UART_Transmit(&huart1, (uint8_t *) &temp, 1, 0xFFFF);        //huart1�Ǵ��ڵľ��
    HAL_Delay(2);

    return ch;
}
