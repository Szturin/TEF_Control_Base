#include "bsp_remoter_uart.h"

static uint8_t dbus_buf[DBUS_BUFLEN];

/*全局变量 warning ！！！*/
rc_info_t rc;

//使用dma而不使用中断的情况下从uart接收数据
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

        huart->pRxBuffPtr = pData; //将一帧完整的信息复制到接收缓冲区的起始位置
        //pRxBuffptr是指向接收缓冲区的指针
        huart->RxXferSize = Size;  //传送信息的字节数
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        //开启DMA传输，将数据从uart的DR寄存器，搬运到接收缓冲区
        HAL_DMA_Start(huart->hdmarx, (uint32_t) &huart->Instance->DR, (uint32_t) pData, Size);

        //启用DMA功能
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

//获取当前dma已传输的字节数。
//DMA_stream是DMA的寄存器结构体，NDTR寄存器保存当前已传输的字节数
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
    return ((uint16_t) (dma_stream->NDTR));
}

//信息解码函数
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
    my_printf(&huart2,"ok4");
    rc->remote.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->remote.ch1 -= 1024; //减1024的目的是让摇杆不动时，即中间值变为0，这样摇杆的范围变为660 — -660
    rc->remote.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->remote.ch2 -= 1024;
    rc->remote.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->remote.ch3 -= 1024;
    rc->remote.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;//摇杆数据解析
    rc->remote.ch4 -= 1024;
    rc->remote.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->remote.sw2 = (buff[5] >> 4) & 0x0003;//拨片数据解析
    rc->remote.wheel = (buff[16] | buff[17] << 8) - 1024;//拨轮数据解析

    rc->mouse.x = (buff[6] | (buff[7] << 8));//鼠标移动数据解析
    rc->mouse.y = (buff[8] | (buff[9] << 8));
    rc->mouse.z = (buff[10] | (buff[11] << 8));
    rc->mouse.press_l = buff[12];//鼠标左右键
    rc->mouse.press_r = buff[13];

    rc->key.val = (buff[14] | (buff[15] << 8));//按键值
    rc->key.w = (rc->key.val & key_w) >> 0;//键盘按键解析
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

    //摇杆最大值减去1024后，变为660，如果超过660，则说明出现异常
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
    }//左边三种模式的切换
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
    }//右边三种模式的切换
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
	rc->key.ctrl = (rc->key.val & key_ctrl) >> 7;//键盘
}*/

//回调函数，在uart接收器检测到空闲状态时自动调用。 
//当uart的接收缓冲区中没有任何数据时，进入空闲状态。
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{

    __HAL_UART_CLEAR_IDLEFLAG(huart); //用于清除空闲标志


    if (huart == &DBUS_HUART)
    {
        my_printf(&huart2,"ok5:%d",DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance));
        __HAL_DMA_DISABLE(huart->hdmarx); //关闭DMA数据传输功能

        //当uart接收缓冲区已满时，调用信息解析函数，对信息进行处理
        if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
        {
            my_printf(&huart2,"ok3");
            rc_callback_handler(&rc, dbus_buf);
        }

        //这两行代码的作用是，告诉dma计数器，当传输的数据达到最大值时，开启dma的传输功能
        //进行下一次的数据传输
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);

        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

//检测uart是否空闲，若是，则调用接收空闲回调函数
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
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART); //清楚uart的空闲标志位，以便于进入下一个周期
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//打开uart的空闲中断

    uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}

int fputc(int ch, FILE *f)
{
    uint32_t temp = ch;

    HAL_UART_Transmit(&huart1, (uint8_t *) &temp, 1, 0xFFFF);        //huart1是串口的句柄
    HAL_Delay(2);

    return ch;
}
