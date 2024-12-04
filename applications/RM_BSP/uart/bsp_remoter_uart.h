#ifndef BSP_REMOTER_UART_H
#define BSP_REMOTER_UART_H

#include "bsp_system.h"


#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1
#define key_w     ((uint16_t)0x01<<0)
#define key_s     ((uint16_t)0x01<<1)
#define key_a     ((uint16_t)0x01<<2)
#define key_d     ((uint16_t)0x01<<3)
#define key_q        ((uint16_t)0x01<<6)
#define key_e        ((uint16_t)0x01<<7)
#define key_shift ((uint16_t)0x01<<4)
#define key_ctrl    ((uint16_t)0x01<<5)
#define key_r     ((uint16_t)0x01<<8)
#define key_f     ((uint16_t)0x01<<9)
#define key_g        ((uint16_t)0x01<<10)
#define key_z        ((uint16_t)0x01<<11)
#define key_x     ((uint16_t)0x01<<12)
#define key_c     ((uint16_t)0x01<<13)
#define key_v        ((uint16_t)0x01<<14)
#define key_b        ((uint16_t)0x01<<15)

typedef struct
{
    struct
    {
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;//·½ÏòÒ¡¸Ë
        int16_t wheel;//²¦ÂÖ

        uint8_t sw1;
        uint8_t sw2;//²¦Æ¬

    }remote;//Ò£¿Ø¿ØÖÆ

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;//Êó±êxyzÖáÒÆ¶¯

        uint8_t press_l;
        uint8_t press_r;//Êó±ê×óÓÒ¼ü

    }mouse;//Êó±ê¿ØÖÆ

    struct
    {
        uint16_t val;
        uint16_t w;
        uint16_t s;
        uint16_t a;
        uint16_t d;
        uint16_t shift;
        uint16_t ctrl;
        uint16_t q;
        uint16_t e;
        uint16_t r;
        uint16_t f;
        uint16_t g;
        uint16_t z;
        uint16_t x;
        uint16_t c;
        uint16_t v;
        uint16_t b;
    }key;//¼üÅÌ¿ØÖÆ

    enum
    {
        rc_OFF,
        rc_CL,
        rc_HL,
    }control_mode1;//×ó²¦Æ¬µÄÈýÖÖÄ£Ê½

    enum
    {
        rc_GPS,
        rc_ATT1,
        rc_ATT2,
    }control_mode2;//ÓÒ²¦Æ¬µÄÈýÖÖÄ£Ê½

}rc_info_t;

void uart_receive_handler(UART_HandleTypeDef *huart);

void dbus_uart_init(void);

void USART_SendData(USART_TypeDef *USARTx, uint16_t Data);

extern rc_info_t rc;
#endif

/*
¼üÅÌ
Bit0------W     1
Bit1------S     2
Bit2------A     4
Bit3------D     8
Bit4------Shift 10
Bit5------Ctrl  20
Bit6------Q     40 
Bit7------E     80
Bit8------R     100
Bit9------F     200
Bit10-----G     400
Bit11-----Z     800
Bit12-----X     1000
Bit13-----C     2000
Bit14-----V     4000
Bit15-----B     8000
*/
