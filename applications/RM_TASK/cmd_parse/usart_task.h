#ifndef UART_TASK_H
#define UART_TASK_H

#include "applications/RM_BSP/bsp_system.h"
#include "string.h"

void uart_proc(void);
int My_printf(UART_HandleTypeDef *huart, const char *format, ...);

extern uint8_t uart2_rx_dma_buffer[1024];
extern uint8_t uart2_read_buffer[1024];

extern uint8_t uart3_rx_dma_buffer[1024];
extern uint8_t uart3_read_buffer[1024];

//接收上下文结构体
typedef struct {
    uint8_t data_type; // 数据类型
	uint8_t data;		//内容
} DataPacket;

int parse_buffer(uint8_t *buffer,size_t size,DataPacket* data);
int task_parse_buffer(uint8_t *buffer,size_t size,DataPacket* task_data);
void AutoAim_DeviceInit();
void uart_sem_init();

/****自瞄结构体****/
typedef struct {
    uint8_t auto_aim_enabled;  // 自动瞄准开启标志，1 表示开启，0 表示关闭
    float pitch_offset;        // Pitch 轴的偏移量，单位可以是度
    float yaw_offset;          // Yaw 轴的偏移量，单位可以是度
} AutoAimData;

extern DataPacket context_task;
extern AutoAimData AutoAim_device1;

#endif



