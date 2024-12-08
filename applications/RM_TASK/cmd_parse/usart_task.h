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

//���������Ľṹ��
typedef struct {
    uint8_t data_type; // ��������
	uint8_t data;		//����
} DataPacket;

int parse_buffer(uint8_t *buffer,size_t size,DataPacket* data);
int task_parse_buffer(uint8_t *buffer,size_t size,DataPacket* task_data);
void AutoAim_DeviceInit();
void uart_sem_init();

/****����ṹ��****/
typedef struct {
    uint8_t auto_aim_enabled;  // �Զ���׼������־��1 ��ʾ������0 ��ʾ�ر�
    float pitch_offset;        // Pitch ���ƫ��������λ�����Ƕ�
    float yaw_offset;          // Yaw ���ƫ��������λ�����Ƕ�
} AutoAimData;

extern DataPacket context_task;
extern AutoAimData AutoAim_device1;

#endif



