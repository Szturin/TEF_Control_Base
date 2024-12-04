#include "usart_task.h"

uint8_t uart2_rx_dma_buffer[1024]={0};//串口2DMA缓冲区

ringbuffer_t uart2_rb; //定义ringbuffer_t类型结构体变量
uint8_t uart2_read_buffer[1024];//定义环形缓存区数组

DataPacket context; // 初始化上下文
DataPacket context_task; // 初始化任务上下文

static struct rt_semaphore uart2_sem;//定义信号量

AutoAimData AutoAim_device1;//自瞄，对象设备1
// 给成员赋值
int SOC_Parse_buffer(uint8_t *buffer, size_t size, AutoAimData* aim_data);

void AutoAim_DeviceInit()
{
    AutoAim_device1.auto_aim_enabled = 0;//默认关闭自瞄
    AutoAim_device1.pitch_offset = 0;
    AutoAim_device1.yaw_offset = 0;
}

//空闲中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if(huart->Instance == USART2)
    {
        if(!ringbuffer_is_full(&uart2_rb))
        {
            //my_printf(&huart2,"dma receive\r\n");

            ringbuffer_write(&uart2_rb,uart2_rx_dma_buffer,Size);
        }
        memset(uart2_rx_dma_buffer,0,sizeof(uart2_rx_dma_buffer));
    }
}


void uart_proc(void)
{
	// 如果环形缓冲区为空，直接返回 
	if( ringbuffer_is_empty(&uart2_rb)) return;

    // 从环形缓冲区读取数据到读取缓冲区
    ringbuffer_read(&uart2_rb, uart2_read_buffer, uart2_rb.itemCount);

   // my_printf(&huart2,"proc is running");

    //PID上位机调整解析
    PID_UpdateFromCommand(&PID_Test,(const char *)uart2_read_buffer);

    //调试解析，将收到的串口数据帧解析，解析的数据放在context报文中
    parse_buffer(uart2_read_buffer,sizeof(uart2_read_buffer),&context);

    //任务解析
    task_parse_buffer(uart2_read_buffer,sizeof(uart2_read_buffer),&context_task);

    SOC_Parse_buffer(uart2_read_buffer,sizeof(uart2_read_buffer),&AutoAim_device1);

    memset(uart2_read_buffer, 0, sizeof(uart2_read_buffer));
}

void uart_sem_init()
{
    rt_sem_init(&uart2_sem,"uart2_sem",0,RT_IPC_FLAG_FIFO);
}

//数据帧解析函数
int parse_buffer(uint8_t *buffer,size_t size,DataPacket* data)
{
	if(size < 4)return 0;//数据帧长度小于4，返回 0 表示解析失败
	
	if(buffer[0] == 0xFF && buffer[3] == 0xFB)//帧头帧尾检测
	{
		if(buffer[1] == 0x2A){
			data -> data_type = 1;//正数
			data -> data = buffer[2];
		}
		else if(buffer[1] == 0x2B){
			data -> data_type = 2;//负数/** */
			data -> data = buffer[2];
		}		
		else{
			return 0; //非正确类型，解析失败
		}
		return 1;//解析成功
	}
	else{
		return 0; //帧头帧尾错误，解析失败
	}
}


// 任务帧解析函数
// 上位机发送0xAA 0x01 0x00 0xFF表示任务未完成
// 上位机发送0xAA 0x01 0x01 0xFF表示任务完成
int task_parse_buffer(uint8_t *buffer, size_t size, DataPacket* task_data) {
    if (size < 4) return 0; // 数据帧长度小于4，返回0表示解析失败
    
    if (buffer[0] == 0xAA && buffer[3] == 0xFF) { // 帧头帧尾检测
        task_data->data = buffer[1]; // 解析任务指令
        
        // 判断任务完成的状态
        if (buffer[2] == 0x01) {
            task_data->data_type = 1; // 任务已完成
        } else {
            task_data->data_type = 0; // 任务未完成
        }

        return 1; // 解析成功
    } else {
        return 0; // 帧头帧尾错误，解析失败
    }
}

int SOC_Parse_buffer(uint8_t *buffer, size_t size, AutoAimData* aim_data)
{
    if (size < 6) return 0; //数据帧小于6，认为是无效帧 (包括帧头0xDD，数据和校验和)
   // my_printf(&huart2,"proc is running");
    if (buffer[0] == 0xDD) // 检测到帧头0xDD
    {

        // 解析yaw和pitch的偏移量（数据2是yaw，数据4是pitch）
        uint16_t data1 = buffer[2] << 8 | buffer[1]; // Yaw 偏移量

        uint16_t data2 = buffer[4] << 8 | buffer[3]; // Pitch 偏移量


        // 校验位计算
        uint8_t sum = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4];

        // 校验和对比
        if (sum == buffer[5])
        {
            // 将raw数据转换为角度偏移
            aim_data->yaw_offset = ((float)data1 / 65535.0f) * 60.0f - 30.0f;
            aim_data->pitch_offset = ((float)data2 / 65535.0f) * 60.0f - 30.0f;

            // 调试输出
            //my_printf(&huart2, "yaw: %.2f, pitch: %.2f\r\n", aim_data->yaw_offset, aim_data->pitch_offset);

            return 1; // 解析成功
        }
        else
        {
            // 校验失败
            return 0;
        }
    }
    else
    {
        // 帧头不匹配
        return 0;
    }
}



