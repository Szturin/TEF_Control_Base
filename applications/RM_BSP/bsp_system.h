#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

/* 嵌入式工程文件链接树*/

/* C语言标准库 */
#include <stdint.h>             // 定义整型
#include <stdio.h>              // 标准输入输出
#include <stdlib.h>             // 标准库
#include <string.h>             // 字符串操作
#include <math.h>               // 数学库

#include "function/typedef.h"                // 类型定义


/* 硬件层 */
#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"           // RTOS API
#include "main.h"
#include "stm32f4xx_it.h"

/* 驱动层 (初始化) */
#include "gpio.h"               // GPIO驱动
#include "tim.h"                // 定时器驱动
#include "usart.h"              // 串口驱动
#include "can.h"
#include "dma.h"

/*中间层*/
//#include "freertos.h"
#include "rt-thread/include/rtthread.h"
#include "rtdevice.h"
#include "board.h"

/* 驱动层 (bsp) */
#include "can/bsp_can.h"                // CAN总线驱动
#include "uart/bsp_JY901.h"                // 陀螺仪驱动
#include "imu/JY901USE.h"            // 陀螺仪相关声明
#include "uart/bsp_remoter_uart.h"        // 遥控器串口
#include "PID/pid.h"                    // PID控制模块
#include "function/RM_Define.h"            // 定义常量和宏
#include "arm_math.h"           	// DSP库
#include "bsp_system.h"
#include "judge/JudgeMent.h"
//#include "structure_typedef.h"		// 结构体类型定义

/* 应用层 */
#include "keyboard/keyboard.h"            // 键盘控制

#include "chassic/chassis_task.h"        // 底盘任务
#include "gimbal/gimbal_task.h"            // 云台任务
#include "shoot/shoot_task.h"            // 射击任务
#include "cmd_parse/usart_task.h"
#include "ringbuffer/ringbuffer.h"
#include "PID/pid_test_app.h"

#endif /* BSP_SYSTEM_H */
