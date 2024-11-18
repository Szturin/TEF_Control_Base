#include "applications/RM_BSP/bsp_system.h"
#include "chassic/chassis.h"
/* defined the LED_G pin: PF14 */
#define LED0_PIN    GET_PIN(F, 14)
#define LED1_PIN    GET_PIN(E, 11)

/*创建线程*/
static rt_thread_t tid1 = RT_NULL;//储存线程信息
static rt_thread_t tid2 = RT_NULL;//储存线程信息
static rt_thread_t thread_gimbal = RT_NULL;  //云台线程
static rt_thread_t thread_chassis = RT_NULL; //底盘线程
static rt_thread_t thread_shoot = RT_NULL;   //射击线程
static rt_thread_t thread_uart = RT_NULL;    //串口线程

void RM_thread_create(void);

/*线程1入口函数*/
static void thread1_entry(void  *parameter)
{
    while(1){
        /*rt-thread封装的引脚功能函数*/
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(100);
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(100);
    }
}

static void thread2_entry(void  *parameter)
{
    while(1){
        /*Hal库封装的引脚功能函数*/
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
        rt_thread_mdelay(100);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
        rt_thread_mdelay(100);
    }
}

/*线程 云台 入口函数*/
static void gimbal_entry(void  *parameter){
    gimbal_init();
    while(1){
        gimbaltask();
        //my_printf(&huart2,"云台运行中...\r\n");scxxsc
        rt_thread_mdelay(5);
    }
}

/*线程 底盘 入口函数*/
static void chassis_entry(void  *parameter){
    while(1){
        chassistask();
        //ChassisTask();
        //my_printf(&huart2,"底盘运行中...\r\n");
        rt_thread_mdelay(5);
    }
}

/*线程 射击 入口函数*/
static void shoot_entry(void  *parameter){
    while(1){
        shoottask();
        //my_printf(&huart2,"shoot is running...\r\n");
        rt_thread_mdelay(5);
    }
}

/*线程 射击 入口函数*/
static void uart_entry(void  *parameter){
    while(1){
        uart_proc();
        rt_thread_mdelay(5);// tip：如果这里周期设置过长，时间片又只有10ms,没有成功解析就退出了线程
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    dbus_uart_init();
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    MX_CAN2_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_UART8_Init();
    MX_USART3_UART_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_UART7_Init();
    User_USART_Init(&JY901_data);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); //使能IDLE中断   裁判系统读取状态用
    HAL_UART_Receive_DMA(&huart6, RxBuff, RxBuff_SIZE);//打开DMA接收
    can_filter_init();
    ringbuffer_init(&uart2_rb);
    AutoAim_DeviceInit();

    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    /*创建线程1*/
    tid1 = rt_thread_create("thread1",
                            thread1_entry,RT_NULL,
                            2048,20, 10);

    /*创建线程2*/
    tid2 = rt_thread_create("thread2",
                            thread2_entry,RT_NULL,
                            2048,20, 10);

    RM_thread_create();

    /*启动线程*/
    //if(tid1 != RT_NULL){rt_thread_startup(tid1);}//线程1不为空，激活线程
    //if(tid2 != RT_NULL){rt_thread_startup(tid2);}//线程2不为空，激活线程
    if(thread_gimbal != RT_NULL){rt_thread_startup(thread_gimbal);}
    if(thread_chassis != RT_NULL){rt_thread_startup(thread_chassis);}
    if(thread_shoot != RT_NULL){rt_thread_startup(thread_shoot);}
    if(thread_uart != RT_NULL){rt_thread_startup(thread_uart);}
    return 0;
}

void RM_thread_create(void)
{
    thread_gimbal = rt_thread_create("thread3",
                                     gimbal_entry,
                                     RT_NULL,
                                     4096,20,10);
    thread_chassis = rt_thread_create("thread4",
                                      chassis_entry,
                                      RT_NULL,
                                      4096,20,10);
    thread_shoot = rt_thread_create("thread5",
                                      shoot_entry,
                                      RT_NULL,
                                      4096,20,10);
    thread_uart = rt_thread_create("thread6",
                                   uart_entry,
                                   RT_NULL,
                                  4096,20,15);
}