#include "bsp_can.h"

//问题1：发送向谁发，接收从哪接收
//问题2：数据解析的原理
//问题3：can滤波器配置原理
//问题4：can的寄存器
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
CANSend_TypeDef  Sendresult; //速度的接口API
CANSend_TypeDef  targrt_Speed; //速度的接口API
motor_measure_t MOTORDATA1;
motor_measure_t MOTORDATA2;
motor_measure_t MOTORDATA3;
motor_measure_t MOTORDATA4;//底盘数据

motor_measure_t RIGHTMOTORDATA;
motor_measure_t LEFTMOTORDATA;

motor_measure_t DATAGIMBALYAW;
motor_measure_t DATAGIMBALPITCH_1;
motor_measure_t DATAGIMBALPITCH_2;
motor_measure_t DATATRIGGER;
int flag_num = 5;

#define CAN_SEND_TIMEOUT 10 // 超时时间 10 个 tick

void can_filter_init(void)  //can掩码器
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];

static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch(rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    {
        Driver_MOTOR_ReadData(&MOTORDATA1,rx_data);//底盘轮1数据获取
    }	break;
    case CAN_3508_M2_ID:
    {
        Driver_MOTOR_ReadData(&MOTORDATA2,rx_data);//底盘轮2数据获取
    }break;
    case CAN_3508_M3_ID:
    {
        Driver_MOTOR_ReadData(&MOTORDATA3,rx_data);//底盘轮3数据获取
    }break;
    case CAN_3508_M4_ID:
    {
        Driver_MOTOR_ReadData(&MOTORDATA4,rx_data);//底盘轮4数据获取
    }break;
    case CAN_3508_RIGHT_ID:
    {
        Driver_MOTOR_ReadData(&RIGHTMOTORDATA,rx_data);//摩擦轮右数据获取
    }break;
    case CAN_3508_LEFT_ID:
    {
        Driver_MOTOR_ReadData(&LEFTMOTORDATA,rx_data);//摩擦轮左数据获取
    }break;
    case CAN_TRIGGER_MOTOR_ID:
    {
        Driver_MOTOR_ReadData(&DATATRIGGER,rx_data);//获取拨弹电机数据；
    }break;
    case CAN_YAW_MOTOR_ID:
    {
        Driver_MOTOR_ReadData(&DATAGIMBALYAW,rx_data);//获取云台yaw轴电机数据
    }break;
    case CAN_PIT_MOTOR_1_ID:
    {
        Driver_MOTOR_ReadData(&DATAGIMBALPITCH_1,rx_data);//获取云台pitch轴电机数据
    }break;
    case CAN_PIT_MOTOR_2_ID:
    {
        Driver_MOTOR_ReadData(&DATAGIMBALPITCH_2,rx_data);//获取云台pitch轴电机数据
    }break;

    }
}

void Driver_MOTOR_ReadData(motor_measure_t *MOTOR_Data,uint8_t *rx_data)
{
    MOTOR_Data->ecd= (int16_t)((rx_data[0] << 8) | rx_data[1]);
    MOTOR_Data->speed_rpm= (int16_t)((rx_data[2] << 8) | rx_data[3]);
    MOTOR_Data->given_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    MOTOR_Data->temperate= ( int16_t)rx_data[6];
    MOTOR_Data->last_ecd=( int16_t)rx_data[7];
}

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(CANSend_TypeDef *motor)
{
    uint32_t send_mail_box;
    rt_tick_t start_tick = rt_tick_get();//记录当前时间戳
    gimbal_tx_message.StdId = CAN_GIMBAL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (uint8_t)(motor->targrt_1>>8);
    gimbal_can_send_data[1] = (uint8_t)(motor->targrt_1);
    gimbal_can_send_data[2] = (uint8_t)(motor->targrt_2>>8);
    gimbal_can_send_data[3] = (uint8_t)(motor->targrt_2);
    gimbal_can_send_data[4] = (uint8_t)(motor->targrt_3>>8);
    gimbal_can_send_data[5] = (uint8_t)(motor->targrt_3);
    gimbal_can_send_data[6] = (uint8_t)(motor->targrt_4>>8);
    gimbal_can_send_data[7] = (uint8_t)(motor->targrt_4);

    /*超时退出，防止阻塞进程*/


    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)==0){
        if(rt_tick_get() - start_tick >= 20){
            return;
        }
    }

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_shoot(CANSend_TypeDef *motor)
{
    uint32_t send_mail_box;

    rt_tick_t start_tick = rt_tick_get();//记录当前时间戳
    shoot_tx_message.StdId = CAN_SHOOT_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (uint8_t)(motor->targrt_1>>8);
    shoot_can_send_data[1] = (uint8_t)(motor->targrt_1);
    shoot_can_send_data[2] = (uint8_t)(motor->targrt_2>>8);
    shoot_can_send_data[3] = (uint8_t)(motor->targrt_2);
    shoot_can_send_data[4] = (uint8_t)(motor->targrt_3>>8);
    shoot_can_send_data[5] = (uint8_t)(motor->targrt_3);
    shoot_can_send_data[6] = (uint8_t)(motor->targrt_4>>8);
    shoot_can_send_data[7] = (uint8_t)(motor->targrt_4);

    /*超时退出，防止阻塞进程*/

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)==0){
        if(rt_tick_get() - start_tick >= 20){
            return;
       }
    }


    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(CANSend_TypeDef *motor)
{
    uint32_t send_mail_box;
    rt_tick_t start_tick = rt_tick_get();//记录当前时间戳

    chassis_tx_message.StdId = CAN_CHASSIS_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = (uint8_t)(motor->targrt_1>>8);
    chassis_can_send_data[1] = (uint8_t)(motor->targrt_1);
    chassis_can_send_data[2] = (uint8_t)(motor->targrt_2>>8);
    chassis_can_send_data[3] = (uint8_t)(motor->targrt_2);
    chassis_can_send_data[4] = (uint8_t)(motor->targrt_3>>8);
    chassis_can_send_data[5] = (uint8_t)(motor->targrt_3);
    chassis_can_send_data[6] = (uint8_t)(motor->targrt_4>>8);
    chassis_can_send_data[7] = (uint8_t)(motor->targrt_4);

    /*超时退出，防止阻塞进程*/
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0){
        if(rt_tick_get() - start_tick >= 20){
            return;
        }
    }


    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    //return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    // return &motor_chassis[5];
}

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    //  return &motor_chassis[6];
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    // return &motor_chassis[(i & 0x03)];
}

