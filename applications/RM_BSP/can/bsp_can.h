#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "bsp_system.h"

extern void can_filter_init(void);

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ID = 0x200,//3508电机,发送标识符，控制 id1~4
    /*底盘相关电机*/
    CAN_3508_M1_ID = 0x201,//3508 id1(1) can1
    CAN_3508_M2_ID = 0x202,//3508 id2(2) can1
    CAN_3508_M3_ID = 0x203,//3508 id3(3) can1
    CAN_3508_M4_ID = 0x204,//3508 id4(4) can1

    CAN_SHOOT_ID = 0x1FF,//发送标识符，控制6020 id 1~4，3508 id5~8
    /*火控相关电机*/
    CAN_3508_LEFT_ID = 0x205,//3508 id5(1) can2
    CAN_3508_RIGHT_ID = 0x206,//3508 id6(2) can2
    CAN_TRIGGER_MOTOR_ID = 0x207,//6020 id3(3) can2

    CAN_GIMBAL_ID = 0X2FF,//6020电机，发送标识符，控制id 5~8
    /*云台相关电机*/
    CAN_YAW_MOTOR_ID = 0x209,//6020 id5(1) can2
    CAN_PIT_MOTOR_1_ID = 0x20A,//6020 id6(2) can2
    CAN_PIT_MOTOR_2_ID = 0x20B,//6020 id7(3) can2

}can_msg_id_e;

//rm motor data
typedef struct
{
    int16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    int16_t temperate;
    int16_t last_ecd;
    fp32 speed;
    fp32 speed_set;
}motor_measure_t;


typedef struct
{
    int16_t targrt_1;
    int16_t targrt_2;
    int16_t targrt_3;
    int16_t targrt_4;

}CANSend_TypeDef;


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(CANSend_TypeDef *motor);

void CAN_cmd_shoot(CANSend_TypeDef *motor);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

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
extern void CAN_cmd_chassis(CANSend_TypeDef *motor);    //发送数据
void Driver_MOTOR_ReadData(motor_measure_t *msg, uint8_t *rx_data);
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
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

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
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

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
extern const motor_measure_t *get_trigger_motor_measure_point(void);

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
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif


