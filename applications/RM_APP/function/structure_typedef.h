#ifndef STRUCTURE_TYPEDEF_H
#define STRUCTURE_TYPEDEF_H

#include "applications/RM_BSP/bsp_system.h"
#include "can/bsp_can.h"

// 意义不明
//typedef struct 
//{
//	uint16_t targrt_201;
//	uint16_t targrt_202;
//	uint16_t targrt_203;
//	uint16_t targrt_204;
//	
//}CANSend_TypeDef;


typedef struct
{
    CANSend_TypeDef RM3510;    //RM3510电机CAN发送数据

}Driver_RMMotor_CANSend_TypeDef;

typedef struct
{
    uint16_t CMotor1Speed_Sendvalue;
    uint16_t CMotor2Speed_Sendvalue;
    uint16_t CMotor3Speed_Sendvalue;
    uint16_t CMotor4Speed_Sendvalue;
    uint16_t i;
}SendSpeed_Target_TypeDef;


typedef struct
{
    uint16_t currentAngle;     //读出电机角度
    uint16_t currentSpeed;     //读出转速    rpm的直接意思对换为 r/min （转圈每分钟）, 1rpm=1 r/min
    uint16_t currentMa;        //电机实际转矩得电流
    uint16_t currentTemp;      //电机温度
    uint16_t currentDatafinally;//用于检测是否收到数据


}motor3510_TypeDef;

#endif
