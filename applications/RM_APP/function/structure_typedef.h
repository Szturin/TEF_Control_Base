#ifndef STRUCTURE_TYPEDEF_H
#define STRUCTURE_TYPEDEF_H

#include "applications/RM_BSP/bsp_system.h"
#include "can/bsp_can.h"

// ���岻��
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
    CANSend_TypeDef RM3510;    //RM3510���CAN��������

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
    uint16_t currentAngle;     //��������Ƕ�
    uint16_t currentSpeed;     //����ת��    rpm��ֱ����˼�Ի�Ϊ r/min ��תȦÿ���ӣ�, 1rpm=1 r/min
    uint16_t currentMa;        //���ʵ��ת�صõ���
    uint16_t currentTemp;      //����¶�
    uint16_t currentDatafinally;//���ڼ���Ƿ��յ�����


}motor3510_TypeDef;

#endif
