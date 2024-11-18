#ifndef PID_H
#define PID_H

#include "applications/RM_BSP/bsp_system.h"





//PID���ݽṹ
typedef struct __PID_TypeDef
{
    //PID��ز���
    float Kp;            //����ϵ��
    float Ki;            //����ϵ��
    float Kd;            //΢��ϵ��

    float Output_Max;        //�������

    float DeadZone;            //�������ޣ���Ϊ0��ȡ��Ч��

    float EIS_Max;            //���ַ���PID�õ�ƫ�����ޡ�����ʽPIDʹ�á�����Ϊһ���㹻�������ȡ��Ч��
    float EAIS_Max;            //�����ֱ���PID�õ�ƫ�����ޡ�����ʽPIDʹ�á�����Ϊһ���㹻�������ȡ��Ч��

    float Integral_Max;    //�������ޡ�λ��ʽPIDʹ�á�����Ϊһ���㹻�������ȡ��Ч��

    float Error[3];            //0����ǰƫ�1���ϴ�ƫ�2�����ϴ�ƫ��
    float Integral;            //�����λ��ʽPIDʹ�á�

    float Output;                //��ǰ���
    float Output_Last;    //��һ�����������ʽPIDʹ�á�

    //����ָ��
    float (*Calc)(struct __PID_TypeDef *pid, float Input, float Setpoint);
    void (*RST)(struct __PID_TypeDef *pid);
}PID_TypeDef;


void PID_Reset(PID_TypeDef *PID);

float Position_PID(PID_TypeDef *PID, float Input, float Setpoint);

float Incremental_PID(PID_TypeDef *PID, float Input, float Setpoint);

#endif

