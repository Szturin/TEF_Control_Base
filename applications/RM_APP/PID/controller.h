/**
 ******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "main.h"
#include "stdint.h"
#include "memory.h"
#include "stdlib.h"
#include "arm_math.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID �Ż�����ʹ�ܱ�־λ,ͨ��λ������ж����õ��Ż�����;Ҳ���Ըĳ�λ�����ʽ
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
    PID_Integral_Limit = 0b00000001,              // 0000 0001
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
    PID_OutputFilter = 0b00010000,                // 0001 0000
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
    PID_DerivativeFilter = 0b01000000,            // 0100 0000
    PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement_e;

/* PID ��������ö��*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID�ṹ�� */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // �����޷�
    float CoefA;             // ���ٻ��� For Changing Integral
    float CoefB;             // ���ٻ��� ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // ����˲��� RC = 1/omegac
    float Derivative_LPF_RC; // ΢���˲���ϵ��

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* ����PID��ʼ���Ľṹ��*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // ����޷�
    float DeadBand; // ����

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // �����޷�
    float CoefA;         // ABΪ���ٻ��ֲ���,���ٻ���ʵ���Ͼ������˻��ַ���
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config_s;

/**
 * @brief ��ʼ��PIDʵ��
 * @todo ���޸�Ϊͳһ��PIDRegister���
 * @param pid    PIDʵ��ָ��
 * @param config PID��ʼ������
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief ����PID���
 *
 * @param pid     PIDʵ��ָ��
 * @param measure ����ֵ
 * @param ref     �趨ֵ
 * @return float  PID�������
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif