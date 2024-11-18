#include "MotorControl_app.h"

float CM1Speed_Target = 0.0f;
float CM2Speed_Target = 0.0f;
float CM3Speed_Target = 0.0f;
float CM4Speed_Target = 0.0f;

PID_TypeDef CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;;
PID_TypeDef CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_TypeDef CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_TypeDef CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

//��ȡ�ٶȻ�
extern motor_measure_t DATASPEED1;
extern motor_measure_t DATASPEED2;
extern motor_measure_t DATASPEED3;
extern motor_measure_t DATASPEED4;
//extern float CM1Speed_Target;

CANSend_TypeDef Sendresult;
CANSend_TypeDef targrt_Speed;
uint8_t flag_CMSpeedPIDRST = 0;

/*
***************************************************
��������CMSpeedAdjust
���ܣ������ٶȵ��ڵ���    pid�ļ���
��ڲ�����	angle_target��Ŀ��Ƕ�
			flag_reset���Ƿ�λPID������־��1����λPID��0��PID����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void CMSpeedAdjust(float speed1_target, float speed2_target, float speed3_target, float speed4_target, uint8_t flag_reset)
{

    if (flag_reset)
    {
        CM1SpeedPID.RST(&CM1SpeedPID);
        CM2SpeedPID.RST(&CM2SpeedPID);
        CM3SpeedPID.RST(&CM3SpeedPID);
        CM4SpeedPID.RST(&CM4SpeedPID);
    }
    else
    {
        //�ٶȻ�
        CM1SpeedPID.Calc(&CM1SpeedPID, DATASPEED1.given_current, speed1_target);
        CM2SpeedPID.Calc(&CM2SpeedPID, DATASPEED2.given_current, speed2_target);
        CM3SpeedPID.Calc(&CM3SpeedPID, DATASPEED3.given_current, speed3_target);
        CM4SpeedPID.Calc(&CM4SpeedPID, DATASPEED4.given_current, speed4_target);
    }
}

/*
***************************************************
��������CMControl
���ܣ����̿���
��ڲ�����	CANSend_TypeDef��RM3510������ͽṹ��ָ��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��  ���ֵ����
***************************************************
*/

void CMControl(CANSend_TypeDef *RM3510_CANSend)  //����һ���û�������ֵ
{

    CM1Speed_Target = 0;
    CM2Speed_Target = 0;
    CM3Speed_Target = 0;
    CM4Speed_Target = 0;

    CMSpeedAdjust(CM1Speed_Target, CM2Speed_Target, CM3Speed_Target, CM4Speed_Target, flag_CMSpeedPIDRST);

    // ���岻��
    //	Sendresult.targrt_201 = (short)CM1SpeedPID.Output;
    //	Sendresult.targrt_202 = (short)CM2SpeedPID.Output;
    //	Sendresult.targrt_203 = (short)CM3SpeedPID.Output;
    //	Sendresult.targrt_204 = (short)CM4SpeedPID.Output;
}









