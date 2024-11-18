#include "MotorControl_app.h"

float CM1Speed_Target = 0.0f;
float CM2Speed_Target = 0.0f;
float CM3Speed_Target = 0.0f;
float CM4Speed_Target = 0.0f;

PID_TypeDef CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;;
PID_TypeDef CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_TypeDef CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_TypeDef CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

//获取速度环
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
函数名：CMSpeedAdjust
功能：底盘速度调节调整    pid的计算
入口参数：	angle_target：目标角度
			flag_reset：是否复位PID参数标志：1：复位PID；0：PID计算
返回值：无
应用范围：外部调用
备注：
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
        //速度环
        CM1SpeedPID.Calc(&CM1SpeedPID, DATASPEED1.given_current, speed1_target);
        CM2SpeedPID.Calc(&CM2SpeedPID, DATASPEED2.given_current, speed2_target);
        CM3SpeedPID.Calc(&CM3SpeedPID, DATASPEED3.given_current, speed3_target);
        CM4SpeedPID.Calc(&CM4SpeedPID, DATASPEED4.given_current, speed4_target);
    }
}

/*
***************************************************
函数名：CMControl
功能：底盘控制
入口参数：	CANSend_TypeDef：RM3510电机发送结构体指针
返回值：无
应用范围：外部调用
备注：  输出值调用
***************************************************
*/

void CMControl(CANSend_TypeDef *RM3510_CANSend)  //传递一个用户给的数值
{

    CM1Speed_Target = 0;
    CM2Speed_Target = 0;
    CM3Speed_Target = 0;
    CM4Speed_Target = 0;

    CMSpeedAdjust(CM1Speed_Target, CM2Speed_Target, CM3Speed_Target, CM4Speed_Target, flag_CMSpeedPIDRST);

    // 意义不明
    //	Sendresult.targrt_201 = (short)CM1SpeedPID.Output;
    //	Sendresult.targrt_202 = (short)CM2SpeedPID.Output;
    //	Sendresult.targrt_203 = (short)CM3SpeedPID.Output;
    //	Sendresult.targrt_204 = (short)CM4SpeedPID.Output;
}









