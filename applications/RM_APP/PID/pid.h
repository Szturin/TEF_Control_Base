#ifndef PID_H
#define PID_H

#include "applications/RM_BSP/bsp_system.h"





//PID数据结构
typedef struct __PID_TypeDef
{
    //PID相关参数
    float Kp;            //比例系数
    float Ki;            //积分系数
    float Kd;            //微分系数

    float Output_Max;        //输出上限

    float DeadZone;            //死区上限，设为0则取消效果

    float EIS_Max;            //积分分离PID用的偏差上限【增量式PID使用】，设为一个足够大的数则取消效果
    float EAIS_Max;            //抗积分饱和PID用的偏差上限【增量式PID使用】，设为一个足够大的数则取消效果

    float Integral_Max;    //积分上限【位置式PID使用】，设为一个足够大的数则取消效果

    float Error[3];            //0：当前偏差，1：上次偏差，2：上上次偏差
    float Integral;            //积分项【位置式PID使用】

    float Output;                //当前输出
    float Output_Last;    //上一次输出【增量式PID使用】

    //函数指针
    float (*Calc)(struct __PID_TypeDef *pid, float Input, float Setpoint);
    void (*RST)(struct __PID_TypeDef *pid);
}PID_TypeDef;


void PID_Reset(PID_TypeDef *PID);

float Position_PID(PID_TypeDef *PID, float Input, float Setpoint);

float Incremental_PID(PID_TypeDef *PID, float Input, float Setpoint);

#endif

