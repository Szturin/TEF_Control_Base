#include "pid.h"

void PID_Reset(PID_TypeDef *PID)
{
    PID->Error[0] = PID->Error[1] = PID->Error[2] = 0;
    PID->Integral = 0;

    PID->Output = 0;
    PID->Output_Last = 0;
}

//位置式PID
float Position_PID(PID_TypeDef *PID, float CurrentValue, float TargetValue)
{
    PID->Error[0] = TargetValue - CurrentValue;

    /************************根据偏差对PID参数进行调整***************************/
    //死区计算，设为0则取消效果
    if ((PID->Error[0] <= PID->DeadZone) && (PID->Error[0] >= -PID->DeadZone))
    {
        PID->Error[0] = 0.0f;
    }
    else
    {
        if ((PID->EIS_Max == 0) || ((PID->Error[0] <= PID->EIS_Max) && (PID->Error[0] >= -PID->EIS_Max)))
        {
            //积分限幅，设为一个足够大的数则取消效果
            PID->Integral += PID->Error[0] * PID->Ki;

            if (PID->Integral > PID->Integral_Max)
            {
                PID->Integral = PID->Integral_Max;
            }
            else if (PID->Integral < -PID->Integral_Max)
            {
                PID->Integral = -PID->Integral_Max;
            }
        }
    }

    /********************************PID计算***************************************/
    PID->Output = PID->Kp * PID->Error[0] + PID->Integral + PID->Kd * (PID->Error[0] - PID->Error[1]);

    /****************************为下次PID计算做准备********************************/
    PID->Error[1] = PID->Error[0];

    /************************************输出限幅***********************************/
    if (PID->Output > PID->Output_Max)
    {
        PID->Output = PID->Output_Max;
    }
    else if (PID->Output < -PID->Output_Max)
    {
        PID->Output = -PID->Output_Max;
    }

    return PID->Output;
}

//增量式PID
float Incremental_PID(PID_TypeDef *PID, float CurrentValue, float TargetValue)
{
    float A0 = PID->Kp + PID->Kd;
    float A1 = -PID->Kp - 2 * PID->Kd;
    float A2 = PID->Kd;
    char KL = 1;

    PID->Error[0] = TargetValue - CurrentValue;

    /************************根据偏差对PID参数进行调整***************************/
    //死区计算，设为0则取消效果
    if ((PID->Error[0] <= PID->DeadZone) && (PID->Error[0] >= -PID->DeadZone)){
        PID->Error[0] = 0.0;
    }

    //积分分离，设为一个足够大的数则取消效果 *
    if ((PID->Error[0] <= PID->EIS_Max) && (PID->Error[0] >= -PID->EIS_Max)){
        KL = 1;
    }
    else{
        KL = 0;
    }
    //抗积分饱和，设为一个足够大的数则取消效果
    if ((PID->Output_Last >= PID->EAIS_Max && PID->Error[0] >= 0) || (PID->Output_Last <= -PID->EAIS_Max && PID->Error[0] <= 0)){
        KL = 0;
    }

    A0 += PID->Ki * KL;

    /********************************PID计算***************************************/
    PID->Output = PID->Output_Last + A0 * PID->Error[0] + A1 * PID->Error[1] + A2 * PID->Error[2];

    /************************************输出限幅***********************************/
    if (PID->Output > PID->Output_Max){
        PID->Output = PID->Output_Max;
    }
    else if (PID->Output < -PID->Output_Max){
        PID->Output = -PID->Output_Max;
    }

    /****************************为下次PID计算做准备********************************/
    PID->Error[2] = PID->Error[1];
    PID->Error[1] = PID->Error[0];
    PID->Output_Last = PID->Output;
    return PID->Output;
}
