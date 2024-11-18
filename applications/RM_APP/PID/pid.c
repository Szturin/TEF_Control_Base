#include "pid.h"

void PID_Reset(PID_TypeDef *PID)
{
    PID->Error[0] = PID->Error[1] = PID->Error[2] = 0;
    PID->Integral = 0;

    PID->Output = 0;
    PID->Output_Last = 0;
}

//λ��ʽPID
float Position_PID(PID_TypeDef *PID, float CurrentValue, float TargetValue)
{
    PID->Error[0] = TargetValue - CurrentValue;

    /************************����ƫ���PID�������е���***************************/
    //�������㣬��Ϊ0��ȡ��Ч��
    if ((PID->Error[0] <= PID->DeadZone) && (PID->Error[0] >= -PID->DeadZone))
    {
        PID->Error[0] = 0.0f;
    }
    else
    {
        if ((PID->EIS_Max == 0) || ((PID->Error[0] <= PID->EIS_Max) && (PID->Error[0] >= -PID->EIS_Max)))
        {
            //�����޷�����Ϊһ���㹻�������ȡ��Ч��
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

    /********************************PID����***************************************/
    PID->Output = PID->Kp * PID->Error[0] + PID->Integral + PID->Kd * (PID->Error[0] - PID->Error[1]);

    /****************************Ϊ�´�PID������׼��********************************/
    PID->Error[1] = PID->Error[0];

    /************************************����޷�***********************************/
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

//����ʽPID
float Incremental_PID(PID_TypeDef *PID, float CurrentValue, float TargetValue)
{
    float A0 = PID->Kp + PID->Kd;
    float A1 = -PID->Kp - 2 * PID->Kd;
    float A2 = PID->Kd;
    char KL = 1;

    PID->Error[0] = TargetValue - CurrentValue;

    /************************����ƫ���PID�������е���***************************/
    //�������㣬��Ϊ0��ȡ��Ч��
    if ((PID->Error[0] <= PID->DeadZone) && (PID->Error[0] >= -PID->DeadZone)){
        PID->Error[0] = 0.0;
    }

    //���ַ��룬��Ϊһ���㹻�������ȡ��Ч�� *
    if ((PID->Error[0] <= PID->EIS_Max) && (PID->Error[0] >= -PID->EIS_Max)){
        KL = 1;
    }
    else{
        KL = 0;
    }
    //�����ֱ��ͣ���Ϊһ���㹻�������ȡ��Ч��
    if ((PID->Output_Last >= PID->EAIS_Max && PID->Error[0] >= 0) || (PID->Output_Last <= -PID->EAIS_Max && PID->Error[0] <= 0)){
        KL = 0;
    }

    A0 += PID->Ki * KL;

    /********************************PID����***************************************/
    PID->Output = PID->Output_Last + A0 * PID->Error[0] + A1 * PID->Error[1] + A2 * PID->Error[2];

    /************************************����޷�***********************************/
    if (PID->Output > PID->Output_Max){
        PID->Output = PID->Output_Max;
    }
    else if (PID->Output < -PID->Output_Max){
        PID->Output = -PID->Output_Max;
    }

    /****************************Ϊ�´�PID������׼��********************************/
    PID->Error[2] = PID->Error[1];
    PID->Error[1] = PID->Error[0];
    PID->Output_Last = PID->Output;
    return PID->Output;
}
