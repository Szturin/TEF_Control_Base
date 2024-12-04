#include "imu/JY901USE.h"

/*全局变量 warning ！！！*/
extern UART_HandleTypeDef huart8;
/*全局变量 warning ！！！*/
User_USART JY901_data;

static struct SAcc stcAcc;

static struct SGyro stcGyro;

static struct SAngle stcAngle;


void User_USART_Init(User_USART *Data)
{
    for (uint16_t i = 0; i < RXBUFFER_LEN; i++)
    {
        Data->RxBuffer[i] = 0;
    }
    Data->frame_head = 0x55;
    Data->Rx_flag = 0;
    Data->Rx_len = 0;
}

void JY901_Process()
{

    if (JY901_data.Rx_len < RXBUFFER_LEN)
    {
        return;
    }    //长度校验
    //my_printf(&huart2,"here\r\n");

    for (uint8_t i = 0; i < 4; i++)
    {

        if (JY901_data.RxBuffer[i * 11] != JY901_data.frame_head)
        {
            continue;
        }
        //my_printf(&huart2,"head:%d\r\n",JY901_data.RxBuffer[i * 11 +1]);
        switch (JY901_data.RxBuffer[i * 11 + 1])
        {

        case 0x51:

            memcpy(&stcAcc, &JY901_data.RxBuffer[2 + i * 11], 8);
            for (uint8_t j = 0; j < 3; j++)
            {

                JY901_data.acc.a[j] = (float) stcAcc.a[j] / 32768 * 16;
            }                                    //加速度
            break;
        case 0x52:
            memcpy(&stcGyro, &JY901_data.RxBuffer[2 + i * 11], 8);
            for (uint8_t j = 0; j < 3; j++)
                JY901_data.w.w[j] = (float) stcGyro.w[j] / 32768 * 2000;                                //角速度
            break;
        case 0x53:
            memcpy(&stcAngle, &JY901_data.RxBuffer[2 + i * 11], 8);
            for (uint8_t j = 0; j < 3; j++)
            {
                JY901_data.angle.angle[j] = (float) stcAngle.Angle[j] / 32768 * 180;        //角度
                //                                                                                           my_printf(&huart2,"head:%f\r\n",JY901_data.angle.angle[j] );
            }

            break;
        }

    }
}

float error_angle_calc(float target_angle, float now_angle)//陀螺仪最小差角计算
{
    float error_angle;
    error_angle = target_angle - now_angle;

    if (error_angle >= 180)
    {
        // 防止小车转到180度时一直旋转的问题
        error_angle = error_angle - 360;
    }
    if (error_angle < -180)
    {
        error_angle = error_angle + 360;
    }
    return error_angle;
}

float ffabs(float number)
{
    if (number >= 0)
    {
        number = number;
    }
    else if (number < 0)
    {
        number = -number;
    }
    return number;
}