#include "imu/JY901USE.h"

/*全局变量 warning ！！！*/
extern UART_HandleTypeDef huart8;
/*全局变量 warning ！！！*/
User_USART JY901_data;

static struct SAcc stcAcc;//角加速度

static struct SGyro stcGyro;//角速度

static struct SAngle stcAngle;//角度


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

/*
void JY901_Process()
{
    if (JY901_data.Rx_len < RXBUFFER_LEN)
    {
        return;
    }    //长度校验

    uint8_t sum;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (JY901_data.RxBuffer[i * 11] != JY901_data.frame_head)//帧头校验
        {
            continue;//帧头不符合则跳过
        }

        switch (JY901_data.RxBuffer[i * 11 + 1]) //判断当前数据段是陀螺仪的哪种数据，数据帧包含三个部分
        {
            case 0x51:
                memcpy(&stcAcc, &JY901_data.RxBuffer[2 + i * 11], 8);

                for (uint8_t j = 0; j < 3; j++)
                {
                    JY901_data.acc.a[j] = (float) stcAcc.a[j] / 32768 * 16;                     //加速度
                }

                break;
            case 0x52:
                memcpy(&stcGyro, &JY901_data.RxBuffer[2 + i * 11], 8);

                for (uint8_t j = 0; j < 3; j++){
                    JY901_data.w.w[j] = (float) stcGyro.w[j] / 32768 * 2000;                    //角速度
                }

                break;
            case 0x53:
                memcpy(&stcAngle, &JY901_data.RxBuffer[2 + i * 11], 8);

                for (uint8_t j = 0; j < 3; j++)
                {
                    JY901_data.angle.angle[j] = (float) stcAngle.Angle[j] / 32768 * 180;        //角度
                }

                break;
        }

    }
}
*/

void JY901_Process()
{
    // 校验接收到的数据长度是否合法
    if (JY901_data.Rx_len < RXBUFFER_LEN )
    {
        return;
    }

    // 处理每一帧数据
    for (uint8_t i = 0; i < 3; i++)
    {

        // 帧头和校验和
        if ((JY901_data.RxBuffer[i * 11] != JY901_data.frame_head) || (!CheckDataChecksum(i)))
        {
            // 如果帧头和校验和不匹配，此数据帧为非法
            continue;
        }

        // 获取当前数据帧的类型
        uint8_t dataType = JY901_data.RxBuffer[i * 11 + 1];

        // 根据数据类型处理不同的数据帧
        switch (dataType)
        {
            case 0x51: // 加速度数据
                ProcessAccData(i);
                break;
            case 0x52: // 陀螺仪数据
                ProcessGyroData(i);
                break;
            case 0x53: // 角度数据
                ProcessAngleData(i);
                break;
            default:
                // 未知数据类型处理
                break;
        }
    }
}

// 校验数据的校验和
uint8_t CheckDataChecksum(uint8_t index)
{
    uint8_t checksum = 0;
    uint8_t length = 11;

    // 计算数据帧的校验和
    for (uint8_t i = 0; i < length - 1; i++)
    {
        checksum += JY901_data.RxBuffer[i + index * length];
    }

    // 校验和校验
    return (checksum == (uint8_t)(JY901_data.RxBuffer[index * length + 10]));
}

// 处理加速度数据
void ProcessAccData(uint8_t index)
{
    // 拷贝加速度数据到结构体
    memcpy(&stcAcc, &JY901_data.RxBuffer[2 + index * 11], sizeof(stcAcc));

    // 将加速度值转化为实际值并存储
    for (uint8_t j = 0; j < 3; j++)
    {
        JY901_data.acc.a[j] = (float)stcAcc.a[j] / 32768.0f * 16;  // 加速度转换
    }
}

// 处理陀螺仪数据
void ProcessGyroData(uint8_t index)
{
    // 拷贝陀螺仪数据到结构体
    memcpy(&stcGyro, &JY901_data.RxBuffer[2 + index * 11], sizeof(stcGyro));

    // 将角速度值转化为实际值并存储
    for (uint8_t j = 0; j < 3; j++)
    {
        JY901_data.w.w[j] = (float)stcGyro.w[j] / 32768.0f * 2000;  // 角速度转换
    }
}

// 处理角度数据
void ProcessAngleData(uint8_t index)
{
    // 拷贝角度数据到结构体
    memcpy(&stcAngle, &JY901_data.RxBuffer[2 + index * 11], sizeof(stcAngle));

    // 将角度值转化为实际值并存储
    for (uint8_t j = 0; j < 3; j++)
    {
        JY901_data.angle.angle[j] = (float)stcAngle.Angle[j] / 32768.0f * 180;  // 角度转换
    }
}


float error_angle_calc(float target_angle, float now_angle)//陀螺仪最小差角计算
{
    float  error_angle = target_angle - now_angle;

    if (error_angle > 180)
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