#include "imu/JY901USE.h"

/*ȫ�ֱ��� warning ������*/
extern UART_HandleTypeDef huart8;
/*ȫ�ֱ��� warning ������*/
User_USART JY901_data;

static struct SAcc stcAcc;//�Ǽ��ٶ�

static struct SGyro stcGyro;//���ٶ�

static struct SAngle stcAngle;//�Ƕ�


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
    }    //����У��

    uint8_t sum;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (JY901_data.RxBuffer[i * 11] != JY901_data.frame_head)//֡ͷУ��
        {
            continue;//֡ͷ������������
        }

        switch (JY901_data.RxBuffer[i * 11 + 1]) //�жϵ�ǰ���ݶ��������ǵ��������ݣ�����֡������������
        {
            case 0x51:
                memcpy(&stcAcc, &JY901_data.RxBuffer[2 + i * 11], 8);

                for (uint8_t j = 0; j < 3; j++)
                {
                    JY901_data.acc.a[j] = (float) stcAcc.a[j] / 32768 * 16;                     //���ٶ�
                }

                break;
            case 0x52:
                memcpy(&stcGyro, &JY901_data.RxBuffer[2 + i * 11], 8);

                for (uint8_t j = 0; j < 3; j++){
                    JY901_data.w.w[j] = (float) stcGyro.w[j] / 32768 * 2000;                    //���ٶ�
                }

                break;
            case 0x53:
                memcpy(&stcAngle, &JY901_data.RxBuffer[2 + i * 11], 8);

                for (uint8_t j = 0; j < 3; j++)
                {
                    JY901_data.angle.angle[j] = (float) stcAngle.Angle[j] / 32768 * 180;        //�Ƕ�
                }

                break;
        }

    }
}
*/

void JY901_Process()
{
    // У����յ������ݳ����Ƿ�Ϸ�
    if (JY901_data.Rx_len < RXBUFFER_LEN )
    {
        return;
    }

    // ����ÿһ֡����
    for (uint8_t i = 0; i < 3; i++)
    {

        // ֡ͷ��У���
        if ((JY901_data.RxBuffer[i * 11] != JY901_data.frame_head) || (!CheckDataChecksum(i)))
        {
            // ���֡ͷ��У��Ͳ�ƥ�䣬������֡Ϊ�Ƿ�
            continue;
        }

        // ��ȡ��ǰ����֡������
        uint8_t dataType = JY901_data.RxBuffer[i * 11 + 1];

        // �����������ʹ���ͬ������֡
        switch (dataType)
        {
            case 0x51: // ���ٶ�����
                ProcessAccData(i);
                break;
            case 0x52: // ����������
                ProcessGyroData(i);
                break;
            case 0x53: // �Ƕ�����
                ProcessAngleData(i);
                break;
            default:
                // δ֪�������ʹ���
                break;
        }
    }
}

// У�����ݵ�У���
uint8_t CheckDataChecksum(uint8_t index)
{
    uint8_t checksum = 0;
    uint8_t length = 11;

    // ��������֡��У���
    for (uint8_t i = 0; i < length - 1; i++)
    {
        checksum += JY901_data.RxBuffer[i + index * length];
    }

    // У���У��
    return (checksum == (uint8_t)(JY901_data.RxBuffer[index * length + 10]));
}

// ������ٶ�����
void ProcessAccData(uint8_t index)
{
    // �������ٶ����ݵ��ṹ��
    memcpy(&stcAcc, &JY901_data.RxBuffer[2 + index * 11], sizeof(stcAcc));

    // �����ٶ�ֵת��Ϊʵ��ֵ���洢
    for (uint8_t j = 0; j < 3; j++)
    {
        JY901_data.acc.a[j] = (float)stcAcc.a[j] / 32768.0f * 16;  // ���ٶ�ת��
    }
}

// ��������������
void ProcessGyroData(uint8_t index)
{
    // �������������ݵ��ṹ��
    memcpy(&stcGyro, &JY901_data.RxBuffer[2 + index * 11], sizeof(stcGyro));

    // �����ٶ�ֵת��Ϊʵ��ֵ���洢
    for (uint8_t j = 0; j < 3; j++)
    {
        JY901_data.w.w[j] = (float)stcGyro.w[j] / 32768.0f * 2000;  // ���ٶ�ת��
    }
}

// ����Ƕ�����
void ProcessAngleData(uint8_t index)
{
    // �����Ƕ����ݵ��ṹ��
    memcpy(&stcAngle, &JY901_data.RxBuffer[2 + index * 11], sizeof(stcAngle));

    // ���Ƕ�ֵת��Ϊʵ��ֵ���洢
    for (uint8_t j = 0; j < 3; j++)
    {
        JY901_data.angle.angle[j] = (float)stcAngle.Angle[j] / 32768.0f * 180;  // �Ƕ�ת��
    }
}


float error_angle_calc(float target_angle, float now_angle)//��������С��Ǽ���
{
    float  error_angle = target_angle - now_angle;

    if (error_angle > 180)
    {
        // ��ֹС��ת��180��ʱһֱ��ת������
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