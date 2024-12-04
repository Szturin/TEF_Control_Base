#include "shoot_task.h"

/*ȫ�ֱ��� warning ������*/
extern motor_measure_t RIGHTMOTORDATA;    // ��Ħ����
extern motor_measure_t LEFTMOTORDATA;   // ��Ħ����
extern motor_measure_t DATATRIGGER;     // �����̵��
extern CANSend_TypeDef shoot_motor_data;

/*ȫ�ֱ��� warning ������*/
CANSend_TypeDef shoot_motor_data;

static SHO_Motor_t left_friction;
static SHO_Motor_t right_friction;
static SHO_Motor_t trigger;

static int16_t give_flag;//��ȡ��ʼֵ�ж�
static float shoot_current_angle;//�Ƕ�ת����˲ʱ��
static int16_t stop_time = 0;//����ʱ��
/*ȫ�ֱ��� warning ������*/
int16_t shoot_mode = 0;//����ģʽ
static int16_t stop_continue_time = 0;
static uint8_t shoot_flag = 0;//Ħ����ģʽ�ж�


int16_t ka_time = 0;
int16_t ka_flag = 0;
int16_t dong_flag = 1;


trigger_change_calc_info_t now;//��ǰ����ֵ��Ȧ��
trigger_change_calc_info_t basic;//תǰ����ֵ��Ȧ��
trigger_change_calc_info_t target;//Ŀ�����ֵ��Ȧ��

#define friction_pid \
{\
    .Kp = 3000.0f, \
    .Ki = 0.5f,\
    .Kd = 0.0f,\
    .Output_Max = 16384.0f,\
    .DeadZone = 0.01f,\
    .EIS_Max = 50000.0f,\
    .EAIS_Max = 50000.0f,\
    .Integral_Max = 2000.0f,\
    .Error = {0.0f,0.0f,0.0f},\
    .Integral = 0.0f,\
    .Output = 0.0f,\
    .Output_Last = 0.0f,\
    .Calc = &Position_PID,\
    .RST = &PID_Reset,\
}

#define trigger_pid \
{\
    .Kp = 3000,\
    .Ki = 0.05f/*0.02*/,\
    .Kd = 5.0f/*1.8*/,\
    .Output_Max = 3000,\
    .DeadZone = 0.01f,\
    .EIS_Max = 50000,\
    .EAIS_Max = 50000,\
    .Integral_Max = 1500,\
    .Error = {0,0,0},\
    .Integral = 0,\
    .Output = 0,\
    .Output_Last = 0,\
    .Calc = &Position_PID,\
    .RST  = &PID_Reset,\
}

PID_TypeDef friction_motor_init = friction_pid;
PID_TypeDef trigger_motor_init = trigger_pid;


void shoottask()
{
    // shoot_init();
    shoot_data_refresh();
    shoot_calc_task();
    shoot_data_send();
    shoot_remote_calc();
    //osDelayUntil(&shoot_task_time, 10);
}

void shoot_init(void)
{
    /*
        now.circle=0;
      now.ecd_sum=0;*/
}

void shoot_remote_calc(void)
{
    switch (rc.remote.sw1)//���Ͽ��ء���>��������ģʽ
    {
    /*
        case 1://���Ͽ���gps����>������ͨ
            shoot_mode = STOP;
            break;
        case 3://���Ͽ���att1����>����ͨ��
            if (shoot_mode != BACK)
            {
                shoot_mode = SPEED_1;
            }
            break;
        case 2://���Ͽ���att2����>�涯
            if (shoot_mode != BACK)
            {
                shoot_mode = SPEED_2;
            }
            break;
    */
        case 1://���Ͽ���gps����> ֹͣ���
            remote_mode = 3;//��̨����
            shoot_mode = STOP;
            break;
        case 3://���Ͽ���att1����> �������
            remote_mode = 3;//��̨����
            if (shoot_mode != BACK)
            {

                shoot_mode = SPEED_1;
            }
            break;
        case 2://���Ͽ���att2����> �Զ���׼
            remote_mode = 3;//**��̨���鸨������**
            if (shoot_mode != BACK)
            {

                shoot_mode = SPEED_2;
            }
            break;
    }

    if (shoot_mode == STOP)
    {
        stop_feed();
    }
    else if (shoot_mode != STOP)
    {
        if (shoot_mode == BACK)
        {
            back_feed();
        }
        else if (shoot_mode != BACK)
        {
            if (shoot_motor_data.targrt_3 < -100 && trigger.speed > -0.0005 && trigger.speed < 0.0005)
            {
                stop_time++;
            }

            if (stop_time == 1)
            {
                shoot_mode = BACK;
            }

            if (shoot_mode == SPEED_1)
            {
                speed1_feed();
            }
            if (shoot_mode == SPEED_2)
            {
                speed2_feed();
            }
        }
    }


}

void speed1_feed(void)//����ι��
{
    left_friction.target_speed = 3.5;
    right_friction.target_speed = -3.5;
    trigger.target_speed = -0.3;//ת���ٶ�

    shoot_motor_data.targrt_1 = left_friction.send_data;
    shoot_motor_data.targrt_2 = right_friction.send_data;
    shoot_motor_data.targrt_3 = trigger.send_data;
}

void speed2_feed(void)//����ι��
{
    //Ŀ���ٶ�

    /*
    left_friction.target_speed = 3.5;
    right_friction.target_speed = -3.5;
    trigger.target_speed = -0.35;//�����ٶȣ�ת��
    */

    left_friction.target_speed=(float)(abs(rc.remote.wheel))/660.0f*3.5f;//��һ��
    right_friction.target_speed=-(float)(abs(rc.remote.wheel))/660.0f*3.5f;//��һ��
    trigger.target_speed = (float)(abs(rc.remote.wheel))/660.0f*0.35f;//��һ��

    //�����ٶ�
    shoot_motor_data.targrt_1 = left_friction.send_data;
    shoot_motor_data.targrt_2 = right_friction.send_data;
    shoot_motor_data.targrt_3 = trigger.send_data;
}

void back_feed(void)//�������������̵����ת��
{
    if (shoot_flag == 0)
    {
        shoot_motor_data.targrt_3 = 700;
        stop_continue_time++;
    }

    if (stop_continue_time == 10)
    {
        shoot_flag = 1;
    }

    if (shoot_flag == 1)
    {
        shoot_flag = 0;
        stop_time = 0;
        stop_continue_time = 0;
        shoot_mode = 2;
    }
}

void stop_feed(void)//ֹͣι��
{
    left_friction.target_speed = 0;
    right_friction.target_speed = 0;
    trigger.target_speed = 0;
    shoot_motor_data.targrt_1 = 0;
    shoot_motor_data.targrt_2 = 0;
    shoot_motor_data.targrt_3 = 0;
    shoot_mode = 0;
    stop_time = 0;
}


void shoot_data_refresh(void)
{
    left_friction.speed_rpm = LEFTMOTORDATA.speed_rpm;
    right_friction.speed_rpm = RIGHTMOTORDATA.speed_rpm;
    trigger.speed_rpm = DATATRIGGER.speed_rpm;
}

void shoot_calc_task(void)
{
    left_friction.speed = left_friction.speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
    right_friction.speed = right_friction.speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
    trigger.speed = trigger.speed_rpm * M2006_MOTOR_RPM_TO_VECTOR;


    left_friction.send_data = (short) friction_motor_init.Calc(&friction_motor_init, left_friction.speed, left_friction.target_speed);
    right_friction.send_data = (short) friction_motor_init.Calc(&friction_motor_init, right_friction.speed, right_friction.target_speed);
    trigger.send_data = (short) trigger_motor_init.Calc(&trigger_motor_init, trigger.speed, trigger.target_speed);

    /*
     left_friction.target_speed=1.5;
     right_friction.target_speed=-1.5;
     trigger.target_speed=-0.15;

     shoot_motor_data.targrt_1=left_friction.send_data;
     shoot_motor_data.targrt_2=right_friction.send_data;
     shoot_motor_data.targrt_3=trigger.send_data;
     */
}

void shoot_data_send(void)//�������ݷ���
{
    CAN_cmd_shoot(&shoot_motor_data);
}
