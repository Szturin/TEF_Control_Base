#include "chassis_task.h"

/***************************************PID����*********************************************/
#define chassic_pid \
{\
	.Kp = 4000.0f, \
	.Ki = 10.0f,\
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

#define chassic_follow_pid \
{\
	.Kp = 17.0f,   \
	.Ki = 0.02f,  \
	.Kd = 375.0f,      \
	.Output_Max = 10.0f,\
	.DeadZone = 0.00f,\
	.EIS_Max = 50000.0f,\
	.EAIS_Max = 50000.0f,\
	.Integral_Max = 1.0f,\
	.Error = {0.0f,0.0f,0.0f},\
	.Integral = 0.0f,\
	.Output = 0.0f,\
	.Output_Last = 0.0f,\
	.Calc = &Position_PID,\
	.RST  = &PID_Reset,\
}



float sin_yaw=0,cos_yaw=0;

PID_TypeDef chassis_motor1_init = chassic_pid; //���̵��pid
PID_TypeDef chassis_motor2_init = chassic_pid;
PID_TypeDef chassis_motor3_init = chassic_pid;
PID_TypeDef chassis_motor4_init = chassic_pid;

PID_TypeDef chassis_follow_init = chassic_follow_pid;


extern PID_TypeDef   gimbal_yaw_init;

chassis_ctrl_info_t chassis_control;
chassis_ctrl_info_t key_chassic;
chassis_motor_parameter_t chassis_motor_parameter; //���̲���
extern motor_measure_t MOTORDATA1;				//����motor 1����
extern motor_measure_t MOTORDATA2;				//����motor 2����
extern motor_measure_t MOTORDATA3;				//����motor 3����
extern motor_measure_t MOTORDATA4;				//����motor 4����
extern CANSend_TypeDef chassis_motor_data; //���̷�����������
extern motor_measure_t DATAGIMBALYAW;			//��̨yaw

fp32 vx_set;
fp32 vy_set;
fp32 wz_set;
fp32 last_set;
fp32 wz_target_set;

CANSend_TypeDef chassis_motor_data;

uint8_t keyboard_flag=0;//����ͨ���ļ����ж�
int16_t move_mode;//����ͨ���µ�С��ģʽ
int16_t coolspeed=0;

void chassistask()
{
    //my_printf(&huart2,"ok");
    chassis_data_calc();
    controlmode1_solution();
    PID_Adjust();
    chassis_data_send();

}

/***************************************���̿��Ʋ���*********************************************/
void controlmode1_solution(void)//ģʽѡ��
{
    switch(rc.remote.sw2)//���Ͽ��ء���>��������ģʽ
    {
    case 1://���Ͽ���gps����>�Ե���Ϊ�ᣬ��ͨģʽ
        chassis_axis_control();
        keyboard_flag=NOT_READY;
        break;
    case 3://���Ͽ���att1����>��̨���棬���涯ģʽ
        gimbal_follow_control();
        //gimbal_axis_control();
        //keyboard_control();
        keyboard_flag=NOT_READY;
        break;
    case 2://���Ͽ���att2����>����̨Ϊ�ᣬС����ģʽ
        gimbal_axis_control();
        //gimbal_follow_control();
        keyboard_flag=NOT_READY;
        break;
    }
    mackenaham_calc(chassis_control.vx_set,chassis_control.vy_set,chassis_control.wz_set);
}

void keyboard_control(void)//����ͨ����������
{
    /*�ҳ������Ǽ���ģʽ�������ǻ���ǰ�����ҵĿ���
      ���������߼���
      w����ǰ����s�������ˣ�a��������d��������
      shift+����������������٣�ctrl+�������������*/

    if(rc.key.z==1&&move_mode!=BASIC)
    {
        move_mode=BASIC;
    }
    if(rc.key.g==1&&move_mode!=GLOBAL)
    {
        move_mode=GLOBAL;
    }
    if(rc.key.f==1&&move_mode!=FOLLOW)
    {
        move_mode=FOLLOW;
    }

    //***********************//
    //Power_limit();  //��������
    //***********************//
}

void gimbal_axis_control(void)//����̨Ϊ��Ŀ���ģʽ����С����
{
    vx_set=(float)rc.remote.ch3*CHASSIS_VX_RC_SEN;
    vy_set=(float)rc.remote.ch4*CHASSIS_VY_RC_SEN;
    //chassis_control.wz_set=3;
    chassis_control.wz_set=5;
    sin_yaw = arm_sin_f32(chassis_motor_parameter.chassis_relative_angle);  //sin��
    cos_yaw = arm_cos_f32(chassis_motor_parameter.chassis_relative_angle);  //cos���߼����жϣ���ƽ����sin��cos���죬�ܹ�ʵ��С����
    chassis_control.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
    chassis_control.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
}

void chassis_axis_control(void)//�Ե���Ϊ��Ŀ���ģʽ������ͨģʽ
{
    chassis_control.wz_set=0;
    chassis_control.vx_set=(float)rc.remote.ch3*CHASSIS_VX_RC_SEN*2.0;
    chassis_control.vy_set=(float)rc.remote.ch4*CHASSIS_VY_RC_SEN*2.0;
}


void gimbal_follow_control(void)//�涯ģʽ
{
    chassis_control.vx_set=(float)rc.remote.ch3*CHASSIS_VX_RC_SEN*2.0;
    chassis_control.vy_set=(float)rc.remote.ch4*CHASSIS_VY_RC_SEN*2.0;

    wz_set = (-chassis_follow_init.Calc(&chassis_follow_init,chassis_motor_parameter.chassis_relative_angle,-0.260));//1.0f

    last_set = wz_set;

    wz_target_set = wz_set * 0.83 + last_set * 0.17;//�ͺ�����ȶ�

    chassis_control.wz_set=wz_target_set;
}


/***************************************�����˶�����********************************************/
void chassis_data_calc(void)//�������ݽ���
{
    //����������������ٶȵ�λ��rpmת��Ϊm/s
    //����ٶ��ǵ����ǰ���ٶ�
    MOTORDATA1.speed = MOTORDATA1.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //��rpmת��Ϊm/s
    MOTORDATA2.speed = MOTORDATA2.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //��rpmת��Ϊm/s
    MOTORDATA3.speed = MOTORDATA3.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //��rpmת��Ϊm/s
    MOTORDATA4.speed = MOTORDATA4.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //��rpmת��Ϊm/s

    //�������˶�ѧ��ʽ���õ�����ڵ��ٶȣ�������̵�ǰ�����������ϵ��ٶ�
    //���ã�������
    chassis_motor_parameter.vx = (-MOTORDATA1.speed + MOTORDATA2.speed + MOTORDATA3.speed - MOTORDATA4.speed)*CHASSIS_VX_SPEED;
    chassis_motor_parameter.vy = (-MOTORDATA1.speed - MOTORDATA2.speed + MOTORDATA3.speed + MOTORDATA4.speed)*CHASSIS_VX_SPEED;
    chassis_motor_parameter.wz = (-MOTORDATA1.speed - MOTORDATA2.speed - MOTORDATA3.speed - MOTORDATA4.speed)*CHASSIS_VX_SPEED/CHASSISMOTOR_TO_CENTER; //�������ٶȵ�w=v/r rΪCHASSISMOTOR_TO_CENTER
}

//@param ������������������ٶ�
//@brief ��������������������ٶȷֽ�Ϊ�����ĸ�����������ٶ�
void mackenaham_calc(fp32 vx_set,fp32 vy_set,fp32 wz_set)//�����˶�ѧ����
{
    chassis_control.wheel_speed[0] = -vx_set - vy_set +(CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
    chassis_control.wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
    chassis_control.wheel_speed[2] = vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
    chassis_control.wheel_speed[3] = -vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
}

//@param2��MOTORDATA1.speed ���������������ٶȣ�����ǰ�ٶ�
//@param3: chassis_control.wheel_speed[0] ����������ٶȣ������溯�������������Ŀ���ٶ�
//�ٶȻ�
void PID_Adjust(void)//PID����
{
    chassis_motor_data.targrt_1 = (short)chassis_motor1_init.Calc(&chassis_motor1_init,MOTORDATA1.speed,chassis_control.wheel_speed[0]);
    chassis_motor_data.targrt_2 = (short)chassis_motor2_init.Calc(&chassis_motor2_init,MOTORDATA2.speed,chassis_control.wheel_speed[1]);
    chassis_motor_data.targrt_3 = (short)chassis_motor3_init.Calc(&chassis_motor3_init,MOTORDATA3.speed,chassis_control.wheel_speed[2]);
    chassis_motor_data.targrt_4 = (short)chassis_motor4_init.Calc(&chassis_motor4_init,MOTORDATA4.speed,chassis_control.wheel_speed[3]);
}

void chassis_data_send(void)//�������ݷ���
{
    CAN_cmd_chassis(&chassis_motor_data);  //�˴���ң����ֵ�������ʹ��ת����������ң����ֵʱ��ɽ���ע�͵����������ʱ�����ܷ�����
}
