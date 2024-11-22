#include "chassis_task.h"

/***************************************PID调参*********************************************/
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

PID_TypeDef chassis_motor1_init = chassic_pid; //底盘电机pid
PID_TypeDef chassis_motor2_init = chassic_pid;
PID_TypeDef chassis_motor3_init = chassic_pid;
PID_TypeDef chassis_motor4_init = chassic_pid;

PID_TypeDef chassis_follow_init = chassic_follow_pid;


extern PID_TypeDef   gimbal_yaw_init;

chassis_ctrl_info_t chassis_control;
chassis_ctrl_info_t key_chassic;
chassis_motor_parameter_t chassis_motor_parameter; //底盘参数
extern motor_measure_t MOTORDATA1;				//底盘motor 1数据
extern motor_measure_t MOTORDATA2;				//底盘motor 2数据
extern motor_measure_t MOTORDATA3;				//底盘motor 3数据
extern motor_measure_t MOTORDATA4;				//底盘motor 4数据
extern CANSend_TypeDef chassis_motor_data; //底盘发送数据轴电机
extern motor_measure_t DATAGIMBALYAW;			//云台yaw

fp32 vx_set;
fp32 vy_set;
fp32 wz_set;
fp32 last_set;
fp32 wz_target_set;

CANSend_TypeDef chassis_motor_data;

uint8_t keyboard_flag=0;//键盘通道的激活判断
int16_t move_mode;//键盘通道下的小车模式
int16_t coolspeed=0;

void chassistask()
{
    //my_printf(&huart2,"ok");
    chassis_data_calc();
    controlmode1_solution();
    PID_Adjust();
    chassis_data_send();

}

/***************************************底盘控制部分*********************************************/
void controlmode1_solution(void)//模式选择
{
    switch(rc.remote.sw2)//右上开关——>控制三种模式
    {
    case 1://右上开关gps——>以底盘为轴，普通模式
        chassis_axis_control();
        keyboard_flag=NOT_READY;
        break;
    case 3://右上开关att1——>云台跟随，即随动模式
        gimbal_follow_control();
        //gimbal_axis_control();
        //keyboard_control();
        keyboard_flag=NOT_READY;
        break;
    case 2://右上开关att2——>以云台为轴，小陀螺模式
        gimbal_axis_control();
        //gimbal_follow_control();
        keyboard_flag=NOT_READY;
        break;
    }
    mackenaham_calc(chassis_control.vx_set,chassis_control.vy_set,chassis_control.wz_set);
}

void keyboard_control(void)//键盘通道，待完善
{
    /*我超，这是键盘模式！以下是基础前后左右的控制
      基础控制逻辑：
      w——前进，s——后退，a——向左，d——向右
      shift+方向键——静步减速，ctrl+方向键——加速*/

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
    //Power_limit();  //功率限制
    //***********************//
}

void gimbal_axis_control(void)//以云台为轴的控制模式，即小陀螺
{
    vx_set=(float)rc.remote.ch3*CHASSIS_VX_RC_SEN;
    vy_set=(float)rc.remote.ch4*CHASSIS_VY_RC_SEN;
    //chassis_control.wz_set=3;
    chassis_control.wz_set=5;
    sin_yaw = arm_sin_f32(chassis_motor_parameter.chassis_relative_angle);  //sinθ
    cos_yaw = arm_cos_f32(chassis_motor_parameter.chassis_relative_angle);  //cosθ逻辑门判断，比平常的sin和cos更快，能够实现小陀螺
    chassis_control.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
    chassis_control.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
}

void chassis_axis_control(void)//以底盘为轴的控制模式，即普通模式
{
    chassis_control.wz_set=0;
    chassis_control.vx_set=(float)rc.remote.ch3*CHASSIS_VX_RC_SEN*2.0;
    chassis_control.vy_set=(float)rc.remote.ch4*CHASSIS_VY_RC_SEN*2.0;
}


void gimbal_follow_control(void)//随动模式
{
    chassis_control.vx_set=(float)rc.remote.ch3*CHASSIS_VX_RC_SEN*2.0;
    chassis_control.vy_set=(float)rc.remote.ch4*CHASSIS_VY_RC_SEN*2.0;

    wz_set = (-chassis_follow_init.Calc(&chassis_follow_init,chassis_motor_parameter.chassis_relative_angle,-0.260));//1.0f

    last_set = wz_set;

    wz_target_set = wz_set * 0.83 + last_set * 0.17;//滞后更加稳定

    chassis_control.wz_set=wz_target_set;
}


/***************************************底盘运动部分********************************************/
void chassis_data_calc(void)//底盘数据解算
{
    //将电机反馈过来的速度单位由rpm转换为m/s
    //这个速度是电机当前的速度
    MOTORDATA1.speed = MOTORDATA1.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //将rpm转换为m/s
    MOTORDATA2.speed = MOTORDATA2.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //将rpm转换为m/s
    MOTORDATA3.speed = MOTORDATA3.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //将rpm转换为m/s
    MOTORDATA4.speed = MOTORDATA4.speed_rpm *M3508_MOTOR_RPM_TO_VECTOR; //将rpm转换为m/s

    //利用正运动学公式，用电机现在的速度，求出底盘当前在三个方向上的速度
    //作用：？？？
    chassis_motor_parameter.vx = (-MOTORDATA1.speed + MOTORDATA2.speed + MOTORDATA3.speed - MOTORDATA4.speed)*CHASSIS_VX_SPEED;
    chassis_motor_parameter.vy = (-MOTORDATA1.speed - MOTORDATA2.speed + MOTORDATA3.speed + MOTORDATA4.speed)*CHASSIS_VX_SPEED;
    chassis_motor_parameter.wz = (-MOTORDATA1.speed - MOTORDATA2.speed - MOTORDATA3.speed - MOTORDATA4.speed)*CHASSIS_VX_SPEED/CHASSISMOTOR_TO_CENTER; //自旋的速度的w=v/r r为CHASSISMOTOR_TO_CENTER
}

//@param 底盘三个方向的期望速度
//@brief 将底盘三个方向的期望速度分解为底盘四个电机的期望速度
void mackenaham_calc(fp32 vx_set,fp32 vy_set,fp32 wz_set)//麦轮运动学解算
{
    chassis_control.wheel_speed[0] = -vx_set - vy_set +(CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
    chassis_control.wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
    chassis_control.wheel_speed[2] = vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
    chassis_control.wheel_speed[3] = -vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) *CHASSISMOTOR_TO_CENTER* wz_set;
}

//@param2：MOTORDATA1.speed 电条反馈过来的速度，即当前速度
//@param3: chassis_control.wheel_speed[0] 电机的期望速度，由上面函数计算而来，即目标速度
//速度环
void PID_Adjust(void)//PID调整
{
    chassis_motor_data.targrt_1 = (short)chassis_motor1_init.Calc(&chassis_motor1_init,MOTORDATA1.speed,chassis_control.wheel_speed[0]);
    chassis_motor_data.targrt_2 = (short)chassis_motor2_init.Calc(&chassis_motor2_init,MOTORDATA2.speed,chassis_control.wheel_speed[1]);
    chassis_motor_data.targrt_3 = (short)chassis_motor3_init.Calc(&chassis_motor3_init,MOTORDATA3.speed,chassis_control.wheel_speed[2]);
    chassis_motor_data.targrt_4 = (short)chassis_motor4_init.Calc(&chassis_motor4_init,MOTORDATA4.speed,chassis_control.wheel_speed[3]);
}

void chassis_data_send(void)//底盘数据发送
{
    CAN_cmd_chassis(&chassis_motor_data);  //此处将遥控器值传给电机使其转动，若测试遥控器值时候可将此注释掉，以免调试时程序跑飞伤人
}
