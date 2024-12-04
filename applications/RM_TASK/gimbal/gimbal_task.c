#include "gimbal_task.h"

/***************************************PID调参*********************************************/
#define pitch_angle_pid \
{\
	.Kp = 80.0f,/*190*/\
	.Ki = 0.1f, /*0.8*/\
	.Kd = 100.0f, /*1.5*/    \
	.Output_Max = 30000,/*15000*/\
	.DeadZone = 0,\
	.EIS_Max = 50000,\
	.EAIS_Max = 50000,\
	.Integral_Max = 300,\
	.Error = {0,0,0},\
	.Integral = 0,\
	.Output = 0,\
	.Output_Last = 0,\
	.Calc = &Position_PID,\
	.RST  = &PID_Reset,\
}

#define pitch_speed_pid \
{\
	.Kp = 255.0f,/*85*/\
	.Ki = 0.1f,/*0*/\
	.Kd = 4.5f,/*7.5*/\
	.Output_Max = 30000,\
	.DeadZone = 0,\
	.EIS_Max = 50000,\
	.EAIS_Max = 50000,\
	.Integral_Max = 1000,\
	.Error = {0,0,0},\
	.Integral = 0,\
	.Output = 0,\
	.Output_Last = 0,\
	.Calc = &Position_PID,\
	.RST  = &PID_Reset,\
}

//k p =155
// ki = 0.01
//kd = 1800
#define yaw_angle_pid \
{\
	.Kp = 145.0f,\
	.Ki = 0.0f, \
	.Kd = 1850.0f, \
	.Output_Max = 30000,\
	.DeadZone = 0,\
	.EIS_Max = 50000,\
	.EAIS_Max = 50000,\
	.Integral_Max = 100,\
	.Error = {0,0,0},\
	.Integral = 0,\
	.Output = 0,\
	.Output_Last = 0,\
	.Calc = &Position_PID,\
	.RST  = &PID_Reset,\
}

#define yaw_speed_pid \
{\
  .Kp = 195.0f,/*85*//*130*/\
	.Ki = 0.1f,/*0*//*0*/\
	.Kd = 7.5f,/*7.5*//*0*/\
	.Output_Max = 30000,\
	.DeadZone = 0,\
	.EIS_Max = 50000,\
	.EAIS_Max = 50000,\
	.Integral_Max = 1000,\
	.Error = {0,0,0},\
	.Integral = 0,\
	.Output = 0,\
	.Output_Last = 0,\
	.Calc = &Position_PID,\
	.RST  = &PID_Reset,\
}

static PID_TypeDef gimbal_pitch_angle_init = pitch_angle_pid;//pitch串级pid
static PID_TypeDef gimbal_pitch_speed_init = pitch_speed_pid;
static PID_TypeDef gimbal_yaw_angle_init = yaw_angle_pid;//yaw串级pid
static PID_TypeDef gimbal_yaw_speed_init = yaw_speed_pid;

/*全局变量 warning ！！！*/
extern chassis_motor_parameter_t chassis_motor_parameter;
extern CANSend_TypeDef  gimbal_motor_data;
extern motor_measure_t DATAGIMBALYAW;
extern motor_measure_t DATAGIMBALPITCH_1;
extern motor_measure_t DATAGIMBALPITCH_2;


static Gim_Motor_t yaw;
static Gim_Motor_t pit_1;
static Gim_Motor_t pit_2;

/*全局变量 warning ！！！*/
CANSend_TypeDef  gimbal_motor_data;

/*全局变量 warning ！！！*/
extern User_USART JY901_data; //维特陀螺仪数据

uint8_t remote_mode= 2;
static int16_t imu_flag=0;
static int16_t yaw_flag=0;

static short yaw_control_target_speed;//陀螺仪自稳目标速度
static uint8_t coditional_constant=0;//陀螺仪初始角获取判断常数
static int16_t space_initial_angle=0;//陀螺仪初始角
static int16_t deflective_angle;//陀螺仪差角

static uint8_t initial_angle_flag=0;//初始机械角获取判断常数
static int16_t initial_angle=0;//初始机械角
static int16_t initial_angle_1=0;

static float pitch_target_angle=0.0f;
static float pitch_target_speed=0.0f;
static float pitch_now_angle=0.0f;

static short yaw_motor_output;     //底盘电机输出

static float turn_angle;

static uint8_t reload_flag=0;//填弹判断

static uint16_t flag=0;//填弹判断

static int16_t last_x;
static int16_t last_y;
static int16_t last_z;
static int16_t mousemove_flag=0;

double map(double x, double min,double max, double min_t,double max_t)//将0到8191转化为-pi到pi
{
    return(x-min)*(max_t - min_t) / (max-min)+min_t;
}


/*************************************************************************************************






*************************************************************************************************/

void gimbaltask()
{
    gimbal_data_refresh();
    gimbal_remote_calc();
    gimbal_calc_task();
    gimbal_data_send();
}

/*******************初始化********************
1.云台yaw轴回正面（机械角）
2.云台picth轴回水平（陀螺仪）
3.获取机械初始角（是否有必要有待商榷，反正你爹我先写）
4.获取空间初始角
*********************************************/
void gimbal_init(void)
{
    gimbal_data_refresh();

    yaw.initial_ecd = 2335;
    yaw.target_ecd = 2335;
    yaw.initial_angle = yaw.now_angle;
    yaw.target_angle = yaw.now_angle;


    pit_1.initial_angle = 100.0f;
    pit_1.target_angle = 0.0f;
    pit_1.last_target_angle=0.0f;

    pit_2.initial_angle = 0.0f;
    pit_2.target_angle = 0.0f;
    pit_2.last_target_angle=0.0f;

}

void gimbal_data_refresh(void)
{

    yaw.now_ecd = DATAGIMBALYAW.ecd;

    yaw.now_angle = JY901_data.angle.angle[2];

    yaw.now_speed = JY901_data.w.w[2];

    pit_1.now_ecd = DATAGIMBALPITCH_1.ecd;
    pit_1.now_angle = JY901_data.angle.angle[0];
    pit_1.now_speed = JY901_data.w.w[0];

    pit_2.now_ecd = DATAGIMBALPITCH_2.ecd;
    pit_2.now_angle = JY901_data.angle.angle[0];
    pit_2.now_speed = JY901_data.w.w[0];

    chassis_motor_parameter.chassis_relative_angle = relative_angle_calc(DATAGIMBALYAW.ecd,yaw.initial_ecd);
}

/*******************云台任务********************
1.遥控器映射空间绝对增角
2.云台picth轴回水平（陀螺仪）
3.获取机械初始角（是否有必要有待商榷，反正你爹我先写）
4.获取空间初始角
*********************************************/
void gimbal_remote_calc(void)
{

    if(remote_mode==1)
    {
        pit_1.add_angle=(double)rc.remote.ch2*GIMBAL_PIT_RC_SEN;
        SATURATE(pit_1.add_angle,-0.2f,0.2f);//pitch轴每次不超过0.2度转向
        //pit_2.add_angle=(double)rc.remote.ch2*GIMBAL_PIT_RC_SEN;
        //SATURATE(pit_2.add_angle,-0.4f,0.4f);//pitch轴每次不超过0.4度转向

        pit_1.target_angle=pit_1.target_angle+pit_1.add_angle;
        //pit_2.target_angle=pit_2.target_angle+pit_2.add_angle;

    }

    if(remote_mode==2)
    {
        if(rc.remote.ch2>=0)
        {
            pit_1.target_angle=-((double)rc.remote.ch2)*33/660;
           // pit_2.target_angle=((double)rc.remote.ch2)*33/660;
        }
        if(rc.remote.ch2<0)
        {
            pit_1.target_angle=-((double)rc.remote.ch2)*33/660;
            // pit_2.target_angle=-((double)rc.remote.ch2)*33/660;
        }
    }
    else if(remote_mode == 3)/*自瞄模式*/
    {
        pit_1.target_angle = target_angle_calc(pit_1.target_angle,AutoAim_device1.pitch_offset);
        AutoAim_device1.pitch_offset = 0;
    }

    SATURATE(pit_1.target_angle,-30.0f,23.0f);    //软件限角：俯下最多13°，仰起最多33°，实际角度：-13.4~33.4
    SATURATE(pit_2.target_angle,-30.0f,23.0f);   //软件限角：俯下最多13°，仰起最多33°，实际角度：-13.4~33.4

    /*判断是否在自瞄模式*/
    if(remote_mode != 3){
        yaw.add_angle=(double)rc.remote.ch1*GIMBAL_YAW_RC_SEN;
        SATURATE(yaw.add_angle,-0.85f,0.85f);//yaw轴每次不超过1.5度转向
        yaw.target_angle = target_angle_calc(yaw.target_angle,-yaw.add_angle);
    }
    else{
        SATURATE(AutoAim_device1.yaw_offset,-0.85f,0.85f);
        yaw.target_angle= target_angle_calc(yaw.target_angle,-AutoAim_device1.yaw_offset);
        AutoAim_device1.yaw_offset=0;
    }
}

void gimbal_calc_task(void)
{
    static uint8_t yaw_calc_counter = 0;
    static uint8_t pitch_calc_counter = 0;


    yaw.last_error_angle=yaw.error_angle;
    yaw.error_angle=error_angle_calc(yaw.target_angle,yaw.now_angle);
    if(++yaw_calc_counter == 2){
        yaw.target_speed=gimbal_yaw_angle_init.Calc(&gimbal_yaw_angle_init,(-yaw.error_angle*3.14/180),0);//yaw轴角度环pid，单位统一：rad/s
        yaw_calc_counter = 0;
    }
    yaw.send_data=gimbal_yaw_speed_init.Calc(&gimbal_yaw_speed_init,(yaw.now_speed/6),yaw.target_speed);//yaw轴速度环pid，单位统一：r/min



    pit_1.last_error_angle = pit_1.error_angle;
    pit_1.error_angle=error_angle_calc(pit_1.target_angle,pit_1.now_angle);
    if(++pitch_calc_counter == 2){
        pit_1.target_speed=gimbal_pitch_angle_init.Calc(&gimbal_pitch_angle_init,(pit_1.now_angle*3.14/180),((pit_1.target_angle - 10.0)*3.14/180));//pitch轴角度环pid，单位统一：rad/s
        pitch_calc_counter = 0;
    }
    pit_1.send_data=gimbal_pitch_speed_init.Calc(&gimbal_pitch_speed_init,(pit_1.now_speed/6),pit_1.target_speed);


    gimbal_motor_data.targrt_1=yaw.send_data;
    gimbal_motor_data.targrt_2=-pit_1.send_data;

}

static float relative_angle_calc(unsigned int angle, unsigned int initial_angle)//机械差角计算
{
    int32_t relative_ecd = angle - initial_angle;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }
    return relative_ecd * Motor_Ecd_to_Rad;
}

static float target_angle_calc(float angle, float add)
{
    float relative_angle = angle + add;
    if (relative_angle > 180)
    {
        relative_angle -= 360;
    }
    else if (relative_angle < -180)
    {
        relative_angle += 360;
    }
    return relative_angle;
}

static float filtering(float last_angle, float angle)
{
    float error_angle = angle;
    if(fabs(angle-last_angle)>90)
    {
        error_angle=last_angle;
    }
    return error_angle;
}

void gimbal_data_send(void)//云台数据发送
{
    CAN_cmd_gimbal(&gimbal_motor_data);
}

/***************************************云台部分********************************************/
void gimbal_calc(void)//随动数据计算
{
    pitch_now_angle=map(DATAGIMBALPITCH_1.ecd,0,8191,-PI,PI);
    chassis_motor_parameter.chassis_relative_angle = relative_angle_calc(DATAGIMBALYAW.ecd,initial_angle);
}



