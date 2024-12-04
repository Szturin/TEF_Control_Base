#ifndef __RM_DEFINE_H__  //此头文件用来放一些RM的宏定义   
#define __RM_DEFINE_H__

#include "applications/RM_BSP/bsp_system.h"
/***************************************常数定义*********************************************/
#define PI 3.141592653f

/***************************************遥控相关定义*********************************************/
//遥控器通道1和通道4的倍率 底盘电机和云台电机的最大值不同，所以根据需要改倍率即可
#define R_ch1 3
#define R_ch2 1
#define R_ch3 1
#define R_ch4 1
//此处用来设置到达最MAX_Motor_SPeed限速值的速率，值越大越快到达MAX_Motor_SPeed。此处最大值为24.8242422
//切记660*R_chX(X=1，2，3，4其中之一)需大于MAX_Motor_SPeed 否则最大速度将由660*R_chX主导

/***************************************底盘相关定义*********************************************/
//Chassis 底盘的一些宏定义
#define ROLL_FORWARD 1
#define ROLL_BACK    -1

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.002f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例

#define CHASSIS_WZ_RC_SEN 0.01f
//rpm是每分钟的旋转次数
//m3508转化成底盘速度(m/s)的比例
//比例系数=周长/减速比/60 注：/60是为了换算成s单位
//举个例子
//已知电机的速度为400rpm，轮子的直径为185mm，电机的减速比为1：19，求当前轮子的速度？

//解： 根据上面的公式得
//比例系数=(0.1852*PI/2)/19/60=0.0005098;
//所以：轮子的速度=400* (0.1852Π/2)/19/60=0.203927(m/s)；
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f//0.0002098115409482972879038f

#define CHASSISMOTOR_TO_CENTER 0.2723967694   //需要测量麦克纳姆轮离几何中心的水平距离和垂直距离，再用勾股定理求得该数值 单位（m）
#define CHASSIS_VX_SPEED  0.25f  //为什么是0.25,VX的速度为四个轮的速度叠加，除4则为底盘速度
#define CHASSIS_VY_SPEED  0.25f
#define CHASSIS_VW_SPEED  0.25f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.0f

#define MAX_Motor_SPeed 3.0f //速度限制  0.5m/s
//设回中位置到推杆到达最外侧位置之间的距离为D，则电机速度达到MAX_Motor_SPeed的值时，此时推杆位置为(MAX_Motor_SPeed/660*R_chX)*D处
//举个例子MAX_Motor_SPeed=2000，660*R_chX=6000，则电机速度达到MAX_Motor_SPeed的值时，推杆位置位于2000/6000也就是（1/3）D处
//2022-12-22 滕治留

#define CHASSIS_CAN hcan1//底盘采用can1口通讯

/***************************************云台相关定义*********************************************/
#define Motor_Ecd_to_Rad   0.000766990394f //      2*  PI  /8192  角度转换

#define Half_ecd_range 4096   //电机码盘值中值
#define ecd_range 8191                //电机码盘值最大值

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define GIMBAL_YAW_RC_SEN 0.00300f
#define GIMBAL_PIT_RC_SEN 0.00115f

#define GIMBAL_CAN hcan2//云台预采用can2口通讯

/*************************************发射机构相关定义*******************************************/
//2006电机转化成拨弹盘切向速度
#define M2006_MOTOR_RPM_TO_VECTOR 0.000036361025462962f

//三档摩擦轮射速
#define shoot_stop_speed 1000
#define shoot_basic_speed 1480
#define shoot_leveltwo_speed 1530
#define shoot_levelthree_speed 1680

//匀速拨弹速度
#define feed_target_speed -0.45
#define feed_target_second_speed -0.75

//拨弹轮模式设定
#define STOP 0
#define BACK 1
#define SPEED_1 2
#define SPEED_2 3

//填弹模式设定
#define CLOSE 0
#define OPEN 1

#define SHOOT_CAN hcan2

/*************************************键盘通道*******************************************/
//键盘是否激活
#define NOT_READY 0
#define READY 1

//键盘通道下的小车模式
#define BASIC 0//普通运动
#define GLOBAL 1//自转小陀螺
#define FOLLOW 2//底盘随动zx

//移动模式
#define SLOW_SPEED 0.6f
#define NORMAL_SPEED 0.95f
#define FAST_SPEED 1.2f

//鼠标射击模式
#define burst  0//点射
#define strafe 1//扫射

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 }

#endif

