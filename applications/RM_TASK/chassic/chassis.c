/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief ����Ӧ��,�������robot_cmd�Ŀ������������������˶�ѧ����,�õ����
 *        ע����̲�ȡ����ϵ,����ƽ����ͼ,���������˶�����ǰ��Ϊx������;�����˶����Ҳ�Ϊy������
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "motor/DJImotor//dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"

/* ����robot_def.h�е�macro�Զ�����Ĳ��� */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // �����
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // ���־�
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // �����ܳ�

/* ����Ӧ�ð�����ģ�����Ϣ�洢,�����ǵ���ģʽ,��˲���ҪΪ���̽��������Ľṹ�� */
#ifdef CHASSIS_BOARD // ����ǵ��̰�,ʹ�ð���IMU��ȡ����ת�����ٶ�
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // ˫��ͨ��CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // ���ڷ������̵�����
static Subscriber_t *chassis_sub;                   // ���ڶ��ĵ��̵Ŀ�������
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // ���̽��յ��Ŀ�������
static Chassis_Upload_Data_s chassis_feedback_data; // ���̻ش��ķ�������

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* �����������ٲ��Ե�ʱ����� */
// static float t;

/* ˽�к���������н����,��Ϊ��̬����������ݵĿ��� */
static float chassis_vx, chassis_vy;     // ����̨ϵ���ٶ�ͶӰ������
static float vt_lf, vt_rf, vt_lb, vt_rb; // �����ٶȽ�������ʱ���,�������޷�

void ChassisInit()
{
    // �ĸ����ӵĲ���һ��,��tx_id�ͷ�ת��־λ����
    Motor_Init_Config_s chassis_motor_config = {
            .can_init_config.can_handle = &hcan1,
            .controller_param_init_config = {
                    .speed_PID = {
                            .Kp = 10, // 4.5
                            .Ki = 0,  // 0
                            .Kd = 0,  // 0
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 12000,
                    },
                    .current_PID = {
                            .Kp = 0.5, // 0.4
                            .Ki = 0,   // 0
                            .Kd = 0,
                            .IntegralLimit = 3000,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .MaxOut = 15000,
                    },
            },
            .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            },
            .motor_type = M3508,
    };
    //  @todo: ��ǰ��û�����õ��������ת,��Ȼ��Ҫ�ֶ����reference��������,��Ҫ���module��֧��,���޸�.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    //referee_data = UITaskInit(&huart6,&ui_data); // ����ϵͳ��ʼ��,��ͬʱ��ʼ��UI


    /*
    SuperCap_Init_Config_s cap_conf = {
            .can_config = {
                    .can_handle = &hcan2,
                    .tx_id = 0x302, // ��������Ĭ�Ͻ���id
                    .rx_id = 0x301, // ��������Ĭ�Ϸ���id,ע��tx��rx�������˿����Ƿ���
            }};
    cap = SuperCapInit(&cap_conf); // �������ݳ�ʼ��
*/

    // �������ĳ�ʼ��,���Ϊ˫��,����Ҫcan comm��������Ϣ
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // ����IMU��ʼ��

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm��ʼ��
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // �����������,��ͨ��pubsub��������Ϣ
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/**
 * @brief ����ÿ����챵�������,���˶�ѧ����
 *        �ú����Ԥ�滻��С����,�˶����������̲ο��̳�
 */
static void MecanumCalculate()
{
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}
/**
 * @brief ����ÿ�����ӵ��ٶȷ���,������̵�ʵ���˶��ٶ�,���˶�����
 *        ����˫������,�����������Ե��̰�IMU������
 *
 */
static void EstimateSpeed()
{
    // ���ݵ���ٶȺ������ǵĽ��ٶȽ��н���,���������ü��ٶȼ��ж��Ƿ��(�����)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* �����˵��̿��ƺ������� */
void ChassisTask()
{
    // ��������û�յ���Ϣ�Ĵ���(˫������)
    // ��ȡ�µĿ�����Ϣ
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // ���������Ҫģ�����߻�ң��������Ϊ��ͣ,�õ��ֹͣ
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // ��������
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // ���ݿ���ģʽ�趨��ת�ٶ�
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // ���̲���ת,��ά��ȫ�����,һ�����ڵ�����̨��̬
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // ������̨,����������pid,�����Ƕ�ƽ��Ϊ�ٶ����
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // ����,ͬʱ����ȫ�����;��ǰwzά�ֶ�ֵ,�������Ӳ�����ı��ٲ���
        chassis_cmd_recv.wz = 4000;
        break;
    default:
        break;
    }

    // ������̨�͵��̵ĽǶ�offset��������ӳ�䵽��������ϵ��
    // ������ʱ����תΪ�Ƕ�������;��̨����ķ�������ָ̨��ķ���Ϊx,��������ϵ(xָ������ʱy������)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // ���ݿ���ģʽ�������˶�ѧ����,����������
    MecanumCalculate();

    // ���ݲ���ϵͳ�ķ������ݺ͵������ݶ�����޷����趨�ջ��ο�ֵ
   // LimitChassisOutput();

    // ���ݵ���ķ����ٶȺ�IMU(�����)������ʵ�ٶ�
    EstimateSpeed();

    // // ��ȡ����ϵͳ����   ���齫����ϵͳ����̷��룬���Դ˴�����Ӧʹ����Ϣ���ķ���
    // // �ҷ���ɫidС��7�Ǻ�ɫ,����7����ɫ,ע�����﷢�͵��ǶԷ�����ɫ, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // ��ǰֻ����17mm���������ݻ�ȡ,��������robot_def�еĺ��л�˫ǹ�ܺ�Ӣ��42mm�����
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // ���ͷ�����Ϣ
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}