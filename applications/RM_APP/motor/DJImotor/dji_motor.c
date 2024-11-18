#include "dji_motor.h"

static uint8_t idx = 0; // register idx,�Ǹ��ļ���ȫ�ֵ������,��ע��ʱʹ��
/* DJI�����ʵ��,�˴�������ָ��,�ڴ�ķ��佫ͨ�����ʵ����ʼ��ʱͨ��malloc()���� */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // ����control�����б�����ָ���������pid����

/**
 * @brief ����DJI����������ĸ�һ�����ʽ����,�ʶ���������⴦��,��6��(2can*3group)can_instanceר�Ÿ�����
 *        �ñ������� DJIMotorControl() ��ʹ��,������ MotorSenderGrouping()�н���
 *
 * @note  ��Ϊֻ���ڷ���,���Բ���Ҫ��bsp_can��ע��
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * ����(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
        [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
        [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
        [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
        [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
        [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
        [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

/**
 * @brief 6������ȷ���Ƿ��е��ע�ᵽsender_assignment�еı�־λ,��ֹ���Ϳ�֡,�˱�������DJIMotorControl()ʹ��
 *        flag�ĳ�ʼ���� MotorSenderGrouping()�н���
 */
static uint8_t sender_enable_flag[6] = {0};

/**
 * @brief ���ݵ��/���뿪���ϵ�ID,����˵�����Ĭ��id���䷽ʽ���㷢��ID�ͽ���ID,
 *        ���Ե�����з����Ա㴦�������������
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // �±���㿪ʼ,�ȼ�һ���㸳ֵ
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // ����ID����
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }

        // �������id�����÷��鷢��id
        config->rx_id = 0x200 + motor_id + 1;   // ��ID+1,���з�������
        sender_enable_flag[motor_grouping] = 1; // ���÷��ͱ�־λ,��ֹ���Ϳ�֡
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // ����Ƿ���id��ͻ
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                //LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1); // 6020��id 1-4��2006/3508��id 5-8�ᷢ����ͻ(����ע��,��1!5,2!6,3!7,4!8) (1!5!,LTC! (((����)
                    //LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
        }

        config->rx_id = 0x204 + motor_id + 1;   // ��ID+1,���з�������
        sender_enable_flag[motor_grouping] = 1; // ֻҪ�е��ע�ᵽ�������,��Ϊ1;�ڷ��ͺ����л�ͨ���˱�־�ж��Ƿ��е��ע��
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                //LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020��id 1-4��2006/3508��id 5-8�ᷢ����ͻ(����ע��,��1!5,2!6,3!7,4!8) (1!5!,LTC! (((����)
                    //LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
                my_printf(&huart2,"[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    default: // other motors should not be registered here
        while (1);
            //LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor."); // ���������Ӧ��������ע��
    }
}

/**
 * @todo  �Ƿ���Լ򻯶�Ȧ�Ƕȵļ��㣿
 * @brief ���ݷ��ص�can_instance�Է������Ľ��н���
 *
 * @param _instance �յ����ݵ�instance,ͨ�����������е�����жԱ���ѡ����ȷ��ʵ��
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // �����can instance��id������ǿ��ת��,�Ӷ���õ����instanceʵ����ַ
    // _instanceָ��ָ���id�Ƕ�Ӧ���instance�ĵ�ַ,ͨ��ǿ��ת��Ϊ���instance��ָ��,��ͨ��->��������ʵ���ĳ�Աmotor_measure,���ȡ��ַ���ָ��
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measureҪ���ʹ��,����ָ���С�ô濪��

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // �������ݲ��Ե������ٶȽ����˲�,����ķ������ľ����ʽ�����˵���ֲ�
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // ��Ȧ�Ƕȼ���,ǰ���Ǽ������β�������ת���ĽǶ�С��180��,�Լ�����ͼ��������������
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    //LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
}

// �����ʼ��,����һ�����ʵ��
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting �����������
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // ����ת,�ջ����͵�

    // motor controller init �����������ʼ��
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // �������ӵ��ǰ��������(�ٶȺ͵���)

    // �������,��Ϊ����4��������Թ���һ֡CAN���Ʊ���
    MotorSenderGrouping(instance, &config->can_init_config);

    // ע������CAN����
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // ע���ػ��߳�
    Daemon_Init_Config_s daemon_config = {
            .callback = DJIMotorLostCallback,
            .owner_id = instance,
            .reload_count = 2, // 20msδ�յ�������ʧ
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* ����ֻ��ͨ������Դ����������,�������Ǽ������ش�����Ӧ��Ƭ�� */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        __NOP();
        //LOGERROR("[dji_motor] loop type error, check memory access and func param"); // ����Ƿ�������ȷ��LOOP����,������ָ��Խ��
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* �޸ĵ����ʵ�ʱջ����� */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// ���òο�ֵ
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

// Ϊ���е��ʵ����������PID,���Ϳ��Ʊ���
void DJIMotorControl()
{
    // ֱ�ӱ���һ��ָ�����ôӶ���С�ô�Ŀ���,ͬ��������߿ɶ���
    uint8_t group, num; // �����ź����ڱ��
    int16_t set;        // �������CAN�����趨ֵ
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // ������Ʋ���
    Motor_Controller_s *motor_controller;   // ���������
    DJI_Motor_Measure_s *measure;           // �������ֵ
    float pid_measure, pid_ref;             // ���PID����ֵ���趨ֵ

    // �������е��ʵ��,���д���PID�ļ��㲢���÷��ͱ��ĵ�ֵ
    for (size_t i = 0; i < idx; ++i)
    { // ��С�ô濪��,�ȱ���ָ������
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // �����趨ֵ,��ֹmotor_controller->pid_ref�ڼ�������б��޸�
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // ���÷�ת

        // pid_ref��˳��ͨ�������õıջ��䵱���ݵ�����
        // ����λ�û�,ֻ������λ�û������ջ�Ϊλ��ʱ������ٶȻ����
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,��total angle�ջ�,��ֹ�ڱ߽紦����ͻԾ
            // ����pid_ref������һ����
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // �����ٶȻ�,(���ջ�Ϊ�ٶȻ�λ��)��(�����ٶȻ�)ʱ������ٶȻ�
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
            // ����pid_ref������һ����
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // ���������,ĿǰֻҪ�����˵������ͼ���,�������ջ���ʲô,���ҵ���ֻ�е�����������ķ���
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // ��ȡ�������
        set = (int16_t)pid_ref;

        // �������뷢������
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // �Ͱ�λ
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // �߰�λ

        // ���õ������ֹͣ״̬,ֱ�ӽ�buff����
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // ����flag,����Ƿ�Ҫ������һ֡����
    for (size_t i = 0; i < 6; ++i)
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}