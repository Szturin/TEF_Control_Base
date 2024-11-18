#include "bsp_can_demo.h"


/* can instance ptrs storage, used for recv callback */
// ��CAN���������жϻ��������,ѡ��hcan��rxid�뷢���жϵ�ʵ����ͬ���Ǹ�,������ص�����
// @todo: ����Ϊÿ��CAN���ߵ������һ��can_instanceָ������,��߻ص����ҵ�����
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // ȫ��CANʵ������,ÿ�����µ�ģ��ע�������

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief ��ӹ�������ʵ�ֶ��ض�id�ı��ĵĽ���,�ᱻCANRegister()����
 *        ��CAN��ӹ�������,BxCAN����ݽ��յ��ı��ĵ�id������Ϣ����,���Ϲ����id�ᱻ����FIFO�����ж�
 *
 * @note f407��bxCAN��28��������,���ｫ������Ϊǰ14����������CAN1ʹ��,��14����CAN2ʹ��
 *       ��ʼ��ʱ,����id��ģ��ᱻ���䵽FIFO0,ż��id��ģ��ᱻ���䵽FIFO1
 *       ע�ᵽCAN1��ģ��ʹ�ù�����0-13,CAN2ʹ�ù�����14-27
 *
 * @attention �㲻��Ҫ��ȫ����������������,��Ϊ����Ҫ�����ڳ�ʼ��,�ڿ��������в���Ҫ���ĵײ��ʵ��
 *            ���ܿ�������Ȥ��!����������֪����������ڸ�ʲô,����ϵ���߻��Լ���������(��ֱ�Ӳ��Ĺٷ���reference manual)
 *
 * @param _instance can instance owned by specific module
 */
static void CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13��can1��,14-27��can2��

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // ʹ��id listģʽ,��ֻ�н�rxid��ӵ��������вŻ���յ�,�������Ļᱻ����
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // ʹ��16λidģʽ,��ֻ�е�16λ��Ч
    can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // ����id��ģ��ᱻ���䵽FIFO0,ż��id��ģ��ᱻ���䵽FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // �ӵ�14����������ʼ���ôӻ�������(��STM32��BxCAN��������CAN2��CAN1�Ĵӻ�)
    can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // �������Ĵ����ĵ�16λ,��Ϊʹ��STDID,����ֻ�е�11λ��Ч,��5λҪ��0
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // ����can_handle�ж���CAN1����CAN2,Ȼ������
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // ���ù�����

    HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

/**
 * @brief �ڵ�һ��CANʵ����ʼ����ʱ����Զ����ô˺���,����CAN����
 *
 * @note �˺���������CAN1��CAN2,����CAN1��CAN2��FIFO0 & FIFO1���֪ͨ
 *
 */
static void CANServiceInit()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit(); // ��һ��ע��,�Ƚ���Ӳ����ʼ��
        //LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // �������ʵ����
    {
        while (1);
            //LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    { // �ظ�ע�� | id�ظ�
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1);
                //LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // ����ռ�
    memset(instance, 0, sizeof(CANInstance));                           // ����Ŀռ�δ����0,����Ҫ�����
    // ���з��ͱ��ĵ�����
    instance->txconf.StdId = config->tx_id; // ����id
    instance->txconf.IDE = CAN_ID_STD;      // ʹ�ñ�׼id,��չid��ʹ��CAN_ID_EXT(Ŀǰû������)
    instance->txconf.RTR = CAN_RTR_DATA;    // ��������֡
    instance->txconf.DLC = 0x08;            // Ĭ�Ϸ��ͳ���Ϊ8
    // ���ûص������ͽ��շ���id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // ����û��,����ɾ��
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);         // ���CAN����������
    can_instance[idx++] = instance; // ��ʵ�����浽can_instance��

    return instance; // ����canʵ��ָ��
}

/* @todo Ŀǰ�ƺ���װ����,Ӧ�����һ��ָ��tx_buff��ָ��,tx_buff��Ӧ����CAN instance���� */
/* �����CANinstance����txbuff,������һ�θ��ƵĿ��� */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = rt_tick_get();

    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // �ȴ��������
    {
        if (rt_tick_get()- dwt_start > timeout) // ��ʱ
        {
            //LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    wait_time = rt_tick_get() - dwt_start;
    // tx_mailbox�ᱣ��ʵ����������һ֡��Ϣ������,����֪�����ĸ����䷢���ƺ�Ҳûɶ��
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
    {
        //LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1; // ���ͳɹ�
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // ���ͳ��ȴ���!�����ò����Ƿ����,�����Ұָ��/Խ�����
    if (length > 8 || length == 0) // ��ȫ���
        while (1)
            //LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
    _instance->txconf.DLC = length;
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief �˺����ᱻ����������������,���ڴ���FIFO0��FIFO1����ж�(˵���յ����µ�����)
 *        ���е�ʵ�����ᱻ����,�ҵ�can_handle��rx_id��ȵ�ʵ��ʱ,���ø�ʵ���Ļص�����
 *
 * @param _hcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // ͬ��
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO��Ϊ��,�п����������ж�ʱ�ж�֡���ݽ���
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // ��FIFO�л�ȡ����
        for (size_t i = 0; i < idx; ++i)
        { // �������˵������Ҫ�ҵ�ʵ��
            if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id)
            {
                if (can_instance[i]->can_module_callback != NULL) // �ص�������Ϊ�վ͵���
                {
                    can_instance[i]->rx_len = rxconf.DLC;                      // ������յ������ݳ���
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // ��Ϣ��������Ӧʵ��
                    can_instance[i]->can_module_callback(can_instance[i]);     // �����ص��������ݽ����ʹ���
                }
                return;
            }
        }
    }
}

/**
 * @brief ע��,STM32������CAN�豸��������FIFO
 * ��������������HAL���еĻص�����,���Ǳ�HAL����Ϊ__weak,��������ǽ�������(��д)
 * ��FIFO0��FIFO1���ʱ���������������
 */
// ����ĺ��������CANFIFOxCallback()����һ�����������ض�CAN�豸����Ϣ

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */

/**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); // ���������Լ�д�ĺ�����������Ϣ
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); // ���������Լ�д�ĺ�����������Ϣ
}
