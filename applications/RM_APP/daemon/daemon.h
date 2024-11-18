#ifndef DAEMON_H
#define DAEMON_H

#include "stdint.h"
#include "string.h"

#define DAEMON_MX_CNT 64

/* ģ�����ߴ�����ָ�� */
typedef void (*offline_callback)(void *);

/* daemon�ṹ�嶨�� */
typedef struct daemon_ins
{
    uint16_t reload_count;     // ����ֵ
    offline_callback callback; // �쳣������,��ģ�鷢���쳣ʱ�ᱻ����

    uint16_t temp_count; // ��ǰֵ,��Ϊ��˵��ģ�����߻��쳣
    void *owner_id;      // daemonʵ���ĵ�ַ,��ʼ����ʱ������
} DaemonInstance;

/* daemon��ʼ������ */
typedef struct
{
    uint16_t reload_count;     // ʵ��������appΨһ��Ҫ���õ�ֵ?
    uint16_t init_count;       // ���ߵȴ�ʱ��,��Щģ����Ҫ�յ����ص�ָ��Żᷴ������,��pc����Ҫ����ʱ��
    offline_callback callback; // �쳣������,��ģ�鷢���쳣ʱ�ᱻ����

    void *owner_id;            // idȡӵ��daemon��ʵ���ĵ�ַ,��DJIMotorInstance*,cast��void*����
} Daemon_Init_Config_s;

/**
 * @brief ע��һ��daemonʵ��
 *
 * @param config ��ʼ������
 * @return DaemonInstance* ����ʵ��ָ��
 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);

/**
 * @brief ��ģ���յ��µ����ݻ������������ʱ,���øú�������temp_count,�൱��"ι��"
 *
 * @param instance daemonʵ��ָ��
 */
void DaemonReload(DaemonInstance *instance);

/**
 * @brief ȷ��ģ���Ƿ�����
 *
 * @param instance daemonʵ��ָ��
 * @return uint8_t �������ҹ�������,����1;���򷵻���. ���������쳣���ͺ�����״̬�Ƚ����Ż�.
 */
uint8_t DaemonIsOnline(DaemonInstance *instance);

/**
 * @brief ����rtos��,���ÿ��daemonʵ����temp_count��Ƶ�ʽ��еݼ�����.
 *        ģ��ɹ��������ݻ�ɹ������������temp_count��ֵΪreload_count.
 *
 */
void DaemonTask();

#endif // !MONITOR_H