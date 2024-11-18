#include "bsp_dwt.h"
#include "rtthread.h"
#include "stdio.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;

/**
 * @brief ˽�к��������ڼ��RTT�δ�����Ƿ�����������¼�����
 */
static void DWT_CNT_Update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = rt_tick_get();  // ʹ�� RT-Thread �� tick ��ȡ��ǰϵͳʱ��
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = cnt_now;
        bit_locker = 0;
    }
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* ��ȡRTT��ϵͳʱ��Ƶ�� */
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

    /* ���¼����� */
    DWT_CNT_Update();
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = rt_tick_get();  // ��ȡϵͳʱ�ӣ��δ������
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));  // ����ʱ���
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = rt_tick_get();  // ��ȡϵͳʱ�ӣ��δ������
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));  // ����ʱ���
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    uint32_t cnt_now = rt_tick_get();  // ��ȡϵͳʱ�ӣ��δ������
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = rt_tick_get();  // ��ȡ��ǰ��ϵͳ�δ����
    float wait = Delay;

    // ʹ�� RT-Thread ��ʱ��ѭ��ֱ����ʱ����
    while ((rt_tick_get() - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
