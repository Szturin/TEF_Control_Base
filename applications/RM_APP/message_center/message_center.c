#include "message_center.h"
#include "stdlib.h"
#include "string.h"
#include "strings.h"

/* message_center��fake head node,�Ƿ��������д�ļ���,�����Ͳ���Ҫ��������ͷ��������� */
static Publisher_t message_center = {
        .topic_name = "Message_Manager",
        .first_subs = NULL,
        .next_topic_node = NULL};

size_t my_strnlen(const char *str, size_t maxlen) {
    size_t len = 0;
    while (len < maxlen && str[len] != '\0') {
        len++;
    }
    return len;
}


static void CheckName(char *name)
{
    if (my_strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
        //LOGERROR("EVENT NAME TOO LONG:%s", name);
        while (1)
            ; // ��������˵��������������������
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        //LOGERROR("EVENT LEN NOT SAME:%d,%d", len1, len2);
        while (1)
            ; // ��������˵����ͬ�������Ϣ����ȴ��ͬ
    }
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    CheckName(name);
    Publisher_t *node = &message_center;
    while (node->next_topic_node) // message_center��ֱ������,����Ҫ���⴦��,�ⱻ����dumb_head(��̼���)
    {
        node = node->next_topic_node;            // �л�����һ��������(����)���
        if (strcmp(node->topic_name, name) == 0) // ����Ѿ�ע������ͬ�Ļ���,ֱ�ӷ��ؽ��ָ��
        {
            CheckLen(data_len, node->data_len);
            node->pub_registered_flag = 1;
            return node;
        }
    } // �����귢����δ����name��Ӧ�Ļ���
    // ������β�������µĻ��Ⲣ��ʼ��
    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->pub_registered_flag = 1;
    return node->next_topic_node;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    Publisher_t *pub = PubRegister(name, data_len); // ���һ򴴽��û���ķ�����
    // �����µĶ����߽��,�����ڴ�,ע��Ҫmemset��Ϊ�¿ռ䲻һ���ǿյ�,������֮ǰ���������ֵ
    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    memset(ret, 0, sizeof(Subscriber_t));
    // ���½���Subscriber���г�ʼ��
    ret->data_len = data_len; // �趨���ݳ���
    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    { // ����Ϣ���е�ÿһ��Ԫ�ط���ռ�,queue�ﱣ���ʵ����������ִָ��,�������Լ��ݲ�ͬ�����ݳ���
        ret->queue[i] = malloc(data_len);
    }
    // ����ǵ�һ��������,���⴦��һ��,��first_subsָ��ָ���½��Ķ�����(����ĵ�)
    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        return ret;
    }
    // ���û����Ѿ��ж�����, ��������������,ֱ������β��
    Subscriber_t *sub = pub->first_subs; // ��Ϊiterator
    while (sub->next_subs_queue)         // ���������˸û���Ķ���������
    {
        sub = sub->next_subs_queue; // �ƶ�����һ��������,������ָ��ͣ��,˵����������β��
    }
    sub->next_subs_queue = ret; // �Ѹոմ����Ķ����߽���
    return ret;
}

/* �������Ϊ��,�᷵��0;�ɹ���ȡ����,����1;����������������޸�,����ʣ����Ϣ��Ŀ�� */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    if (sub->temp_size == 0)
    {
        return 0;
    }
    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx++) % QUEUE_SIZE; // ����ͷ��������
    sub->temp_size--;                                 // popһ������,���ȼ�1
    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    static Subscriber_t *iter;
    iter = pub->first_subs; // iter��Ϊ������ָ��,�������ĸû�������ж�����;���Ϊ��˵����������
    // ���������˵�ǰ��������ж�����,��������������Ϣ
    while (iter)
    {
        if (iter->temp_size == QUEUE_SIZE) // �����������,����Ҫɾ�����ϵ�����(ͷ��),������
        {
            // ����ͷ����ǰ�ƶ�,�൱������ǰһ��λ�õ�����,��������λ���Ժ�ᱻд���µ�����
            iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
            iter->temp_size--; // �൱�ڳ���,size-1
        }
        // ��Pub�����ݸ��Ƶ����е�β��(����)
        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
        iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE; // ����β��ǰ��
        iter->temp_size++;                                  // ���,size+1

        iter = iter->next_subs_queue; // ������һ��������
    }
    return 1;
}