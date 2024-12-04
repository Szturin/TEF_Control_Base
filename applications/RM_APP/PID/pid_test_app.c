#include "pid_test_app.h"

/*ȫ�ֱ��� warning ������*/
PID_Params_t PID_Test;

// ��ʼ��PID����
//void PID_Init(PID_Params_t *pid, float kp, float ki, float kd) {
//    pid->Kp = kp;
//    pid->Ki = ki;
//    pid->Kd = kd;
//}

// ����λ���������������PID����
void PID_UpdateFromCommand(PID_Params_t *pid, const char *command) {
    if (strncmp(command, "Kp:", 3) == 0) {
        pid->Kp = atof(command + 3);
    } else if (strncmp(command, "Ki:", 3) == 0) {
        pid->Ki = atof(command + 3);
    } else if (strncmp(command, "Kd:", 3) == 0) {
        pid->Kd = atof(command + 3);
    } else if (strncmp(command, "Ks:", 3) == 0) {
        pid->Ks = atof(command + 3);
    }
}

// ���ڴ�ӡPID����
void PID_PrintParams(UART_HandleTypeDef *huart, PID_Params_t *pid) {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Kp: %.2f, Ki: %.2f, Kd: %.2f\r\n",
             pid->Kp, pid->Ki, pid->Kd);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
