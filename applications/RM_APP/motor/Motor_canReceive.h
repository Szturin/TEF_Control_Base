#ifndef Motor_canReceive_H
#define Motor_canReceive_H

#include "applications/RM_BSP/bsp_system.h"

#define CAN_driver1ID 0x201
#define CAN_driver2ID 0x202
#define CAN_driver3ID 0x203
#define CAN_driver4ID 0x204

void CAN_ReceiveMsg(CAN_RxHeaderTypeDef *msg);

#endif









