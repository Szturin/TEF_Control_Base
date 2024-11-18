#include "Motor_canReceive.h"

//motor3510_TypeDef DATASPEED1;
//motor3510_TypeDef DATASPEED2;
//motor3510_TypeDef DATASPEED3;
//motor3510_TypeDef DATASPEED4;

/*
***************************************************
备注：CAN中断接收数据处理函数  1ms
***************************************************
*/

/*
void CAN_ReceiveMsg(CanRxMsg *msg)
{
//	CanRxMsg msgcorre;
	switch(msg->StdId)
	{
		case CAN_driver1ID:           //轮子1的数据读取
		{   
		 Driver_RM3510_ReadData(msg,&DATASPEED1);       //获取到速度       
		}	break;
		case CAN_driver2ID:
		{   
			Driver_RM3510_ReadData(msg,&DATASPEED2); ;       //获取到速度  

		}break;
		case CAN_driver3ID:
		{   
			Driver_RM3510_ReadData(msg,&DATASPEED3); ;       //获取到速度;
		}break;
		case CAN_driver4ID:
		{
			Driver_RM3510_ReadData(msg,&DATASPEED4); ;       //获取到速度；
		}break;
	}
}

*/









