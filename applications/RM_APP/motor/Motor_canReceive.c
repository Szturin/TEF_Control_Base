#include "Motor_canReceive.h"

//motor3510_TypeDef DATASPEED1;
//motor3510_TypeDef DATASPEED2;
//motor3510_TypeDef DATASPEED3;
//motor3510_TypeDef DATASPEED4;

/*
***************************************************
��ע��CAN�жϽ������ݴ�����  1ms
***************************************************
*/

/*
void CAN_ReceiveMsg(CanRxMsg *msg)
{
//	CanRxMsg msgcorre;
	switch(msg->StdId)
	{
		case CAN_driver1ID:           //����1�����ݶ�ȡ
		{   
		 Driver_RM3510_ReadData(msg,&DATASPEED1);       //��ȡ���ٶ�       
		}	break;
		case CAN_driver2ID:
		{   
			Driver_RM3510_ReadData(msg,&DATASPEED2); ;       //��ȡ���ٶ�  

		}break;
		case CAN_driver3ID:
		{   
			Driver_RM3510_ReadData(msg,&DATASPEED3); ;       //��ȡ���ٶ�;
		}break;
		case CAN_driver4ID:
		{
			Driver_RM3510_ReadData(msg,&DATASPEED4); ;       //��ȡ���ٶȣ�
		}break;
	}
}

*/









