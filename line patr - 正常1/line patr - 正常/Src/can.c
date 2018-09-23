/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void CAN_Config(CAN_HandleTypeDef* _hcan)
{
	  CAN_FilterTypeDef  CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	  CAN_FilterConfigStructure.FilterBank=0;
    CAN_FilterConfigStructure.FilterIdHigh=0x0000;
    CAN_FilterConfigStructure.FilterIdLow=0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh=0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow=0x0000;
    CAN_FilterConfigStructure.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.SlaveStartFilterBank=14;
    CAN_FilterConfigStructure.FilterActivation=ENABLE;
	 
		    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        //err_deadloop();
    }

    //filter config for can2
    //can1(0-13),can2(14-27)
		CAN_FilterConfigStructure.FilterBank=14;    
		if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
			
    }
}


 void CAN_Start(void) 
{
	CAN_Config(&hcan1);

	HAL_CAN_Start(&hcan1);

	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
}









/**************������װ����***********/

uint8_t Servo_Param_Set(uint8_t servo_id,uint8_t order,uint8_t len,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = servo_id;
	Servo_Structure.Data_Len		= len;
	Servo_Structure.Front_Data[0] = len;
	Servo_Structure.Front_Data[1] = servo_id;
	Servo_Structure.Front_Data[2] = order;
	Servo_Structure.Front_Data[3] = 0;
	Servo_Structure.Last_Data = data;
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}

/**
	*@brief	�ŷ�����������ָ���
	*@param	*Servo_Structure		��Ҫ���ŷ����Ʒ��͵�ָ�����
	*@retval	0�����ͳɹ�
						1����Ϣ����ʧ��
						2�����������Ϣ����ʧ��
	*/
uint8_t Servo_Set_Function(Servo_Struct *Servo_Structure)
{
	uint16_t i;
  uint32_t TxMailbox;
	CanSendData_Type           	TxMessage;  //��������
	
	TxMessage.SendCanTxMsg.IDE	= Servo_Structure->Data_Format;						//��׼֡��ʽ
	TxMessage.SendCanTxMsg.RTR		= Servo_Structure->Data_Type;							//����֡
	TxMessage.SendCanTxMsg.StdId = Servo_Structure->Servo_ID;							//��׼ID
	TxMessage.SendCanTxMsg.ExtId = 0;																			//��չID
	TxMessage.SendCanTxMsg.DLC 	= Servo_Structure->Data_Len;							//���͵����ݳ���,�ֽ���
	for(i=0;i<4;i++){																					//��������
		TxMessage.CAN_TX_Data[i]	= Servo_Structure->Front_Data[i];								
		TxMessage.CAN_TX_Data[i+4] = ((Servo_Structure->Last_Data)>>(i*8))&0xff;
	}
    
  HAL_CAN_AddTxMessage(&hcan1,&TxMessage.SendCanTxMsg,TxMessage.CAN_TX_Data, &TxMailbox);
	return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/************************************Ϊ��Ӧ���ָ������,������ʿ�������װ����ָ��**************************************/
/**					@brief��	�û���ʹ�ô˺���ǰ����Ҫ�����������ƿ�,��������ز���
									
**************************************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
	*@brief �ŷ�������������ʼ��
	*@param
	*@retval
	*/
void Create_MLServo_CB(SERVO_MLCB		*servo_mlcb,					
											 char       	*servo_mlcb_name,			
											 uint8_t     	 star_id,											
											 uint8_t       ratio,								
											 uint16_t      coder)				  
{
	servo_mlcb->Servo_Mlcb_Name   = servo_mlcb_name;			//�û��Զ�����������
	servo_mlcb->Star_Id						= star_id;							//��ʶ��ID
	servo_mlcb->Device_Id					= star_id;							//�豸ID
	servo_mlcb->Ratio							= ratio;								//���������
	servo_mlcb->Coder 						= coder;								//����������
	servo_mlcb->Servo_Error				= Write_Success;				//���󷵻ر�־
}
/**
	*@brief �ŷ�����������д��
	*@retval	0�����ͳɹ�
						1����Ϣ����ʧ��
						2�����������Ϣ����ʧ��
	*/
uint8_t MLServo_Write(SERVO_MLCB *Servo_MLCB, uint8_t order, uint8_t len, int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= len;
	Servo_Structure.Front_Data[0] = len;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = order;
	Servo_Structure.Front_Data[3] = 0;
	if				(		order==ML_SSP		/*��������ٶ�*/\
							||order==ML_SMV		/*������С�ٶ�*/\
							||order==ML_A			/* ���ü��ٶ� */\
							||order==ML_V			)/*���������ٶ�*/
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio));
	
	else if		(		order==ML_PO		/*���õ�ǰλ��Ϊ����λ��*/\
							||order==ML_M			/* ������趨λ���ƶ�   */\
							||order==ML_MR		)/*������趨λ���ƶ�(���λ��)*/
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio)*(4*(Servo_MLCB->Coder))/360);
	
	else
	Servo_Structure.Last_Data = data;
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}








/*************************************************����ٶ�����************************************************/
/*																								����ٶ�����																							 */
/*************************************************����ٶ�����************************************************/
/**
	*	@brief	�������ٶ�����
	*	@param	*Servo_MLCB	��Ҫ���Ƶĵ�����ƿ�
	*					data				�ٶȣ�ת/��
	*	@retval	error:	0	���ݷ��ͳɹ�	,���������ݷ���ʧ��
	*/
uint8_t Motor_MastSpeed(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_SSP_LEN;
	
	Servo_Structure.Front_Data[0] = ML_SSP_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_SSP;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio));
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}

/**
	*	@brief	�����С�ٶ�����
	*	@param	*Servo_MLCB	��Ҫ���Ƶĵ�����ƿ�
	*					data				�ٶȣ�ת/��
	*	@retval	error:	0	���ݷ��ͳɹ�	,���������ݷ���ʧ��
	*/
uint8_t Motor_MinSpeed(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_SMV_LEN;
	
	Servo_Structure.Front_Data[0] = ML_SMV_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_SMV;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio));
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}

/**
	*	@brief	��������ٶ�����,	������Դ��ٶȿ�ʼ����
	*	@param	*Servo_MLCB	��Ҫ���Ƶĵ�����ƿ�
	*					data				�ٶȣ�ת/��
	*	@retval	error:	0	���ݷ��ͳɹ�	,���������ݷ���ʧ��
	*/
uint8_t Motor_SetSpeed(SERVO_MLCB *Servo_MLCB,float data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_V_LEN;
	
	Servo_Structure.Front_Data[0] = ML_V_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_V;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio));
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}
/************************************************������ٶ�����**************************************************/
/*																							 ������ٶ�����																									*/
/************************************************������ٶ�����**************************************************/
/**
	*	@brief	���ٶ�����	
	*	@param	*Servo_MLCB	��Ҫ���Ƶĵ�����ƿ�
/						data				������	ת/(��*��)
	*	@retval	error	:	0	���ݷ��ͳɹ�,		����	���ݷ���ʧ��
	*/
uint8_t Motor_AccelSpeed(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len	= ML_A_LEN;
	
	Servo_Structure.Front_Data[0] = ML_A_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_A;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio));
		
	error = Servo_Set_Function(&Servo_Structure);

	return error;
}

/***********************************************���λ�ÿ���**************************************************/
/*																							���λ�ÿ���																								 */
/***********************************************���λ�ÿ���**************************************************/
/**
	*	@brief	�������λ������	���õ�ǰλ��Ϊ�������λ��
	*	@param	*Servo_MLCB	������ƿ�
						data				������	�Ƕ�
	*	@retval	error	0�����ݷ�������		���������ݷ���ʧ��
	*/
uint8_t Motor_AbsolutePlace(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_PO_LEN;
	
	Servo_Structure.Front_Data[0] = ML_PO_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_PO;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio)*(4*(Servo_MLCB->Coder))/360);
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}


/**
	*	@brief	�������λ������	������ɺ�������˾���λ������
	*	@param	*Servo_MLCB	������ƿ�
						data				������	�Ƕ�
	*	@retval	error	0�����ݷ�������		���������ݷ���ʧ��
	*/
uint8_t Motor_ToAbsolutePlace(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_M_LEN;
	
	Servo_Structure.Front_Data[0] = ML_M_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_M;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio)*(4*(Servo_MLCB->Coder))/360);
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}
/**
	*	@brief	������λ������	������趨λ�ÿ�ʼ�ƶ�
	*	@param	*Servo_MLCB	������ƿ�
						data				������	�Ƕ�
	*	@attention	���λ��Ϊ,������ϴε��趨λ��,������ڵ�����ڵ�ʵʱλ��
	*	@retval	error	0�����ݷ�������		���������ݷ���ʧ��
	*/
uint8_t Motor_ToRelativePlace(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_MR_LEN;
	
	Servo_Structure.Front_Data[0] = ML_MR_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_MR;
	Servo_Structure.Front_Data[3] = 0;
	
	//Servo_Structure.Last_Data = (data*(Servo_MLCB->Ratio)*(4*(Servo_MLCB->Coder))/360);    //�Ƕ�
	Servo_Structure.Last_Data =data ;	                                                    //����
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}



/********************************************����ĳ�ʼ������**********************************************/
/*																					 ����ĳ�ʼ������																							*/
/********************************************����ĳ�ʼ������**********************************************/
/**
	*	@brief	���λ�ñ�������	P
	*	@paaram	*Servo_MLCB	������ƿ�
						data				������
	*	@retval	error	0���������ݳɹ�	���������ݷ���ʧ��
	*/
uint8_t Motor_P_SET(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_MP_LEN;
	
	Servo_Structure.Front_Data[0] = ML_MP_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_MP;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = data;
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}

/**
	*	@brief	����ķֱ�������	����������
	*	@param	*Servo_MLCB		��Ҫ���õĿ��ƿ�
						data					�������ķֱ���
	*	@retval	error	0���������ݳɹ�	���������ݷ���ʧ��
	*/
uint8_t Motor_Code_Set(SERVO_MLCB *Servo_MLCB,int32_t data)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_ENC_LEN;
	
	Servo_Structure.Front_Data[0] = ML_ENC_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_ENC;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = (data*4);
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}



/**
	*	@brief	����Ĺ���ģʽ����	�ٶ�ģʽ
	*	@param	*Servo_MLCB		��Ҫ���õĿ��ƿ�
	*	@retval	error	0���������ݳɹ�	���������ݷ���ʧ��
	*/
uint8_t Motor_Speed_Mode(SERVO_MLCB *Servo_MLCB)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_SMOD_LEN;
	
	Servo_Structure.Front_Data[0] = ML_SMOD_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_SMOD;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = 0;
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}


/**
	*	@brief	�������ģʽ����	λ��(�Ƕ�)ģʽ
	*	@param	*Servo_MLCB		��Ҫ���õĿ��ƿ�
	*	@retval	error	0���������ݳɹ�	���������ݷ���ʧ��
	*/
uint8_t Motor_Position_Mode(SERVO_MLCB *Servo_MLCB)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_SMOD_LEN;
	
	Servo_Structure.Front_Data[0] = ML_SMOD_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_SMOD;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = 256;
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}


/**
	*	@brief	�����������ò���
	*	@param	*Servo_MLCB		��Ҫ���õĿ��ƿ�
	*	@retval	error	0���������ݳɹ�	���������ݷ���ʧ��
	*/
uint8_t Motor_Save_Param(SERVO_MLCB *Servo_MLCB)
{
	uint8_t error;
	Servo_Struct Servo_Structure;
	Servo_Structure.Data_Format = CAN_ID_STD;					//��׼֡��ʽ
	Servo_Structure.Data_Type   = CAN_RTR_DATA;							//����֡
	Servo_Structure.Servo_ID    = Servo_MLCB->Star_Id;
	Servo_Structure.Data_Len		= ML_ESA_LEN;
	
	Servo_Structure.Front_Data[0] = ML_ESA_LEN;
	Servo_Structure.Front_Data[1] = Servo_MLCB->Device_Id;
	Servo_Structure.Front_Data[2] = ML_ESA;
	Servo_Structure.Front_Data[3] = 0;
	
	Servo_Structure.Last_Data = 256;
		
	error = Servo_Set_Function(&Servo_Structure);
	return error;
}


/*************************************************����ٶ�����************************************************/
/*																								����ٶ�����																							 */
/*************************************************����ٶ�����************************************************/
/**
	*	@brief	�������ٶ�����
	*	@param	*Servo_MLCB	��Ҫ���Ƶĵ�����ƿ�
	*					data				�ٶȣ�ת/��
	*	@retval	error:	0	���ݷ��ͳɹ�	,���������ݷ���ʧ��
	*/






/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
