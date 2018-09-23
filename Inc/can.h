/**
  ******************************************************************************
  * File Name          : CAN.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "gpio.h"

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
	 
	 /******���̵��********/
//---���1
#define DEVICE1_ID	    10
#define RATIO1			15
#define CODER1			1000
//---���2	
#define DEVICE2_ID  	11
#define RATIO2			15
#define CODER2			1000
//---���3	
#define DEVICE3_ID	    16
#define RATIO3			15
#define CODER3			1000
//---���4	
#define DEVICE4_ID	    17
#define RATIO4			15
#define CODER4			1000
//---���5	
#define DEVICE5_ID	    12
#define RATIO5			16
#define CODER5			1000
//---���6	
#define DEVICE6_ID	    9
#define RATIO6			27
#define CODER6			1000

/*******************************���ʿ�����ָ�*********************************/

/////-------------------------д�����·�����������----------------------------------//////
/*	��ȡ��������Ʒ��Ϣָ��	*/
#define	ML_SADR 				0x04 	//	����������ID��ַ����ֵ��Χ��2~127
#define ML_SADR_LEN			0X06	//

/*	����CANͨѶ������	*/
#define ML_SCBD					0X07	//	������������CANͨѶ������
#define ML_SCBD_LEN			0x08

/**	@brief	������������������	*/
#define ML_SSP					0X16	//	���õ������������ٶȣ��������κ�ģʽ
#define ML_SSP_LEN			0X08	//	

#define ML_SMV					0X18	//	��������ٶȣ����趨�ٶ�С����С�ٶ�ʱ��������ƶ�
#define ML_SMV_LEN			0X06

#define ML_SPC					0X1A	//	���õ���ķ�ֵ��������ֵ��Χ��0~40000����
#define ML_SPC_LEN			0X08	//

#define ML_SCC					0X1C	//	���õ������������ֵ����ֵ��Χ��0~20000����
#define ML_SCC_LEN			0X08	//

#define ML_ENC					0X1E	//	���ñ������ķֱ���,����Ϊ������ʵ��������4��
#define	ML_ENC_LEN			0X08	//

/**	@brief	����������ģʽ	*/
#define	ML_SMOD					0X2A	//	���õ�ǰ�Ĺ���ģʽ���ź�Դ
#define	ML_SMOD_LEN			0X06	//

/**	@brief	������������ģʽ���������趨	*/
#define ML_SMAV					0X31	//	����������ѹ����ֵ��Χ��0~1000mv
#define ML_SMAV_LEN			0X06	//

#define	ML_SPH					0X33	//	���������λ��λ�����ޣ�����������λ�÷�Χ��0~2100000000
#define ML_SPH_LEN			0X08	//

#define ML_SPL					0X35	//	���������λ��λ������
#define	ML_SPL_LEN			0X08	//

#define	ML_SPE					0X3F	//	���������λ��1�����������λ��0��ȡ�������λ
#define ML_SPE_LEN			0X06	//

/**	@brief	PID��������ָ��	*/
#define ML_A						0X58	//	���ü��ٶȣ���ֵ��Χ��1~30000
#define ML_A_LEN				0X06	//

#define	ML_AP						0X5A	//	���õ���������ϵ��
#define ML_AP_LEN				0X06	//

#define	ML_AI						0X5C	//	���õ���������ϵ��
#define ML_AI_LEN				0X06	//

#define ML_AD						0X5E	//	���õ�����΢��ϵ��
#define ML_AD_LEN				0X06	//

#define	ML_P						0X60	//	�����ٶȻ�����ϵ��
#define ML_P_LEN				0X06	//	

#define ML_I						0X62	//	�����ٶȻ�����ϵ��
#define	ML_I_LEN				0X06	//

#define ML_D						0X64	//	�����ٶȻ�΢��ϵ��
#define	ML_D_LEN				0X06	//

#define ML_MP						0X66	//	����λ�û�����ϵ��
#define ML_MP_LEN				0X06	//

#define ML_MI						0X6A	//	����λ�û�����ϵ��
#define ML_MI_LEN				0X06	//

#define ML_MD						0X6C	//	����λ��΢��ϵ��
#define	ML_MD_LEN				0X06	//

/**	@brief	���ÿ���Ѱ����λָ��	*/
#define	ML_SPOF					0X75	//	����Ѱ����λ��־	1������Ѱ����λ	1��������Ѱ����λ
#define	ML_SPOF_LEN			0X06	//

/**	@brief	������ϵͳ����ָ��	*/
#define	ML_ESA					0X82	//	��������
#define	ML_ESA_LEN			0X06	//

#define	ML_SBS					0X84	//	�����ͣ�����κ�ģʽ��Ӧ�ø�ָ���������������ƶ����
#define	ML_SBS_LEN			0X06	//

#define	ML_CBS					0X85	//	��������ͣ
#define	ML_GBS_LEN			0X06	//

/**	@breif	������������ʽ	*/
#define	ML_SSFT					0X8E	//	���÷�����ʽ	0�������������� 1�����ٻ�����
#define	ML_SSFT_LEN			0X06	//

#define	ML_V						0X90	//	�趨�����ٶ�
#define	ML_V_LEN				0X08	//

/**	@brief	����ģʽ����	*/
#define	ML_EC						0X96	//	����Ŀ�����
#define	ML_EC_LEN				0X08	//

/**	@brief	λ�ÿ���ָ��	*/
#define	ML_PO						0X98	//	���þ���λ��,���õ����ǰλ��Ϊ����λ��	�ڵ��ֹͣ״̬���û�ȽϾ�ȷ
#define ML_PO_LEN				0X08	//

#define	ML_M						0X99	//	���þ��Զ�λ��	�����ʼ���趨��λ���˶�
#define	ML_M_LEN				0X08	//

#define	ML_MR						0X9A	//	�������λ��	������ϴ�����λ���趨���ǵ��ʵʱλ��	�����ʼ���趨λ���˶�
#define	ML_MR_LEN				0X08	//



/////-------------------------�ӿ������ж�����----------------------------------//////

/*	��ȡ��������Ʒ��Ϣָ��	*/
#define	ML_GADR					0X05	//	��ȡ������ID��ַ
#define	ML_GADR_LEN			0X06	//	

/**	@brief	������������������	*/
#define	ML_GSP					0X17	//	��ȡ���õĵ����������ٶ�
#define	ML_GSP_LEN			0X08	//

#define	ML_GMV					0X19	//	��ȡ����ٶ�ֵ
#define	ML_GMV_LEN			0X06	//

#define	ML_GPC					0X1B	//	��ȡ���õķ�ֵ����
#define	ML_GPC_LEN			0X08	//

#define	ML_GCC					0X1D	//	��ȡ���õ���������ֵ
#define	ML_GCC_LEN			0X08	//

#define	ML_GENC					0X1F	//	��ȡ�������ֱ���
#define	ML_GENC_LEN			0X08	//

/**	@brief	����������ģʽ	*/
#define	ML_GMOD					0X2B	//	��ȡ��ǰ�Ĺ���ģʽ���ź�Դ
#define	ML_GMOD_LEN			0X06	//

/**	@brief	������������ģʽ���������趨	*/
#define	ML_GMAV					0X32	//	��ȡ������ѹֵ
#define	ML_GMAV_LEN			0X06	//

#define	ML_GPH					0X34	//	��ȡ�����λ��λ������
#define	ML_GPH_LEN			0X08	//

#define	ML_GPL					0X36	//	��ȡ�����λ��λ������
#define	ML_GPL_LEN			0X08	//

#define	ML_GPE					0X40	//	��ȡ�����λ״̬
#define	ML_GPE_LEN			0X06	//

/**	@brief	PID��������ָ��	*/
#define	ML_GA						0X59	//	��ȡ������ٶ�ֵ
#define	ML_GA_LEN				0X06	//

#define	ML_GAP					0X5B	//	��ȡ����������ϵ��
#define	ML_GPA_LEN			0X06	//

#define	ML_GAI					0X5D	//	��ȡ����������ϵ��
#define	ML_GAI_LEN			0X06	//

#define	ML_GAD					0X5F	//	��ȡ������΢��ϵ��
#define	ML_GAD_LEN			0X06	//	

#define	ML_GP						0X61	//	��ȡ�ٶȻ�����ϵ��
#define	ML_GP_LEN				0X06	//

#define	ML_GI						0X63	//	��ȡ�ٶȻ�����ϵ��
#define	ML_GI_LEN				0X06	//

#define	ML_GD						0X65	//	��ȡ�ٶȻ�΢��ϵ��
#define	ML_GD_LEN				0X06	//

#define	ML_GMP					0X67	//	��ȡλ�û�����ϵ��
#define	ML_GMP_LEN			0X06	//

#define	ML_GMI					0X6A	//	��ȡλ�û�����ϵ��
#define	ML_GMI_LEN			0X06	//

#define	ML_GMD					0X6D	//	��ȡλ�û�΢��ϵ��
#define	ML_GMD_LEN			0X06	//

/**	@brief	���ÿ���Ѱ����λָ��	*/
#define	ML_GPOF					0X76	//	��ȡѰ����λ��־	1:����Ѱ����λ	0��������Ѱ����λ
#define	ML_GPOF_LEN			0X06	//	

#define	ML_GOV					0X78	//	��ȡѰ����λ�ٶ�
#define	ML_GOV_LEN			0X08	//

/**	@breif	������������ʽ	*/
#define	ML_GV						0X91	//	��ȡƽ���ٶ�
#define	ML_GV_LEN				0X04	//

#define	ML_GSV					0X93	//	��ȡ���õ��ٶ�
#define	ML_GSV_LEN			0X08	//

/**	@brief	λ�ÿ���ָ��	*/
#define	ML_GM						0X9B	//	��ȡʵ��λ��
#define	ML_GM_LEN				0X08	//	

#define	ML_GC						0XD0	//	��ȡ����ֵ
#define	ML_GC_LEN				0X06	//	



typedef enum
{
	Write_Success=0,
}SERVO_ERROR;

/********�����ŷ�������ƿ�********/
typedef struct
{
	char		   *Servo_Mlcb_Name;			//�ŷ����ƿ�����
	uint8_t     Star_Id;							//��ʶ��ID
	uint8_t     Device_Id;						//�豸ID
	uint8_t     Ratio;								//���������
	uint16_t    Coder;								//����������
	SERVO_ERROR Servo_Error;					//�������
}SERVO_MLCB;

typedef struct
{
	unsigned Servo_ID:7;
	unsigned Data_Len:4;
	unsigned Data_Format:2;
	unsigned Data_Type:2;
	uint8_t  Front_Data[4];//һ������������ǰ���ֽ�����
	int32_t  Last_Data;
}Servo_Struct;


typedef struct 
{
	  uint8_t     CAN_TX_Data[8];
    CAN_TxHeaderTypeDef  SendCanTxMsg;       //�������� 
}CanSendData_Type;

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Start(void);
uint8_t Servo_Set_Function(Servo_Struct *Servo_Structure);
uint8_t Servo_Param_Set( uint8_t servo_id,uint8_t order,uint8_t len,int32_t data);

void Create_MLServo_CB(SERVO_MLCB		*servo_mlcb,					//�ŷ����ƿ�
											 char       	*servo_mlcb_name,			//�ŷ����ƿ�����
											 uint8_t     	 star_id,							//��ʶ��ID
											 uint8_t       ratio,								//���������
											 uint16_t      coder);							//����������

uint8_t MLServo_Write(SERVO_MLCB *Servo_MLCB, uint8_t order, uint8_t len, int32_t data);

/*****************************************��ؼ�Ҫ���*********************************************/
                                                                                          
/*  ���״̬����    */
                                             
/*	����ٶ����	*/
uint8_t Motor_MastSpeed(SERVO_MLCB *Servo_MLCB,int32_t data);	//	�������ٶ�����
uint8_t Motor_MinSpeed(SERVO_MLCB *Servo_MLCB,int32_t data);	//	�����С�ٶ�����
uint8_t Motor_SetSpeed(SERVO_MLCB *Servo_MLCB,float data);	//	��������ٶ�����
                                             
/*	���ٶ�����	*/
uint8_t Motor_AccelSpeed(SERVO_MLCB *Servo_MLCB,int32_t data);	//���õ�����ٶ�

/*	���λ������	*/
uint8_t Motor_AbsolutePlace(SERVO_MLCB *Servo_MLCB,int32_t data);	//���õ����ǰλ��Ϊ����λ��
uint8_t Motor_ToAbsolutePlace(SERVO_MLCB *Servo_MLCB,int32_t data);	//���õ�������λ�ÿ�ʼ����
uint8_t Motor_ToRelativePlace(SERVO_MLCB *Servo_MLCB,int32_t data);	//���õ��������ϴ�����λ�ÿ�ʼ����

/*	�����ʼ������	*/
uint8_t Motor_P_SET(SERVO_MLCB *Servo_MLCB,int32_t data);	//���λ�ñ���P����
uint8_t Motor_Code_Set(SERVO_MLCB *Servo_MLCB,int32_t data);	//����������ֱ�������
uint8_t Motor_Speed_Mode(SERVO_MLCB *Servo_MLCB);						//���õ��Ϊ�ٶ�ģʽ
uint8_t Motor_Position_Mode(SERVO_MLCB *Servo_MLCB);					//���õ��Ϊλ��(�Ƕ�)ģʽ
uint8_t Motor_Save_Param(SERVO_MLCB *Servo_MLCB);					//�������ϲ���

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
