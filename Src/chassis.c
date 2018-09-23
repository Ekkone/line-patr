/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "chassis.h"
#define ARC 0.05

/* External variables --------------------------------------------------------*/

volatile extern uint8_t Posi;            //ccd数据  
volatile extern float  beta;             //轨迹角度

extern uint32_t distance;                //激光数据
extern uint8_t turn_flag;                //转弯标志
extern uint8_t Width;		             //宽度
extern uint8_t Time;			             //曝光时间

/*----------------------------------------------------------------------------*/
/* Internal variables --------------------------------------------------------*/
SERVO_MLCB UNDER1_MLCB;			//第一象限电机
SERVO_MLCB UNDER2_MLCB;			//第二象限电机
SERVO_MLCB UNDER3_MLCB;			//第三象限电机
SERVO_MLCB UNDER4_MLCB;			//第四象限电机
const float motor_move_max_rpm = 465.f;  //移动电机最大速度、加速度限制
const float motor_move_acc = 200.f;

float y_speed = 160;               //前进速度
float R = 1.0f;                    //转弯半径，单位米
float tab_beta[101] = {0};         //角度表
float theta_ergodic = 0;           //遍历角度
double time_now,time_last,time;    //运行时间
/* ---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
float ABS_float(float param)
{
	if((double)param>0)
		return param;
	else
		return -param;
}



void motor_Init(void)
{
	/*	底盘电机控制块设置	*/
	Create_MLServo_CB(	 (SERVO_MLCB	*)&UNDER1_MLCB,					
						 (char   	*)"underpan_motor1",			
						 (uint8_t      )DEVICE1_ID,											
						 (uint8_t      )RATIO1,								
						 (uint16_t     )CODER1);
	
	Create_MLServo_CB(	 (SERVO_MLCB	*)&UNDER2_MLCB,					
						 (char   	*)"underpan_motor2",			
						 (uint8_t      )DEVICE2_ID,											
						 (uint8_t      )RATIO2,								
						 (uint16_t     )CODER2);
	
	Create_MLServo_CB(	 (SERVO_MLCB	*)&UNDER3_MLCB,					
						 (char   	*)"underpan_motor3",			
						 (uint8_t      )DEVICE3_ID,											
						 (uint8_t      )RATIO3,								
						 (uint16_t     )CODER3);
	
	Create_MLServo_CB(	 (SERVO_MLCB	*)&UNDER4_MLCB,					
						 (char   	*)"underpan_motor4",			
						 (uint8_t      )DEVICE4_ID,											
						 (uint8_t      )RATIO4,								
						 (uint16_t     )CODER4);  
						 	Motor_Speed_Mode(&UNDER4_MLCB);
	Motor_Speed_Mode(&UNDER1_MLCB);
	HAL_Delay(10);  
	Motor_Speed_Mode(&UNDER2_MLCB);
	HAL_Delay(10);  
	Motor_Speed_Mode(&UNDER3_MLCB);
	HAL_Delay(10);  
	Motor_Speed_Mode(&UNDER4_MLCB);
	HAL_Delay(10);  
 }

void motor_move_setvmmps(float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W)
{
    uint8_t i;
    float wheel[4];
    float max=0, rate;
    
	wheel[0] = (-dstVmmps_X + dstVmmps_Y + dstVmmps_W);
	wheel[1] = (-(dstVmmps_X + dstVmmps_Y - dstVmmps_W));
	wheel[2] = (-(-dstVmmps_X + dstVmmps_Y - dstVmmps_W));   
	wheel[3] = (dstVmmps_X + dstVmmps_Y + dstVmmps_W);    
    //find max item
    for(i= 0; i<4; i++)
    {
        if( ABS_float(wheel[i]) > max)
        {
            max = ABS_float(wheel[i]);        
        }            
    }
	//equal proportion      
    if( max > motor_move_max_rpm)
    {      
        rate = motor_move_max_rpm / max;        
        for(i= 0; i < 4; i++)
            wheel[i] *= rate;
    }       
    Motor_SetSpeed(&UNDER1_MLCB,wheel[0]);
	  	HAL_Delay(10);  
    Motor_SetSpeed(&UNDER2_MLCB,wheel[1]);	
	  	HAL_Delay(10);  
    Motor_SetSpeed(&UNDER3_MLCB,wheel[2]);
			HAL_Delay(10);  
    Motor_SetSpeed(&UNDER4_MLCB,wheel[3]);
			HAL_Delay(10);  

}

//直接设置四个电机的转速
void SetCMSpeed(float c201, float c202, float c203, float c204)
{
	uint8_t i;
	float max=0, rate;
	float wheel[4];
	
	wheel[0]=c201;
	wheel[1]=c202;
	wheel[2]=c203;
	wheel[3]=c204;
	
	//find max item
    for(i= 0; i<4; i++)
    {
        if( ABS_float(wheel[i]) > max)
        {
            max = ABS_float(wheel[i]);        
        }            
    }
		
			//equal proportion      
    if( max > motor_move_max_rpm)
    {   
        rate = motor_move_max_rpm / max;        			
        for(i= 0; i < 4; i++)
            wheel[i] *= rate;
    }    
	Motor_SetSpeed(&UNDER1_MLCB,c201 );
	HAL_Delay(10);  
	Motor_SetSpeed(&UNDER2_MLCB,c202 );
	HAL_Delay(10);  
	Motor_SetSpeed(&UNDER3_MLCB,c203 );
	HAL_Delay(10);  
	Motor_SetSpeed(&UNDER4_MLCB,c204 );
	HAL_Delay(10);  
}


void Chassis_Contrl_Task(void const * argument)
{
  osDelay(3000);
	x_pid_init();
	w_pid_init();
	y_pid_init();
	motor_Init();
  
	uint8_t step = 0;
	float L;

	for(uint16_t i = 0;i < 101;i++)
	{
		tab_beta[i] = 16 / (3.1415926 * 3.1415926)* ARC *theta_ergodic * \
                  theta_ergodic - 8/3.1415926 * ARC * theta_ergodic + theta_ergodic;
		theta_ergodic = theta_ergodic + 0.0157;
	}

	
	
	while(1)
	{
		
		time_last = time_now;
		time_now = HAL_GetTick() * 0.001;
		time = time_now - time_last;
		
		switch(step)
		{
			case 0:
				L = 3000 - 1000 * R;    //转弯点距出发点的距离（y）
			  break;
			case 2:
				L = -(2650 - 1000 * R); //转弯点据出发点的距离（x）
				break;
		}	

		switch(step)
		{
			case 0:
			{
				motor_move_setvmmps(0,y_speed,w_err());
			  if(MaPan.y > L)
				{
					step++;
					turn_flag++;
				}
			}break;
			case 1:
			{
				turn_left(R,ARC);
			  if(beta > 1.5)
					{
						step++;
					}
				}break;
			case 2:
			{
        if(MaPan.x > -(R * 1000.f + 400))
        {
            motor_move_setvmmps(-follow_Mapan(step),y_speed,0);
        }
        else
				motor_move_setvmmps(0,y_speed,w_err());
			  if(MaPan.x < L)
				{
					step++;
					turn_flag++;
				}
			}break;
			case 3:
			{
				turn_left(R,ARC);
			  if(beta > 1.5)
					{
						step++;
					}
			}break;
			case 4:
        if(MaPan.y > R * 1000.f + 400)
        {
            motor_move_setvmmps(-follow_Mapan(step),y_speed,0);
        }
        else
	 			motor_move_setvmmps(0,stop_y(y_speed),w_err());
				break;
		}	
		
	  //	printf("total:%f\t\tangle:%f\tlast%f\toffset:%f\tround%f\r\n",yaw.angle_total,yaw.angle,yaw.last_angle,yaw.angle_offset,yaw.angle_round);
    //  printf("step=%d\tx=%f\ty=%f\tPosi=%d\r\n",step,MaPan.x,MaPan.y,Posi);
		  printf("Posi=%d\tWidth=%d\ttime=%d\r\n",Posi,Width,Time);
		osDelay(5);
  }
	
	
	
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
