#include "follow.h"

/* External variables --------------------------------------------------------*/
volatile extern uint8_t Posi;            //ccd数据

extern uint8_t distance;        //激光数据
extern float R;                 //转弯半径，单位米
extern float tab_beta[101];     //角度表
extern float  y_speed;          //前进速度
extern double time;             //运行时间

/*----------------------------------------------------------------------------*/
/* Internal variables --------------------------------------------------------*/
pid_t x_pid;
pid_t w_pid;
pid_t y_pid;

volatile float theta,theta_next;         //车身角度，预测下一时刻车身角度
volatile float beta,beta_next;           //轨迹角度，预测下一时刻轨迹角度

/* ---------------------------------------------------------------------------*/

float follow_Mapan(int flag)
{
  
  if(flag == 2)
  {
    pid_calc(&x_pid,MaPan.y,MaPan.offset_y);
  }else if(flag == 4)
  {
    pid_calc(&x_pid,MaPan.x,MaPan.offset_x);
  }
  
  return x_pid.pos_out;
}




void get_angle_offset(angle * ptr,float angle)
{
	
	ptr->angle = angle;
	ptr->angle_offset = ptr->angle;
}
	  
void get_angle_total(volatile angle * ptr,float angle)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = angle;
	
	if(ptr->angle - ptr->last_angle > 180)
		ptr->angle_round --;
	else if (ptr->angle - ptr->last_angle < -180)
		ptr->angle_round ++;
	ptr->angle_total = (ptr->angle_round * 360 + ptr->angle - ptr->angle_offset) * 0.01745;
}
void x_pid_init(void)
{
	float p=0.006f;
	float i=0.0f;
	float d=0.0f;
	PID_struct_init(&x_pid, POSITION_PID, 200, 100,p,	i,d);  
  x_pid.deadband = 20;
	
}

void w_pid_init(void)
{
	float p=6.0f;
	float i=0.0f;
	float d=0.0f;
  
	PID_struct_init(&w_pid, POSITION_PID, 200, 100,p,	i,d);  
}

void y_pid_init(void)
{
	float p=1.0f;
	float i=0.0f;
	float d=0.0f;
	PID_struct_init(&y_pid, POSITION_PID, 200, 100,p,	i,d);  
}


float x_err(void)
{
	float err;
	float k=0.5f;
	
	if(pid_calc(&x_pid,Posi,64)!=0)
	{
		err=-k*x_pid.pos_out;
		return err;
	}
	else
		return 0;
}

float w_err(void)
{
	float err;
	float k=0.7f;
	
	if(pid_calc(&w_pid,Posi,64)!=0)
	{
		err=k*w_pid.pos_out;
		return err;
	}
	else
		return 0;
}


float stop_y(float y_speed)
{
   float stop_speed;
  
	 if(MaPan.y < (2000 - 1000 * R))
		{
			stop_speed=y_speed;
			if(pid_calc(&y_pid,MaPan.y,1000)!=0)
			{
				stop_speed = y_pid.pos_out;
				return stop_speed;
			}
			else
				return 0;
		}
		else
			return y_speed;

}
void turn_left(float r,float k)
{
	float w;
	float kk,b;
	uint16_t i = 0;
  float vx;                       //车x方向速度
  float vy;                       //车y方向速度 
	
	beta = 16/(3.1415926*3.1415926)*k*theta*theta-8/3.1415926*k*theta+theta; //弧度
	//求与V和R相关的w
	beta_next = (y_speed / (R* 1.1045) * 0.0159)* time + beta; //弧度  
	for( i = 0;i < 100;i++)
	{
		if(beta_next <= tab_beta[i])
		{
			break;
		}else
		{
			theta_next = tab_beta[99];
		}
	}
	if(i == 0)
	{
		theta_next = tab_beta[0];
	}else if(i == 100)
	{
			
	}
	if((i != 100)&&(i != 0))
	{
    kk = 0.0157f/(tab_beta[i+1] - tab_beta[i]);
    b  = 0.0157f * i - kk * tab_beta[i];
    theta_next = kk * beta_next + b;
	}
	
	w = 60 * (theta_next - theta )/ time;
	vx = y_speed * sin(theta-beta);
	vy = y_speed * cos(theta-beta);

	motor_move_setvmmps(vx,vy,w);
}


