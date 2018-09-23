#ifndef FOLLOW_H
#define FOLLOW_H
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "can.h"
//#include "chassis.h"
#include "math.h"
typedef struct {
		
		float angle;
		float last_angle;
		float angle_round;
		float angle_total;
		float angle_offset;
		float angle_num;
	}angle;

typedef struct {

    volatile float	x;		           //码盘传回的xy和角度
    volatile float 	y;
    volatile float 	theta;

    float		offset_x;		     //码盘传回的第一次xy和角度
    float 	offset_y;
    float 	offset_theta;
}mapan;

extern mapan MaPan;
extern volatile angle yaw;

void x_pid_init(void);
void w_pid_init(void);
void y_pid_init(void);
float x_err(void);
float w_err(void);
float stop_y(float y_speed);
void turn_left(float r,float k);
void get_angle_offset(angle * ptr,float angle);
void get_angle_total(volatile angle * ptr,float angle);
float follow_Mapan(int flag);
  
#endif

