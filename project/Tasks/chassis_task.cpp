/**
 *******************************************************************************
 * @file      :chassis_task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1。<note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis_task.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
M3508 motor_3508;

pid m3508_speed_pid;
pid m3508_position_pid;

float vx = 0;
float vy = 0;
float w = 0;

//TODO:底盘长宽
const float a = 0.5;
const float b = 0.5;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void chassis_init(void)
{
    //TODO:pid参数
    m3508_speed_pid.pid_init(0.15,0.01,0,300,1000);
    m3508_position_pid.pid_init(30,0,0,0,6000);
    motor_3508.set_speed(0,0,0,0,m3508_speed_pid);
}

void speed_cal(void)
{
    vx = (motor_3508.motor4_speed + motor_3508.motor3_speed)/2;
    vy = (motor_3508.motor3_speed - motor_3508.motor1_speed)/2;
    w = (motor_3508.motor2_speed - motor_3508.motor3_speed)/(2*(a+b));
}

void chassis_normal(float target_vx, float target_vy)
{
    // 麦轮姿态解算
    float wheel_speed[4];

    //TODO: 计算每个轮子的速度
    wheel_speed[0] = target_vy + target_vx;
    wheel_speed[1] = -target_vy + target_vx;
    wheel_speed[2] = -target_vy - target_vx;
    wheel_speed[3] = target_vy - target_vx;

   

    // 设置电机速度
    motor_3508.set_speed(wheel_speed[0] , wheel_speed[1] , wheel_speed[2] , wheel_speed[3] , m3508_speed_pid);
}

void chassis_cyro(float target_vx, float target_vy , float target_w)
{
    // 麦轮姿态解算
    float wheel_speed[4];

    //TODO: 计算每个轮子的速度
    wheel_speed[0] = target_vy + target_vx + target_w*(a+b);
    wheel_speed[1] = -target_vy + target_vx + target_w*(a+b);
    wheel_speed[2] = -target_vy - target_vx + target_w*(a+b);
    wheel_speed[3] = target_vy - target_vx + target_w*(a+b);

 
    // 设置电机速度
    motor_3508.set_speed(wheel_speed[0] , wheel_speed[1] , wheel_speed[2] , wheel_speed[3] , m3508_speed_pid);
}

void chassis_follow_gimbal(float vx , float vy ,float *chassis_euler_angles, float *gimbal_euler_angles )
{
    float target_temp = gimbal_euler_angles[0]/3.1415926*180 + 180;
    float chassis_tem = chassis_euler_angles[0]/3.1415926*180 + 180;
    float target_w;
    if(target_temp-chassis_tem>180)
    {
        m3508_position_pid.m_ref = target_temp-360;
        m3508_position_pid.m_fdb = chassis_tem;
        target_w = m3508_position_pid.pid_cal(0);
    }
    else
    {
        m3508_position_pid.m_ref = target_temp;
        m3508_position_pid.m_fdb = chassis_tem;
        target_w = m3508_position_pid.pid_cal(0);
    }
    chassis_cyro(vx,vy,target_w);
}

void m3508_test(void)
{
    motor_3508.set_speed(0,1500,1500,0,m3508_speed_pid);
}

