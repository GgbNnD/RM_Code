/**
 *******************************************************************************
 * @file      :gimbal_task.cpp
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
#include "gimbal_task.hpp"
#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Joint_Motor_t motor_dm4310;
pid gimbal_position_pid;
pid gimbal_tor_pid;
/* Private function prototypes -----------------------------------------------*/
void gimbal_init(void)
{
    dm4310_init(&motor_dm4310,MIT_MODE,3);
    gimbal_position_pid.pid_init(0.5,0.01,0,0,6);
    gimbal_tor_pid.pid_init(0.5,0.01,0,0,6);
    mit_ctrl(&hcan1,3,0,0,0,0,0);
}

void gimbal_set_speed(float target_speed)
{
    mit_ctrl(&hcan1,3,0,target_speed,0,1,0);
}

void gimbal_set_position(float target_position ,float *euler_angles)
{
    if(target_position>0)
    {
        target_position = target_position-(int16_t)(target_position/360)*360;
    }
    else
    {
        target_position = target_position-(int16_t)(target_position/360)*360 + 360;
    }
    
    float target_angle = target_position;
    float gimbal_angle = euler_angles[0]/3.1415926*180 + 180;
    float target_w;
//    if(target_angle-gimbal_angle>180)
//    {
//        gimbal_position_pid.m_ref = target_angle-360;
//        gimbal_position_pid.m_fdb = gimbal_angle;
//        target_w = gimbal_position_pid.pid_cal(0);
//    }
//    else
//    {
//        gimbal_position_pid.m_ref = target_angle;
//        gimbal_position_pid.m_fdb = gimbal_angle;
//        target_w = gimbal_position_pid.pid_cal(0);
//    }
    if(gimbal_angle<180)
    {
        if(target_angle<gimbal_angle+180 && target_angle>gimbal_angle)
        {
            gimbal_position_pid.m_ref = target_angle;
            gimbal_position_pid.m_fdb = gimbal_angle;
            target_w = gimbal_position_pid.pid_cal(0);
        }
        else
        {
            if(target_angle > gimbal_angle)
            {
                gimbal_position_pid.m_ref = target_angle-360;
                gimbal_position_pid.m_fdb = gimbal_angle;
                target_w = gimbal_position_pid.pid_cal(0);
            }
            else
            {
                gimbal_position_pid.m_ref = target_angle;
                gimbal_position_pid.m_fdb = gimbal_angle;
                target_w = gimbal_position_pid.pid_cal(0);
            }
        }
    }
    else
    {
        if(target_angle > gimbal_angle -180 && target_angle <gimbal_angle)
        {
            gimbal_position_pid.m_ref = target_angle;
            gimbal_position_pid.m_fdb = gimbal_angle;
            target_w = gimbal_position_pid.pid_cal(0);
        }
        else
        {
            if(target_angle > gimbal_angle)
            {
                gimbal_position_pid.m_ref = target_angle;
                gimbal_position_pid.m_fdb = gimbal_angle;
                target_w = gimbal_position_pid.pid_cal(0);
            }
            else
            {
                gimbal_position_pid.m_ref = target_angle+360;
                gimbal_position_pid.m_fdb = gimbal_angle;
                target_w = gimbal_position_pid.pid_cal(0);
            }
        }
    }
    mit_ctrl(&hcan1,3,0,target_w,0,1,0);
}

void gimbal_tor_control(float target_position ,float *euler_angles)
{
    
}

