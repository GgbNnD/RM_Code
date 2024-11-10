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
#include "HW_can.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Joint_Motor_t motor_dm4310;
pid gimbal_position_pid;
pid gimbal_tor_pid;

//TODO:pitch角度限制
const float pitch_max = 29;
const float pitch_min = -24;
/* Private function prototypes -----------------------------------------------*/
void gimbal_init(void)
{
    dm4310_init(&motor_dm4310,MIT_MODE,3);
    gimbal_position_pid.pid_init(0.6,0.01,0,0,6);
    gimbal_tor_pid.pid_init(1,0,0,2,6);
    mit_ctrl(&hcan1,3,0,0,0,1,0);
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
//    mit_ctrl(&hcan1,3,0,target_w,0,1,0);
    gimbal_tor_control(target_w);
}

void gimbal_tor_control(float target_speed)
{
    gimbal_tor_pid.m_ref = target_speed;
    gimbal_tor_pid.m_fdb = motor_dm4310.para.vel;
    float target_tor = gimbal_tor_pid.pid_cal(0);
    mit_ctrl(&hcan1,3,0,0,0,0,target_tor);
}

void gimbal_tor_test(float target_speed)
{
    gimbal_tor_control(target_speed);
}

void gimbal_pitch_contrl(float *pitch)
{

    if(*pitch > pitch_max)
    {
        *pitch = pitch_max;
    }
    else if(*pitch < pitch_min)
    {
        *pitch = pitch_min;
    }
    uint8_t data[8];
    data[0] = (int16_t)(*pitch*500)>>8;
    data[1] = (int16_t)(*pitch*500);
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    CAN_Send_Msg(&hcan1,data,0x60,8);
}

