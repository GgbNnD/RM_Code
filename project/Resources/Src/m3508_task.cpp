/**
 *******************************************************************************
 * @file      :m3508_task.cpp
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
#include "m3508_task.hpp"
#include "HW_can.hpp"
#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void M3508::set_speed(float target_speed1 , float target_speed2 , float target_speed3 , float target_speed4 , pid &PID)
{
    uint8_t data[8];
    //TODO:速度与电流的关系
    PID.m_ref = target_speed1;
    PID.m_fdb = motor1_speed;
    int16_t motor1_current_set = (int16_t) (PID.pid_cal(0) / 1000 * 16384);

    PID.m_ref = target_speed2;
    PID.m_fdb = motor2_speed;
    int16_t motor2_current_set = (int16_t) (PID.pid_cal(0) / 1000 * 16384);

    PID.m_ref = target_speed3;
    PID.m_fdb = motor3_speed;
    int16_t motor3_current_set = (int16_t) (PID.pid_cal(0) / 1000 * 16384);

    PID.m_ref = target_speed4;
    PID.m_fdb = motor4_speed;
    int16_t motor4_current_set = (int16_t) (PID.pid_cal(0) / 1000 * 16384);

    data[0] = (uint8_t)(motor1_current_set >> 8);
    data[1] = (uint8_t)(motor1_current_set);
    data[2] = (uint8_t)(motor2_current_set >> 8);
    data[3] = (uint8_t)(motor2_current_set);
    data[4] = (uint8_t)(motor3_current_set >> 8);
    data[5] = (uint8_t)(motor3_current_set);
    data[6] = (uint8_t)(motor4_current_set >> 8);
    data[7] = (uint8_t)(motor4_current_set);

    CAN_Send_Msg(&hcan2, data, 0x200, 8);
}




