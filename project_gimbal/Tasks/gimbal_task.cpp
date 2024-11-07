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
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Joint_Motor_t motor_dm4310;
/* Private function prototypes -----------------------------------------------*/
void gimbal_init(void)
{
    dm4310_init(&motor_dm4310,MIT_MODE,3);
}

void gimbal_set_speed(float target_speed)
{
    mit_ctrl(&hcan1,3,0,target_speed,0,1,0);
}

