/**
 *******************************************************************************
 * @file      :chassis_task.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_TASK_HPP_
#define CHASSIS_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "dm4310_drv.h"
#include "m3508_task.hpp"
#include "pid.hpp"
 /* Exported macro ------------------------------------------------------------*/
 /* Exported types ------------------------------------------------------------*/
extern M3508 motor_3508;
extern pid m3508_speed_pid;
extern pid m3508_position_pid;

void chassis_init(void);
void chassis_normal(float target_vx, float target_vy);
void chassis_cyro(float target_vx, float target_vy , float target_w);
//void chassis_follow_gimbal(float vx , float vy ,float *chassis_euler_angles, float *gimbal_euler_angles );
void chassis_follow_gimbal(float vx , float vy ,Joint_Motor_t *motor);
void m3508_test(void);
void gimbalbased_chassis_move(float vx, float vy, float w, Joint_Motor_t *motor);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* CHASSIS_TASK_HPP_ */
