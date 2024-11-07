/**
 *******************************************************************************
 * @file      :gimbal_task.hpp
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
#ifndef GIMBAL_TASK_HPP_
#define GIMBAL_TASK_HPP_

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.hpp"
#include "dm4310_drv.h"
 /* Exported macro ------------------------------------------------------------*/
 /* Exported types ------------------------------------------------------------*/
extern Joint_Motor_t motor_dm4310;
void gimbal_init(void);
#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_TASK_HPP_ */
