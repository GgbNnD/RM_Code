/**
 *******************************************************************************
 * @file      :m3508_task.hpp
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
#ifndef M3508_TASK_HPP_
#define M3508_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "main.h"
#include "pid.hpp"
 /* Exported macro ------------------------------------------------------------*/
 /* Exported types ------------------------------------------------------------*/
 class M3508
 {
    public:
        int16_t motor1_speed;
        float motor1_angle;
        int16_t  motor1_current;
        int16_t  motor1_temperature;

        int16_t motor2_speed;
        float motor2_angle;
        int16_t  motor2_current;
        int16_t  motor2_temperature;

        int16_t motor3_speed;
        float motor3_angle;
        int16_t  motor3_current;
        int16_t  motor3_temperature;

        int16_t motor4_speed;
        float motor4_angle;
        int16_t  motor4_current;
        int16_t  motor4_temperature;

        void set_speed(float target_speed1 , float target_speed2 , float target_speed3 , float target_speed4 , pid &PID);
 };

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PROJECT_PATH_FILE_HPP_ */
