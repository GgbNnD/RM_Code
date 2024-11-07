/**
 *******************************************************************************
 * @file      :pid.hpp
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
#ifndef PID_HPP_
#define PID_HPP_
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdint.h"
 /* Exported macro ------------------------------------------------------------*/
 /* Exported types ------------------------------------------------------------*/
 class pid
 {
 public:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_i_max;
    float m_out_max;

    float m_ref;      // target value
    float m_fdb;      // feedback value
    float m_err[2];   // 0:error and 1:last error

    float m_p_out;
    float m_i_out;
    float m_d_out;
    float m_output;

    void pid_init(float kp,float ki,float kd,float i_max,float out_max);
    float pid_cal(uint32_t isi_isolate);

 };
#endif /* PID_HPP_ */
