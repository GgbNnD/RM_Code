/**
 *******************************************************************************
 * @file      :pid.cpp
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
#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief      pid初始化
 * @param       kp:
 * @param       ki:
 * @param       kd:
 * @param       i_max:
 * @param       out_max:
 *   @arg       None
 * @retval      None
 * @note        None
 */
void pid::pid_init(float kp,float ki,float kd,float i_max,float out_max)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_i_max = i_max;
    m_out_max = out_max;
}

/**
 * @brief      pid计算
 *   @arg       None
 * @retval      None
 * @note        None
 */
float pid::pid_cal(uint32_t isi_isolate)
{
    m_err[0] = m_ref - m_fdb;
    m_p_out = m_kp * m_err[0];

    //pwm位置pid时使用
    if(isi_isolate ==1)
    {
        if(m_err[0]>-0.3*m_ref && m_err[0]<0.3*m_ref)
        {
            m_i_out += m_ki * m_err[0];
        }
    }

    //can速度pid时使用
    if(isi_isolate ==0 )
    {
        m_i_out += m_ki * m_err[0];
    }
    m_d_out = m_kd * (m_err[0] - m_err[1]);
    
    LIMIT_MIN_MAX(m_i_out,-m_i_max,m_i_max);
    m_output = m_p_out + m_i_out + m_d_out;
    LIMIT_MIN_MAX(m_output,-m_out_max,m_out_max);
    m_err[1] = m_err[0];
    return m_output;
}

