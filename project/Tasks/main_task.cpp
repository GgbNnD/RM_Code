/**
*******************************************************************************
 * @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "HW_can.hpp"
#include "math.h"
#include "DT7.hpp"

#include "m3508_task.hpp"
#include "imu_task.hpp"
#include "chassis_task.hpp"
#include "gimbal_task.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern float euler_angles[3];
extern float gimbal_euler_angles[3];

float gimbal_angle = 0;
float gimbal_pitch = 0;

float gimbal_imu_angle = 0;
float gimbal_imu_angleplus = 0;
float chassis_imu_angle = 0;
float chassis_imu_angleplus = 0;

/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr;

void RobotInit(void)
{
    rc_ptr = new remote_control::DT7();
}

void MainInit(void)
{
    RobotInit();


    //开启CAN1和CAN2
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

    //开启遥控器接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);

    //陀螺仪初始化
    ImuInit();

    //云台初始化
    gimbal_init();

    //底盘初始化
    chassis_init();

    //开启定时器
    HAL_TIM_Base_Start_IT(&htim6);

}

void MainTask(void)
{
    tick++;
    if(rc_ptr->rc_r_switch()==remote_control::kSwitchStateUp)
    {
        chassis_normal(0,0);
        gimbal_set_speed(0);
        gimbal_pitch = 60;
        gimbal_pitch_contrl(&gimbal_pitch);
        gimbal_pitch = 0;
    }
    else{
        ImuUpdate();
    if(tick == 1)
    {
        gimbal_imu_angle = gimbal_euler_angles[0];
        chassis_imu_angle = euler_angles[0];
    }
    else if(tick > 1 && tick < 1000)
    {
        gimbal_imu_angleplus += gimbal_euler_angles[0] - gimbal_imu_angle;
        chassis_imu_angleplus += euler_angles[0] - chassis_imu_angle;
    }
    else if(tick == 1000)
    {
        gimbal_imu_angleplus = gimbal_imu_angleplus/998;
        chassis_imu_angleplus = chassis_imu_angleplus/998;
        gimbal_angle = (gimbal_euler_angles[0]-gimbal_imu_angleplus)/3.1415926*180 + 180;
    }
    else if(tick > 1000)
    {
        euler_angles[0] = euler_angles[0] - chassis_imu_angleplus;
        if(euler_angles[0] < -3.1415926)
        {
            euler_angles[0] = euler_angles[0] + 2*3.1415926;
        }
        gimbal_euler_angles[0] = gimbal_euler_angles[0] - gimbal_imu_angleplus;
        if(gimbal_euler_angles[0] < -3.1415926)
        {
            gimbal_euler_angles[0] = gimbal_euler_angles[0] + 2*3.1415926;
        }
//        gimbal_tor_test(5);
        gimbal_angle -= rc_ptr->rc_lh()*0.1;
        gimbal_set_position(gimbal_angle , gimbal_euler_angles);
        gimbal_pitch -= rc_ptr->rc_lv()*0.03;
        gimbal_pitch_contrl(&gimbal_pitch);
    //TODO:测试模式转变
        if(rc_ptr->rc_l_switch() == remote_control::kSwitchStateUp)
        {
            float vx = rc_ptr->rc_rv()*2000;
            float vy = rc_ptr->rc_rh()*3000;
            gimbalbased_chassis_move(vx,vy,0,&motor_dm4310);
            
        }
        else if(rc_ptr->rc_l_switch() == remote_control::kSwitchStateMid)
        {
            float vx = rc_ptr->rc_rv()*2000;
            float vy = rc_ptr->rc_rh()*3000;
            chassis_follow_gimbal(vx,vy,&motor_dm4310);
            
        }
       else if(rc_ptr->rc_l_switch() == remote_control::kSwitchStateDown)
       {
           float vx = rc_ptr->rc_rv()*2000;
           float vy = rc_ptr->rc_rh()*3000;
           gimbalbased_chassis_move(vx,vy,2500,&motor_dm4310);
           
       }


    }
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == &htim6)
    {
        MainTask();
    }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        if (Size == remote_control::kRcRxDataLen)
        {
            //TODO:看门狗测试，在控制代码打断点，关闭遥控时不会达到断点
            HAL_IWDG_Refresh(&hiwdg);

            rc_ptr->decode(rx_buf);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
    }
}
