//
// Created by 34285 on 2026/7/20.
//
#include "drv_wdg.h"

/**
 * @brief 喂独立看门狗
 *
 */
void IWDG_Independent_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_IWDG_PeriodElapsedCallback(void)
{
    IWDG_Independent_Feed();
}