/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      SW_Handler
 *
 * @brief   软中断处理：IAP 跳转 App 的入口（WCH 官方 EVT USB_UART 例程同款）。
 *
 * @return  none
 *
 * ⚠⚠ 为什么必须经软中断跳转：
 *   BOOT 的 main 在 startup 的 mret（mstatus.MPP=00）之后运行于用户模式，
 *   普通函数调用/`jr` 跳转不会切换特权级 —— 若在用户模式直接 jr 进入 App，
 *   App 的 handle_reset 里 csrw mstatus/mtvec/mepc 等 M 态 CSR 写全部触发
 *   非法指令异常（GDB 实测 mcause=2，第一个倒下的是 csrw 0xbc0，去掉后
 *   csrw mstatus 接着倒），异常经 BOOT 的 mtvec 落回 BOOT 的 HardFault →
 *   NVIC_SystemReset → 复位死循环（现象：errLED 长亮、runled 不闪）。
 *   软中断/异常固定进入 M 模式，在这里 jr 进 App，App 启动的 CSR 写才合法。
 */
void SW_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void SW_Handler(void)
{
    __asm("li  a6, 0x4000");   /* = bsp_iap.h 的 APP_EXEC_START_ADDR（App 执行窗口入口） */
    __asm("jr  a6");
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
  NVIC_SystemReset();
  while (1)
  {
  }
}


