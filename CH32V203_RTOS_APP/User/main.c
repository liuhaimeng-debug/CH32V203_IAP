/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *task1 and task2 alternate printing
 */

#include "debug.h"
#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_iap.h"

#include "FreeRTOS.h"
#include "task.h"


/*
  任务优先级数值越大优先级越高
*/
#define START_TASK_PRIO 1               //任务优先级
#define START_STK_SIZE  256             //任务堆栈大小
TaskHandle_t StartTask_Handler;         //任务句柄
void start_task(void *pvParameters);    //任务函数

#define LED_TASK_PRIO 2                 //任务优先级
#define LED_STK_SIZE  256               //任务堆栈大小
TaskHandle_t LedTask_Handler;           //任务句柄
void led_task(void *pvParameters);      //任务函数

#define USART4_TASK_PRIO 3              //任务优先级
#define USART4_STK_SIZE  1024            //任务堆栈大小
TaskHandle_t Usart4Task_Handler;        //任务句柄
void usart4_task(void *pvParameters);   //任务函数

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    SystemCoreClockUpdate();
    // Delay_Init();
    // USART_Printf_Init(115200);

    BSP_PORT_Init();
    USART4_Init();

    //创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄
    vTaskStartScheduler();

    while(1)
    {
        // printf("shouldn't run at here!!\n");
    }
}

void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();       //开启临界区

    xTaskCreate((TaskFunction_t ) led_task,
                (const char   * ) "led_task",
                (uint16_t       ) LED_STK_SIZE,
                (void *         ) NULL,
                (UBaseType_t    ) LED_TASK_PRIO,
                (TaskHandle_t * ) &LedTask_Handler);
    xTaskCreate((TaskFunction_t ) usart4_task,
                (const char   * ) "usart4_task",
                (uint16_t       ) USART4_STK_SIZE,
                (void *         ) NULL,
                (UBaseType_t    ) USART4_TASK_PRIO,
                (TaskHandle_t * ) &Usart4Task_Handler);

    vTaskDelete(StartTask_Handler);           //删除开始任务
    taskEXIT_CRITICAL();                      //退出临界区
}

void led_task(void *pvParameters)
{

  for(;;)
  {
        Run_LED(0);
        vTaskDelay(1000);
        Run_LED(1);
        vTaskDelay(1000);
  }
}

void usart4_task(void *pvParameters)
{
    while(1)
    {
        USART4_DataPack_Process();
        vTaskDelay(10);
    }
}