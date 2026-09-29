#include "debug.h"
#include "bsp_gpio.h"
#include "bsp_usart.h"


/*********************************************************************
 * @fn      main
 *
 * @brief   APP 主循环：1Hz RUN_LED 心跳 + 上位机 IAP 协议服务
 *          （UART4 查询 GET_INFO/GET_VER、升级入口 GO_IAP 0x21）。
 *          LED 与协议均按 10ms 时间片轮询，查询响应延迟 <= 20ms。
 *
 * @return  none
 */
int main(void)
{
    u32 tick = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    SystemCoreClockUpdate();
    Delay_Init();

    PORT_Init();        /* PB13/PB14 LED + PB10 RS-485 方向 */
    USART4_Init();      /* UART4 DMA+IDLE 接收，服务上位机查询/升级命令 */

    while (1) {
        USART4_DataPack_Process();   /* 非阻塞：解析并应答上位机请求帧 */

        Delay_Ms(10);
        tick++;
        if ((tick % 50) == 0) {      /* 10ms x 50 = 500ms 翻转 -> 1Hz 心跳 */
            if (GPIO_ReadOutputDataBit(GPIOB, LED_Pin) == Bit_SET) {
                GPIO_ResetBits(GPIOB, LED_Pin);
            } else {
                GPIO_SetBits(GPIOB, LED_Pin);
            }
        }
    }
}