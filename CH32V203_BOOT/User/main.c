#include "debug.h"
#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_iap.h"


/*********************************************************************
 * @fn      main
 *
 * @brief   Bootloader main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    PORT_Init();        /* 先初始化 GPIO（PB13/PB14 LED + PB10 RS-485 EN） */
    USART4_Init();      /* 再初始化 UART4（PB0 TX AF_PP / PB1 RX FLOATING） */


    printf("BOOT v1.0.3\r\n");
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* 检测是否进入 IAP 模式（二者取或，BKP 标志读取即清除）：
     * ① BKP 标志：App 收到 GO_IAP(0x21) 写入后复位 —— 上位机请求升级；
     * ② App 入口无效（空片 / App 区被擦除）；
     * ③ App 入口有效、但升级状态标志 0x08003F00 为 DIRTY（或未识别值）—— 上次升级
     *    在擦/写中途掉电，App 残缺，严禁跳转（防变砖核心，见 bsp_iap.c 的 IAP_EnterBootMode）。 */
    if (IAP_CheckBootFlag() || IAP_EnterBootMode()) {
        ERR_LED(0);  /* 亮灯 = IAP 模式 */
        printf("Enter IAP mode\r\n");
        while (1) {
            USART4_DataPack_Process();
        }
    } else {
        ERR_LED(1);  /* 灭灯 = 正常运行 */
        printf("Jump to App\r\n");
        IAP_JumpToApp();
    }
}
