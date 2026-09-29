#include "bsp_gpio.h"

/*******************************************************************************
 * 函数名  : PORT_Init
 * 说明       : 初始化IO
 * 输入       : None
 * 输出       : None
 *******************************************************************************/
void PORT_Init (void) {

    GPIO_InitTypeDef GPIO_InitStructure = {0};                                     // 定义一个GPIO_InitTypeDef类型的结构体

    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);  // 使能GPIOA、B端口时钟


    GPIO_InitStructure.GPIO_Pin = ERR_LED_Pin | LED_Pin;  // 配置GPIO引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      // 设置GPIO模式为推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;      // 设置GPIO口输出速度
    GPIO_Init (GPIOB, &GPIO_InitStructure);               // 调用库函数，初始化GPIOB

    GPIO_SetBits (GPIOB, LED_Pin );          // 设置引脚输出高电平
    GPIO_ResetBits(GPIOB, ERR_LED_Pin);        //设置引脚输出低电平


    GPIO_InitStructure.GPIO_Pin = SENSOR_Pin;         // 配置GPIO引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     // 设置GPIO模式为上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  // 设置GPIO口输出速度
    GPIO_Init (GPIOA, &GPIO_InitStructure);           // 调用库函数，初始化GPIOB
}
