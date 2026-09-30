#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "ch32v20x_conf.h"

#define LED_Pin                 GPIO_Pin_14
#define ERR_LED_Pin             GPIO_Pin_13
//#define LED(n)      (n ? GPIO_WriteBit(GPIOB, LED_Pin, Bit_SET) : GPIO_WriteBit(GPIOB, LED_Pin, Bit_RESET))          //运行灯
#define ERR_LED(n)    (n ? GPIO_WriteBit(GPIOB, ERR_LED_Pin, Bit_SET) : GPIO_WriteBit(GPIOB, ERR_LED_Pin, Bit_RESET))          //故障灯


#define SENSOR_Pin              GPIO_Pin_12
#define SEN_STATE               GPIO_ReadInputDataBit(GPIOA, SENSOR_Pin)

void PORT_Init(void);  //初始化

#endif
