#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "ch32v20x_conf.h"

/* ---------- RS485 收发控制引脚 ---------- */
#define TX4_EN_Pin          GPIO_Pin_10
#define TX4_EN(n)           (n ? GPIO_WriteBit(GPIOB, TX4_EN_Pin, Bit_SET) : GPIO_WriteBit(GPIOB, TX4_EN_Pin, Bit_RESET))

/* ---------- DMA 接收缓冲区：双行各 512 字节，环形累积 ---------- */
#define UART4_DMABufSize    512
#define UART4_MAX_ROW       2
extern u8 usart4_rx_buf[UART4_MAX_ROW][UART4_DMABufSize];
extern u16 usart4_rx_sta[UART4_MAX_ROW];
extern u8 quere_head_usart4;
extern u8 quere_tail_usart4;
extern u8 quere_empty_usart4;



/* ---------- 函数声明 ---------- */
void USART4_Init(void);
void USART4_DMA_ReceiveData(void);
void USART4_DMA_INIT(void);
void USART4_DataPack_Process(void);
void Usart4_Send(u8 *pSendDataA, u8 LengthA);

/* ---------- IAP 入口检测 ---------- */
uint8_t IAP_EnterBootMode(void);
void    IAP_JumpToApp(void);

#endif /* __BSP_USART_H */
