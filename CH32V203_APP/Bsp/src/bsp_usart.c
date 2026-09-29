#include "bsp_usart.h"
#include "bsp_iap.h"
#include "string.h"

#define IAP_RX_BUF_SIZE  (UART4_DMABufSize * UART4_MAX_ROW)  /* 256B */

/* ---------- DMA 原始接收缓冲区（独立于环形队列，避免竞争） ---------- */
static u8 Uart4DmaBuf[UART4_DMABufSize];

/* ---------- 双行环形队列（累积接收数据） ---------- */
u8 usart4_rx_buf[UART4_MAX_ROW][UART4_DMABufSize] = {0};
u16 usart4_rx_sta[UART4_MAX_ROW] = {0};
u8 quere_head_usart4  = 0;
u8 quere_tail_usart4  = 0;
u8 quere_empty_usart4 = 1;




/* =========================================================
 *  UART4 初始化
 * ========================================================= */
void USART4_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* TX: PB0 AF_PP, RX: PB1 floating, EN: PB10 push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = TX4_EN_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOB, TX4_EN_Pin, Bit_RESET);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART4, &USART_InitStructure);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART4_DMA_INIT();
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART4, ENABLE);
}

void USART4_DMA_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel8);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&UART4->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Uart4DmaBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = UART4_DMABufSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel8, &DMA_InitStructure);

    /* 注意：本工程不使用 DMA TC 中断（DMA1_Channel8_IRQHandler 未实现，为
     * 弱定义死循环）——DMA 接收完全由 UART4 IDLE 中断触发
     * USART4_DMA_ReceiveData() 搬运并重装。不开 TC 中断、不开 NVIC，
     * 避免跳转 App 后残留 TC 中断在 App 里命中弱定义处理函数导致挂死。 */

    DMA_Cmd(DMA1_Channel8, ENABLE);
}
/* =========================================================
 *  UART4 DMA 接收处理（双缓冲环形队列）
 * ========================================================= */
void USART4_DMA_ReceiveData(void)
{
    DMA_Cmd(DMA1_Channel8, DISABLE);

    usart4_rx_sta[quere_tail_usart4] = UART4_DMABufSize -
                                       DMA_GetCurrDataCounter(DMA1_Channel8);
    memcpy(usart4_rx_buf[quere_tail_usart4], Uart4DmaBuf,
           usart4_rx_sta[quere_tail_usart4]);

    quere_tail_usart4++;
    quere_empty_usart4 = 0;
    if (quere_tail_usart4 >= UART4_MAX_ROW)
        quere_tail_usart4 = 0;

    /* 正常模式：重启DMA等待下一帧 */
    DMA_SetCurrDataCounter(DMA1_Channel8, UART4_DMABufSize);
    DMA_Cmd(DMA1_Channel8, ENABLE);
}
/* =========================================================
 *  UART4 IDLE 中断处理
 * ========================================================= */
void UART4_IRQHandler(void) __attribute__((interrupt()));

void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        USART4_DMA_ReceiveData();
        USART_ReceiveData(UART4);   /* 清 IDLE 标志 */
    }
}

/* =========================================================
 *  IAP 帧解析（从环形缓冲区中抽取完整请求帧）
 *  请求帧格式：[5A A5][LENH LENL][CMD][SEQ][PAYLOAD...][CRC_L CRC_H]
 *  LEN = CMD(1)+SEQ(1)+Payload，CRC范围：从LEN字节到Payload末尾
 *
 *  跨调用保留 parse_state/parse_pos/offset，
 *  半帧未凑齐时 break，下次继续解析，避免残留脏数据导致 CRC 误判。
 * ========================================================= */
void USART4_DataPack_Process(void)
{
    static u8   parse_buf[IAP_RX_BUF_SIZE];
    static u16  parse_pos   = 0;  /* 已合并进 parse_buf 的总字节数（含未消费尾部） */
    static u8   parse_state = 0;  /* 0=找0x5A, 1=等0xA5, 2=等LENH..SEQ头, 3=收Payload+CRC */
    static u8   frame_seq   = 0;
    static u16  frame_len   = 0;  /* LEN字段值 */
    static u16  offset      = 0;  /* 当前解析位置（跨调用保留） */

    u8 *buf_ptr;
    u16 buf_len;
    u16 crc_recv, crc_calc;
    volatile u16 i;

    /* 将环形队列中的新数据追加到 parse_buf 尾部 */
    if (!quere_empty_usart4) {
        i = quere_head_usart4;
        do {
            buf_ptr  = usart4_rx_buf[i];
            buf_len  = usart4_rx_sta[i];
            if (buf_len > 0) {
                if (parse_pos + buf_len > IAP_RX_BUF_SIZE) {
                    /* 溢出保护：仅在主循环被长时间阻塞（如整片擦除）导致
                     * 队列积满时触发，丢弃已缓冲数据防止越过 parse_buf 边界，
                     * 上位机超时重发即可恢复 */
                    parse_pos = 0;
                    offset = 0;
                    parse_state = 0;
                }
                memcpy(parse_buf + parse_pos, buf_ptr, buf_len);
                parse_pos += buf_len;
            }
            i++;
            if (i >= UART4_MAX_ROW) i = 0;
        } while (i != quere_tail_usart4);

        quere_head_usart4 = quere_tail_usart4;
        quere_empty_usart4 = 1;
    }

    if (parse_pos == 0) return;

    /* 状态机解析请求帧（offset 从上次保留的位置继续） */
    for (; offset < parse_pos; ) {
        if (parse_state == 0) {
            /* 查找同步头第一个字节 0x5A */
            while (offset < parse_pos && parse_buf[offset] != IAP_FRAME_SYNC_H)
                offset++;
            if (offset >= parse_pos) break;
            offset++;
            parse_state = 1;
            continue;
        }

        if (parse_state == 1) {
            /* 第二个同步字节必须是 0xA5；offset 恰指向待确认字节 */
            if (offset >= parse_pos) break;
            if (parse_buf[offset] != IAP_FRAME_SYNC_L) {
                /* 不是有效同步头，回到状态0继续搜索 */
                offset++;
                parse_state = 0;
                continue;
            }
            offset++;       /* 已确认 0x5A 0xA5，offset → LENH */
            parse_state = 2;
            continue;
        }

        if (parse_state == 2) {
            /* 等待帧头 LENH LENL CMD SEQ 共 4 字节；offset 恰指向 LENH */
            if (parse_pos - offset < 4) break;

            frame_len  = ((u16)parse_buf[offset] << 8) | parse_buf[offset + 1];
            frame_seq  = parse_buf[offset + 3];
            offset    += 4;     /* offset → Payload 起始 */

            /* 长度合法性检查 */
            if (frame_len > IAP_MAX_DATA_LEN) {
                parse_state = 0;
                continue;
            }
            parse_state = 3;
            continue;
        }

        if (parse_state == 3) {
            /* 等待完整帧：Payload(frame_len-2) + CRC(2) = frame_len 字节 */
            if (parse_pos - offset < frame_len)
                break;  /* 半帧未凑齐，保留 state/offset，下次继续 */

            /* CRC16 校验：覆盖 LENH~PAYLOAD 末尾（共 frame_len+2 字节） */
            crc_calc = IAP_CRC16_Calc(parse_buf + offset - 4, frame_len + 2);

            crc_recv = (u16)parse_buf[offset + frame_len - 2] |
                       ((u16)parse_buf[offset + frame_len - 1] << 8);

            if (crc_recv != crc_calc) {
                /* CRC 错误，跳过此帧（offset 在 Payload 起始，整帧占 frame_len 字节） */
                offset += frame_len;
                parse_state = 0;
                continue;
            }

            /* 提取 CMD 和 Payload
             * 帧布局: [5A][A5][LenH][LenL][Cmd][Seq][Data...][CRC_L][CRC_H]
             *                              ^offset-2（Cmd 起始）
             * IAP_ProcessFrame 约定 data = [Cmd][Seq][Payload...]，len = frame_len
             */
            u8 *pload = parse_buf + offset - 2;  /* 指向 Cmd */
            u16 plen  = frame_len;               /* Cmd(1)+Seq(1)+Payload，须用 u16（262 超出 u8） */

            IAP_ProcessFrame(frame_seq, pload, plen);

            offset += frame_len;
            parse_state = 0;
            continue;
        }
    }

    /* 压缩 parse_buf：丢弃已消费前缀，保留未消费尾部，避免 parse_buf 无限增长。
     * 关键：state 3 挂起的半帧 offset 已指向 Payload，必须保留从 LENH 起的
     * 4 字节帧头（offset-4），否则下次继续解析时 CRC 基准指针
     * parse_buf+offset-4 会越过缓冲区头部，导致跨 IDLE 分片到达的帧永远
     * 校验失败。state 0/1/2 挂起时 offset 恰指向下一个待确认字节，
     * 从 offset 处保留即可（state 1/2 首字节即 A5/LENH，保留后 offset=0 语义不变）。 */
    if (parse_state == 3 && offset >= 4) {
        u16 keep_from = offset - 4;                 /* LENH 在压缩后缓冲中的位置 0 */
        u16 remain    = parse_pos - keep_from;
        memmove(parse_buf, parse_buf + keep_from, remain);
        parse_pos = remain;
        offset    = 4;                              /* Payload 起新偏移 */
    } else if (offset > 0 && offset < parse_pos) {
        u16 remain = parse_pos - offset;
        memmove(parse_buf, parse_buf + offset, remain);
        parse_pos = remain;
        offset    = 0;
    } else if (offset >= parse_pos) {
        /* 全部消费完 */
        parse_pos = 0;
        offset    = 0;
    }
}

/* =========================================================
 *  公共函数
 * ========================================================= */
void Usart4_Send(u8 *pSendDataA, u8 LengthA)
{
    u8 count;
    TX4_EN(1);
    Delay_Ms(1);
    for (count = 0; count < LengthA; count++)
    {
        USART_SendData(UART4, *pSendDataA++);
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    }
    /* ⚠ 勿用 while(TC==RESET) 代替尾延时！TC 在"读 STATR 后再访问 DATAR"时
     * 被硬件清除，而 UART4 的 RX 路径（DMA1_Channel8 读 DATAR 收字节）会持续
     * 访问 DATAR，随时把刚置位的 TC 清掉 → 死等循环。
     * GDB 实测复现：Boot 卡死在 USART_GetFlagStatus(TC) 的轮询里，
     * ra 指向 TC 轮询点，ACK 发不完、IAP 通道整体失联。
     * 固定尾延时 1ms（约 43 个字节时间）不依赖任何标志，久经考验。 */
    Delay_Ms(1);
    TX4_EN(0);
}
