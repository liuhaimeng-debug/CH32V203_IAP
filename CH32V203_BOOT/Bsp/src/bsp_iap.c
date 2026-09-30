/*
 * bsp_iap.c：IAP 通讯协议栈实现（新协议 v2）
 * 适用于 CH32V203 系列，基于 UART4 + DMA 接收
 *
 * 帧格式（请求）：[5A A5][LENH LENL][CMD][SEQ][PAYLOAD...][CRC_L CRC_H]
 * 帧格式（响应）：[5A A5][LENH LENL][CMD|0x80][SEQ][FLAG][STATUS][DATA...][CRC_L CRC_H]
 * CRC16：CCITT-FALSE（poly 0x1021, init 0xFFFF），计算范围从 LEN 字节到 DATA 末尾
 */
#include "bsp_iap.h"
#include "bsp_usart.h"
#include "bsp_gpio.h"
#include "ch32v20x_flash.h"
#include "core_riscv.h"
#include <string.h>

#ifdef IAP_DELAY_MS_IS_RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/* ---------- 内部变量 ---------- */
static volatile u8   g_iap_seq_recv  = 0;   /* 当前序列号（回显用） */
static volatile u32  g_iap_dl_addr   = APP_START_ADDR;
static volatile u32  g_iap_dl_size   = 0;
static volatile u32  g_iap_dl_written = 0;
static volatile u8   g_iap_dl_active = 0;

/* ---------- Flash 超时保护位（防止 RISC-V Flash 控制器 SR_BSY 卡死主循环） ----------
 * 库函数 FLASH_ErasePage_Fast / FLASH_ProgramPage_Fast 内部是 while(STATR & SR_BSY)
 * 死循环，若 64KB(D8) 芯片某页时序异常会永远卡住 Boot（表现为上位机"等待响应超时"）。
 * 下面本地实现加了超时计数，超时则清错误标志位并返回失败，避免主循环卡死。 */
#define FLASH_BUSY_TIMEOUT    10000000UL   /* 忙等待最大循环次数（约 <50ms @ 高频） */
#define FLASH_SR_BSY          0x01
#define FLASH_SR_WR_BSY       0x02
#define FLASH_SR_PGERR        0x04   /* STM32 移植遗留：CH32V20x STATR 无此错误位
                                        (WCH SPL 只定义 BSY/WR_BSY/WRPRTERR/EOP)，
                                        该位读出为脏值，禁止用于错误判定 */
#define FLASH_SR_WRPRTERR     0x10
#define FLASH_SR_EOP          0x20
#define FLASH_CR_PAGE_PG      0x00010000
#define FLASH_CR_PAGE_ER      0x00020000
#define FLASH_CR_PG_STRT      0x00200000
/* 擦除启动位 = CR_STRT_Set = 0x40（与 ch32v20x_flash.c 一致）。
 * 历史 bug：曾误写成 0x00000001（那是标准编程位 CR_PG），导致擦除从未真正
 * 启动——BSY 不置位、无错误标志、函数"假成功"，页数据原样保留，后续编程
 * 报 PGERR（NAK 0x06）。 */
#define FLASH_CR_STRT         0x00000040

/* 擦除单个 256B 页（带超时保护）。成功返回 1，失败（超时/写保护/回读非 0xFF）返回 0。
 * 注意：错误判定只用 WRPRTERR——CH32V20x 没有 PGERR 位，0x04 读出是脏值。 */
static u8 IAP_FlashErasePageSafe(u32 page_addr)
{
    u32 v;

    /* 清残留 EOP/WRPRTERR（写 1 清 0） */
    FLASH->STATR = FLASH_SR_EOP | FLASH_SR_WRPRTERR;

    FLASH->CTLR |= FLASH_CR_PAGE_ER;
    FLASH->ADDR  = page_addr & 0xFFFFFF00;
    FLASH->CTLR |= FLASH_CR_STRT;

    v = 0;
    while (FLASH->STATR & FLASH_SR_BSY) {
        if (++v >= FLASH_BUSY_TIMEOUT) {
            FLASH->CTLR &= ~FLASH_CR_PAGE_ER;
            FLASH->STATR = FLASH_SR_EOP | FLASH_SR_WRPRTERR;
            return 0;   /* 忙超时 */
        }
    }
    FLASH->CTLR &= ~FLASH_CR_PAGE_ER;

    if (FLASH->STATR & FLASH_SR_WRPRTERR) {
        FLASH->STATR = FLASH_SR_EOP | FLASH_SR_WRPRTERR;
        return 0;   /* 写保护 */
    }

    /* 回读校验：CH32V203 Flash 带 ECC，擦除态读回为 0xE339E339（不是常规的
     * 0xFFFFFFFF！）。防止擦除"假成功"（如 STRT 启动位写错时 BSY 不置位、
     * 无错误标志，函数照样返回成功但页数据原样保留——此 bug 曾真实发生）。 */
    if (*(volatile u32 *)(page_addr & 0xFFFFFF00) != 0xE339E339UL) {
        return 0;
    }
    return 1;
}

/* 编程单个 256B 页（带超时保护）。page_addr 须 256B 对齐，pbuf 提供完整 256B。
 * 注意：pbuf 可为任意对齐（它指向 parse_buf+10，不保证 4B 对齐），
 * 故下面按字节 memcpy 组装 u32，绝不直接 *(const u32 *)pbuf 读取。
 * 直接按 u32* 读非对齐地址会触发 QingKe V4 对齐异常 → HardFault →
 * NVIC_SystemReset() 静默复位（实测现象：首个 WRITE_BLK 无应答超时，
 * 重试回 NAK 0x0A App 未就绪 = active 被复位清零）。 */
static u8 IAP_FlashProgramPageSafe(u32 page_addr, const u8 *pbuf)
{
    u8  size = 64;
    u32 v;
    u32 paddr = page_addr & 0xFFFFFF00;
    u32 w;

    /* 先清残留 EOP/WRPRTERR（写 1 清 0），防止残留标志误判"写失败"（NAK 0x06）。
     * 0x04 不是本芯片错误位，不得清除/判定。 */
    FLASH->STATR = FLASH_SR_EOP | FLASH_SR_WRPRTERR;

    FLASH->CTLR |= FLASH_CR_PAGE_PG;
    v = 0;
    while (FLASH->STATR & FLASH_SR_BSY) {
        if (++v >= FLASH_BUSY_TIMEOUT) { FLASH->CTLR &= ~FLASH_CR_PAGE_PG; return 0; }
    }
    v = 0;
    while (FLASH->STATR & FLASH_SR_WR_BSY) {
        if (++v >= FLASH_BUSY_TIMEOUT) { FLASH->CTLR &= ~FLASH_CR_PAGE_PG; return 0; }
    }

    while (size) {
        memcpy(&w, pbuf, 4);       /* 按字节组装，容忍 pbuf 非 4B 对齐 */
        *(volatile u32 *)paddr = w;
        paddr += 4;
        pbuf  += 4;
        size  -= 1;
        v = 0;
        while (FLASH->STATR & FLASH_SR_WR_BSY) {
            if (++v >= FLASH_BUSY_TIMEOUT) {
                FLASH->CTLR &= ~FLASH_CR_PAGE_PG;
                return 0;   /* 单次写超时 */
            }
        }
    }

    FLASH->CTLR |= FLASH_CR_PG_STRT;
    v = 0;
    while (FLASH->STATR & FLASH_SR_BSY) {
        if (++v >= FLASH_BUSY_TIMEOUT) { FLASH->CTLR &= ~FLASH_CR_PAGE_PG; return 0; }
    }
    FLASH->CTLR &= ~FLASH_CR_PAGE_PG;

    if (FLASH->STATR & FLASH_SR_WRPRTERR) {
        FLASH->STATR = FLASH_SR_EOP | FLASH_SR_WRPRTERR;
        return 0;   /* 写保护（0x04 非本芯片错误位，不判定） */
    }
    return 1;
}

/* ---------- CRC-16-CCITT 软件实现 ---------- */
u16 IAP_CRC16_Calc(const u8 *data, u16 len)
{
    u16 crc = CRC16_INIT;
    for (u16 i = 0; i < len; i++) {
        crc ^= (u16)data[i] << 8;
        for (u8 j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (u16)((crc << 1) ^ CRC16_POLY);
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ---------- 发送一帧（同步头+Len+Cmd+Seq+Flag+Status+Data+CRC16） ----------
 * 响应帧布局：[5A A5][LENH LENL][CMD][SEQ][FLAG][STATUS][DATA...][CRC_L CRC_H]
 *  - FLAG:   0x79=ACK, 0x1F=NAK
 *  - STATUS: 0x00=成功；NAK 时为错误码（见 bsp_iap.h 错误码表）
 *  - DATA:   响应数据；NAK 帧不携带 DATA（len 必须为 0）
 * 注意：CRC 计算范围从 LENH 到 DATA 末尾（crc_len+4 字节），crc_len = 2 + len
 */
void IAP_SendFrame(u8 cmd, const u8 *data, u8 len, u8 flag, u8 status)
{
    u8 buf[256] = {0};
    u16 crc;
    u16 crc_len;  /* LEN字段值 = FLAG(1)+STATUS(1)+data(len) */

    buf[0] = IAP_FRAME_SYNC_H;    /* 0x5A */
    buf[1] = IAP_FRAME_SYNC_L;    /* 0xA5 */

    crc_len = 2 + len;            /* LEN = FLAG(1)+STATUS(1)+data */
    buf[2] = (u8)(crc_len >> 8);
    buf[3] = (u8)(crc_len & 0xFF);

    buf[4] = cmd;
    buf[5] = g_iap_seq_recv;      /* 回显SEQ */
    buf[6] = flag;                /* FLAG: 0x79=ACK, 0x1F=NAK */
    buf[7] = status;             /* STATUS: 0x00=成功，NAK 时=错误码 */

    if (data && len > 0) {
        memcpy(buf + 8, data, len);
    }

    /* CRC16 计算范围：从 LENH 字节开始到 DATA 末尾（共 crc_len+4 字节） */
    crc = IAP_CRC16_Calc(buf + 2, crc_len + 4);
    buf[8 + len]     = (u8)(crc & 0xFF);        /* CRC低字节 */
    buf[8 + len + 1] = (u8)((crc >> 8) & 0xFF); /* CRC高字节 */

    Usart4_Send(buf, (u8)(8 + len + 2));
}

/* ---------- 发送 NAK 错误响应 ----------
 * 错误码放入 DATA 字段（1 字节），STATUS 固定为 0x00。
 * 响应帧布局：[5A A5][LENH LENL][F0][SEQ][1F][00][errcode][CRC_L][CRC_H]
 */
void IAP_SendNAK(u8 errcode)
{
    u8 status = errcode;
    IAP_SendFrame(NAK_CMD, &status, 1, IAP_FLAG_NAK, 0);
}

/* ---------- 写入 Flash 升级状态标志（0x08003F00） ----------
 * 位于 Boot 区最后一页（256B），由 Boot 自身维护，App 绝不访问。
 * 成功返回 1，失败返回 0。
 */
static u8 IAP_FlashWriteFlag(u32 flag_val)
{
    u8 buf[256];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &flag_val, sizeof(flag_val));

    FLASH_Unlock_Fast();
    u8 ok = IAP_FlashErasePageSafe(IAP_FLAG_PAGE_ADDR);
    if (ok) {
        ok = IAP_FlashProgramPageSafe(IAP_FLAG_PAGE_ADDR, buf);
    }
    FLASH_Lock_Fast();

    if (ok && *(volatile u32 *)IAP_FLAG_PAGE_ADDR == flag_val) {
        return 1;
    }
    return 0;
}

/* ---------- 计算 App 区 Flash CRC16 ---------- */
static u16 IAP_CalcAppFlashCRC(u32 addr, u32 size)
{
    const u8 *p = (const u8 *)addr;
    return IAP_CRC16_Calc(p, (u16)size);
}

/* ---------- 处理 GET_INFO 命令 ---------- */
static void IAP_HandleGetInfo(void)
{
    u8 resp[48];
    u16 sz = 0;

    const char *info = "CH32V203C8T6|BOOT:v1.0.3|Flash:64KB";
    sz = (u16)strlen(info);
    if (sz > sizeof(resp)) sz = sizeof(resp);
    memcpy(resp, info, sz);

    IAP_SendFrame(ACK_INFO, resp, (u8)sz, IAP_FLAG_ACK, 0);
}

/* ---------- 处理 GET_VER 命令 ---------- */
static void IAP_HandleGetVer(void)
{
    u8 ver[2] = { 0x01, 0x00 };  /* 协议版本 1.0 */
    IAP_SendFrame(ACK_VER, ver, 2, IAP_FLAG_ACK, 0);
}

/* ---------- 处理 ERASE_APP 命令 ----------
 * 用本地 IAP_FlashErasePageSafe 逐页擦除（256B/页），不用库 FLASH_ROM_ERASE。
 * 原因：库 FLASH_ROM_ERASE 内部调 ROM_ERASE，其 while(STATR & SR_BSY) 无超时保护，
 * 且库函数擦完后 KEYR/Lock 会清 BSY/EOP 等状态位，但 PGERR/WRPRTERR 残留不清，
 * 导致紧接着的第一次写块 IAP_FlashProgramPageSafe while(BSY) 死循环卡死主循环。
 * 逐页擦除每页后显式清错误位，可避免此问题。
 *
 * 【掉电保护状态机】：
 * 进入擦除阶段首先写入 UPGRADE_DIRTY 标志。
 * 若后续擦除/写入过程中遭遇突发掉电，重启后 Boot 读取标志强制停留在 IAP 模式，
 * 绝不会把已写入部分代码/损坏的 App 误当作正常固件跳转。
 */
static void IAP_HandleEraseApp(void)
{
    u32 page_count = (APP_END_ADDR - APP_START_ADDR) / 0x100;  /* 192 页 @ 48KB */
    u32 page_addr;
    u8  all_ok = 1;

    /* 升级开始：标记 UPGRADE_DIRTY（防掉电变砖核心机制） */
    if (!IAP_FlashWriteFlag(IAP_UPGRADE_DIRTY)) {
        IAP_SendNAK(ERR_ERASE_FAIL);
        return;
    }

    FLASH_Unlock_Fast();

    for (page_addr = APP_START_ADDR; page_addr < APP_END_ADDR;
         page_addr += 0x100, page_count--) {
        if (!IAP_FlashErasePageSafe(page_addr)) {
            all_ok = 0;
            break;
        }
    }

    /* 清残留 EOP/WRPRTERR（0x04 非本芯片错误位） */
    FLASH->STATR = FLASH_SR_EOP | FLASH_SR_WRPRTERR;
    FLASH_Lock_Fast();

    if (all_ok) {
        g_iap_dl_written = 0;
        g_iap_dl_active  = 0;
        IAP_SendFrame(ACK_OK, NULL, 0, IAP_FLAG_ACK, 0);
    } else {
        /* 诊断：NAK 数据 = [0x05][STATR 低 8 位][失败页序号(0 起)]，上位机日志可见 */
        u8 detail[3] = {
            ERR_ERASE_FAIL,
            (u8)(FLASH->STATR & 0xFF),
            (u8)((APP_END_ADDR - APP_START_ADDR) / 0x100 - page_count)
        };
        IAP_SendFrame(NAK_CMD, detail, 3, IAP_FLAG_NAK, 0);
    }
}

/* ---------- 处理 START_DL 命令 ----------
 * 载荷: [Addr:4B][Size:4B]（32 位大端，完整 32 位地址）
 * 严格幂等方案：用 App 区区间做校验。
 */
static void IAP_HandleStartDL(const u8 *data)
{
    u32 addr = ((u32)data[0] << 24) | ((u32)data[1] << 16) |
               ((u32)data[2] << 8) | data[3];
    u32 size = ((u32)data[4] << 24) | ((u32)data[5] << 16) |
               ((u32)data[6] << 8) | data[7];

    /* 先做区间校验：起始地址须在 App 区内，且整段不越界（48KB 全为可写区）。
     * 校验通过才激活写指针，避免非法 START_DL 污染下载状态。 */
    if (addr < APP_START_ADDR || addr >= APP_END_ADDR ||
        size == 0 ||
        (addr + size) > APP_END_ADDR) {
        g_iap_dl_active = 0;
        IAP_SendNAK(ERR_ADDR_OUT);
        return;
    }

    g_iap_dl_addr    = addr;
    g_iap_dl_size    = size;
    g_iap_dl_written = 0;
    g_iap_dl_active  = 1;

    /* 回显解析出的 addr/size（大端 4+4 = 8 字节），供上位机核对（临时调试） */
    u8 echo[8] = {
        (u8)((addr >> 24) & 0xFF), (u8)((addr >> 16) & 0xFF),
        (u8)((addr >> 8) & 0xFF),  (u8)(addr & 0xFF),
        (u8)((size >> 24) & 0xFF), (u8)((size >> 16) & 0xFF),
        (u8)((size >> 8) & 0xFF),  (u8)(size & 0xFF)
    };
    IAP_SendFrame(ACK_OK, echo, 8, IAP_FLAG_ACK, 0);
}

/* ---------- 处理 WRITE_BLK 命令 ---------- */
static void IAP_HandleWriteBlk(const u8 *data, u16 len)
{
    ERR_LED(1);   /* 亮 = 收到写块请求 */
    if (!g_iap_dl_active) {
        /* 未先发送 START_DL 就开始写块 */
        IAP_SendNAK(ERR_APP_NOT_READY);
        return;
    }

    /* 载荷格式: [Addr:4B][Data:N]（32 位大端地址）
     * 入参 len = 载荷长度（IAP_ProcessFrame 已去掉 CMD/SEQ；
     * 帧尾 CRC 不在载荷内，无需再减），故数据段 = len - 4(addr) */
    if (len < 5) {   /* 4(addr) + 1(data) 最小 */
        IAP_SendNAK(ERR_LEN_ERR);
        return;
    }

    u32 addr = ((u32)data[0] << 24) | ((u32)data[1] << 16) |
               ((u32)data[2] << 8) | data[3];
    u8  *payload = (u8 *)(data + 4);
    u16 plen    = len - 4;   /* 去掉 4 字节地址 */

    if (addr != g_iap_dl_addr) {
        IAP_SendNAK(ERR_ADDR_OUT);
        return;
    }
    if (plen > IAP_CMD_WRITE_BLK_DATA_MAX) {
        IAP_SendNAK(ERR_LEN_ERR);
        return;
    }
    if ((g_iap_dl_written + plen) > g_iap_dl_size) {
        IAP_SendNAK(ERR_LEN_ERR);
        return;
    }

    /* WCH 官方 IAP 例程的写路径：Fast 编程模式，写前单页擦除。
     * FLASH_ROM_WRITE 在 64KB(D8) 芯片上 STATR 读出脏值导致误判，
     * 改用带超时保护的本地 Flash_ErasePage / Flash_ProgramPage（参考例程 Fast API），
     * 避免库函数 while(SR_BSY) 死循环卡死主循环。 */
    FLASH_Unlock_Fast();
    ERR_LED(0);   /* 灭 = 开始擦除 */
    u8 ok = IAP_FlashErasePageSafe(addr);                /* 擦当前 256B 页（地址已 256B 对齐） */
    ERR_LED(1);   /* 亮 = 擦除完成（或超时返回） */
    if (ok) {
        ok = IAP_FlashProgramPageSafe(addr, payload);
    }
    ERR_LED(0);   /* 灭 = 编程完成（或超时返回） */
    FLASH_Lock_Fast();

    if (!ok) {
        /* 写/擦失败：保持写指针不动（addr 不变），上位机重传同 addr 即幂等恢复 */
        IAP_SendNAK(ERR_WRITE_FAIL);
        return;
    }

    /* 写成功才推进指针：同 addr 重传时因 addr 已变而 NAK，避免覆盖错位 */
    g_iap_dl_written += plen;
    g_iap_dl_addr    += plen;

    /* 写块确认：[Addr:4B]（32 位大端地址） */
    u8 wr_ack[4] = {
        (u8)((addr >> 24) & 0xFF),
        (u8)((addr >> 16) & 0xFF),
        (u8)((addr >> 8)  & 0xFF),
        (u8)(addr & 0xFF)
    };
    IAP_SendFrame(ACK_WRITE, wr_ack, 4, IAP_FLAG_ACK, 0);

    /* 每 16 包上报一次进度 */
    if ((g_iap_dl_written % (16 * IAP_CMD_WRITE_BLK_DATA_MAX)) == 0) {
        u8 prog[3] = {
            (u8)((g_iap_dl_written >> 16) & 0xFF),
            (u8)((g_iap_dl_written >> 8)  & 0xFF),
            (u8)(g_iap_dl_written & 0xFF)
        };
        IAP_SendFrame(ACK_PROGRESS, prog, 3, IAP_FLAG_ACK, 0);
    }
}

/* ---------- 处理 CALC_CRC 命令 ---------- */
static void IAP_HandleCalcCRC(void)
{
    u32 crc_addr = APP_START_ADDR;
    u32 crc_size = (u32)(g_iap_dl_active ? g_iap_dl_written
                                          : (APP_END_ADDR - APP_START_ADDR));

    u16 crc = IAP_CalcAppFlashCRC(crc_addr, crc_size);

    u8 crc_resp[2] = {
        (u8)(crc & 0xFF),
        (u8)((crc >> 8) & 0xFF)
    };
    IAP_SendFrame(ACK_CRC, crc_resp, 2, IAP_FLAG_ACK, 0);
}

/* ---------- 处理 JUMP_APP 命令 ----------
 * 收到 JUMP_APP（上位机已确认 CRC 一致）→ 写 UPGRADE_VALID 标志 → 回 0xA0 → 50ms → 跳转。
 * 若 App 入口非法，不写标志，IAP_JumpToApp() 直接返回，BOOT 继续停在 IAP 模式。
 */
static void IAP_HandleJumpApp(void)
{
    u32 appEntry = *(volatile u32 *)(APP_EXEC_START_ADDR + 4);

    /* 仅当 App 入口有效时才标记升级完成（防掉电状态机收尾）。
     * 先写标志再回 ACK：即使 ACK 丢失/掉电，重启后 Boot 读到 VALID 也能正常跳转。 */
    if (appEntry >= APP_EXEC_START_ADDR && appEntry < APP_EXEC_END_ADDR &&
        (appEntry & 0x01) == 0) {
        IAP_FlashWriteFlag(IAP_UPGRADE_VALID);
    }

    IAP_SendFrame(ACK_JUMP, NULL, 0, IAP_FLAG_ACK, 0);
#ifdef IAP_DELAY_MS_IS_RTOS
    vTaskDelay(50);    /* RTOS：见 bsp_iap.h 说明——Delay_Ms 会踩坏 SysTick/内核节拍 */
#else
    Delay_Ms(50);
#endif
    IAP_JumpToApp();
}

/* ---------- IAP 帧处理主函数 ---------- */
u8 IAP_ProcessFrame(u8 seqno, const u8 *data, u16 len)
{
    g_iap_seq_recv = seqno;

    switch (data[0]) {
        case CMD_GET_INFO:
            IAP_HandleGetInfo();
            break;
        case CMD_GET_VER:
            IAP_HandleGetVer();
            break;
        case CMD_ERASE_APP:
            IAP_HandleEraseApp();
            break;
        case CMD_START_DL:
            IAP_HandleStartDL(data + 2);
            break;
        case CMD_WRITE_BLK:
            IAP_HandleWriteBlk(data + 2, len - 2);
            break;
        case CMD_CALC_CRC:
            IAP_HandleCalcCRC();
            break;
        case CMD_JUMP_APP:
            IAP_HandleJumpApp();
            break;
        case CMD_GO_IAP:
            /* Boot 本身已在升级模式，无需复位进入；明确回角色错误而非未知命令 */
            IAP_SendNAK(ERR_ROLE_ERR);
            break;
        default:
            IAP_SendNAK(ERR_UNKNOWN_CMD);
            break;
    }
    return 0;
}

/* ---------- 读取并清除 BKP“请求进 IAP”标志 ----------
 * App 收到 GO_IAP(0x21) 后：写 DATAR1=0xB007 / DATAR2=0x4FF8 → 回 0xA1
 * → Delay 50ms（等帧发完）→ NVIC_SystemReset()。Boot 启动时在此读取：
 * 命中 → 清零 → 返回 1（本次启动强制停留 IAP）；未命中 → 0。
 * - 需先开 PWR/BKP 时钟并解除备份域写保护（PWR_CTLR_DBP）才能读写 BKP；
 * - 双字互反校验防上电随机值误判；读取即消费，只对紧随的一次启动生效；
 * - 备份域不随系统复位清零；断电（无 VBAT 供电）自然丢失 → 不会跨掉电误进 IAP。
 * ⚠ 硬件验证点：需实测 NVIC_SystemReset 后 DATAR1/2 保持（CH32V20x 备份域
 *   仅被 RCC_BackupResetCmd / 掉电复位清零，不随系统复位清零）。 */
u8 IAP_CheckBootFlag(void)
{
    u16 m1, m2;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);            /* 解除备份域写保护（PWR_CTLR_DBP） */

    m1 = BKP->DATAR1;
    m2 = BKP->DATAR2;

    if (m1 == (u16)IAP_BKP_FLAG_MAGIC1 && m2 == (u16)IAP_BKP_FLAG_MAGIC2) {
        BKP->DATAR1 = 0;                    /* 消费标志 */
        BKP->DATAR2 = 0;
        return 1;
    }
    return 0;
}

/* ---------- 判断是否应进入 IAP 模式 ----------
 * 进入 IAP 的条件（命中任一项即留在 Boot 升级模式）：
 * 1. App 入口地址无效（向量表[0]即 _start 不在 [0x00004000, 0x00010000) 或最低位非 0；
 *    空片/全擦除后该字为 0xE339E339 或 0xFFFFFFFF，必然命中）。
 * 2. Flash 升级标志（0x08003F00）处于脏状态（IAP_UPGRADE_DIRTY）：
 *    说明上一次升级在擦除或烧写过程中发生意外掉电/中断，App 代码已残缺，
 *    即便部分向量已被写入也不能跳转，强制留在 Boot 等待上位机重新升级（防变砖核心！）。
 * 3. 升级标志为未识别的非法值（非 VALID、非 DIRTY、非擦除态），判定异常留在 Boot。
 *
 * 允许跳转 App 的条件：
 * - App 入口向量合法，且升级标志为 IAP_UPGRADE_VALID；
 * - 或者升级标志为出厂/调试器全片擦除态（0xE339E339 / 0xFFFFFFFF）：
 *   保证开发者使用 WCH-Link / OpenOCD / ISP 直接全片擦写后也能顺利启动，无缝兼容调试。
 */
u8 IAP_EnterBootMode(void)
{
    u32 appEntry = *(volatile u32 *)(APP_EXEC_START_ADDR + 4);

    /* 1. 检查 App 入口合法性 */
    if (appEntry < APP_EXEC_START_ADDR || appEntry >= APP_EXEC_END_ADDR ||
        (appEntry & 0x01) != 0) {
        return 1;
    }

    /* 2. 读取 0x08003F00 处的升级状态标志 */
    u32 flag = *(volatile u32 *)IAP_FLAG_PAGE_ADDR;

    if (flag == IAP_UPGRADE_DIRTY) {
        /* 处于未完成的升级状态（断电/异常中断），绝对不能跳转 */
        return 1;
    }

    if (flag == IAP_UPGRADE_VALID) {
        /* 上次 IAP 升级完整成功，允许跳转 */
        return 0;
    }

    if (flag == IAP_FLASH_ERASED_ECC || flag == IAP_FLASH_ERASED_FF) {
        /* 空片或调试器（WCH-Link/ISP）直接整片擦写态，未经历过 IAP 流程，允许直接跳转 */
        return 0;
    }

    /* 其它未知脏状态/异常值：保守处理，留在 Boot 升级模式 */
    return 1;
}

/* ---------- 跳转到 App ----------
 */
void IAP_JumpToApp(void)
{
    /* 读 App 入口：执行窗口向量表[0] = _start（链接于 0x00004000） */
    u32 appEntry = *(volatile u32 *)(APP_EXEC_START_ADDR + 4);

    /* 校验入口合法性：必须落在执行窗口 [0x00004000, 0x00010000) 内且半字对齐 */
    if (appEntry < APP_EXEC_START_ADDR || appEntry >= APP_EXEC_END_ADDR) return;
    if ((appEntry & 0x01) != 0) return;

    /* 关闭 BOOT 打开的外设（UART4 IDLE 中断 / RX DMA），
     * 否则跳到 App 后仍有中断源指向 BOOT 的 UART4_IRQHandler。
     */
    USART_ITConfig(UART4, USART_IT_IDLE, DISABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, DISABLE);
    NVIC_DisableIRQ(UART4_IRQn);
    DMA_Cmd(DMA1_Channel8, DISABLE);


    ERR_LED(1);   /* 诊断标记：灭 = 已执行跳转（若 App 崩溃，其 HardFault 会重新点亮） */

    NVIC_EnableIRQ(Software_IRQn);
    NVIC_SetPendingIRQ(Software_IRQn);

    while (1) { }   /* 不可达：软中断随即进入 SW_Handler */
}
