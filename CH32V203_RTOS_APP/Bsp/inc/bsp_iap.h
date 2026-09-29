/*
 * bsp_iap.h：IAP 通讯协议栈头文件（新协议 v2）
 * 适用于 CH32V203 系列，基于 UART4 + DMA 接收
 *
 * 帧格式（请求）：[5A A5][LENH LENL][CMD][SEQ][PAYLOAD...][CRC_L CRC_H]
 * 帧格式（响应）：[5A A5][LENH LENL][CMD|0x80][SEQ][FLAG][STATUS][DATA...][CRC_L CRC_H]
 * CRC16：CCITT-FALSE（poly 0x1021, init 0xFFFF），计算范围从 LEN 字节到 DATA 末尾
 */
#ifndef __BSP_IAP_H
#define __BSP_IAP_H

#include "ch32v20x_conf.h"

/* ---------- 角色：本工程为 App（应答查询 + GO_IAP 升级入口） ----------
 * 与 Boot 共用帧格式/解析器；擦写类升级命令一律 NAK(ERR_ROLE_ERR)。 */
#define IAP_ROLE_APP          1
#define APP_VERSION_STR       "v1.0.0"         /* App 应用版本（GET_INFO 上报） */

/* ---------- 工程相关的毫秒延时实现选择（bsp_iap.c 内使用） ----------
 * RTOS 工程（本工程）：SysTick 归 FreeRTOS 内核所有（CMP=SystemCoreClock/1kHz≈1ms），
 *   且 main() 中 Delay_Init() 被注释（p_ms=0）。debug.c 的 Delay_Ms 会
 *   ① 用 p_ms=0 算出错误目标值；② 直接改写 SysTick->CMP；③ 最后关闭 SysTick 计数
 *   → 内核节拍死亡、所有 vTaskDelay 永久阻塞（GO_IAP 后设备"无响应"的根因）。
 *   因此本工程所有 IAP 等待改用 vTaskDelay（1 tick = 1ms）。
 * 裸机工程（BOOT / CH32V203_APP）：不定义本宏，沿用 Delay_Ms（Delay_Init 已调用）。 */
#define IAP_DELAY_MS_IS_RTOS  1

/* ---------- Flash 分区定义 ---------- */
#define BOOT_START_ADDR       0x08000000
#define APP_START_ADDR        0x08004000        /* Flash 编程/校验窗口（烧写用） */
#define APP_END_ADDR          0x08010000        /* 64KB Flash 末（0x08000000 + 64KB） */
#define APP_EXEC_START_ADDR   0x00004000       
#define APP_EXEC_END_ADDR     0x00010000        /* 执行窗口末（= 0x08010000 别名） */

/* ---------- IAP 协议参数 ---------- */
#define IAP_FRAME_SYNC_H      0x5A             /* 同步头高字节 */
#define IAP_FRAME_SYNC_L      0xA5             /* 同步头低字节 */
#define IAP_MAX_DATA_LEN      272              /* 单包最大数据字节数（容纳 256B 数据载荷的整帧 LEN=262） */
#define IAP_CMD_WRITE_BLK_DATA_MAX 256          /* WRITE_BLK 数据最大 256B（整页，FLASH_ROM_WRITE 要求 256B 整数倍） */
#define IAP_TIMEOUT_MS        500

/* ---------- FLAG 定义 ---------- */
#define IAP_FLAG_ACK          0x79
#define IAP_FLAG_NAK          0x1F

/* ---------- Flash 分区说明 ----------
 * App 区 48KB（0x08004000 ~ 0x08010000）全为可写区，无保留页。
 */

/* ---------- 命令定义 ---------- */
#define CMD_GET_INFO          0x01
#define CMD_GET_VER           0x02
#define CMD_ERASE_APP         0x03
#define CMD_START_DL          0x10
#define CMD_WRITE_BLK         0x11
#define CMD_CALC_CRC          0x12
#define CMD_JUMP_APP          0x20
#define CMD_GO_IAP            0x21             /* 请求复位进 IAP；仅 App 执行，Boot 回 NAK 0x0B */

/* ---------- 响应命令 = 请求命令 | 0x80 ---------- */
#define ACK_INFO              (CMD_GET_INFO  | 0x80)   /* 0x81 */
#define ACK_VER               (CMD_GET_VER   | 0x80)   /* 0x82 */
#define ACK_OK                (CMD_ERASE_APP | 0x80)   /* 0x83 */
#define ACK_WRITE             (CMD_WRITE_BLK | 0x80)   /* 0x91 */
#define ACK_CRC               (CMD_CALC_CRC  | 0x80)   /* 0x92 */
#define ACK_JUMP              (CMD_JUMP_APP  | 0x80)   /* 0xA0 */
#define ACK_GO_IAP            (CMD_GO_IAP    | 0x80)   /* 0xA1 */
#define ACK_PROGRESS          0xFE                 /* 进度上报（特殊，不回显0x80） */
#define NAK_CMD               0xF0                 /* NAK 响应帧命令 */

/* ---------- 错误码 ---------- */
#define ERR_UNKNOWN_CMD       0x01
#define ERR_CRC_FAIL          0x02
#define ERR_ADDR_OUT          0x03
#define ERR_LEN_ERR           0x04
#define ERR_ERASE_FAIL        0x05
#define ERR_WRITE_FAIL        0x06
#define ERR_SEQ_ERR           0x07
#define ERR_BUF_OVERFLOW      0x08
#define ERR_TIMEOUT           0x09
#define ERR_APP_NOT_READY     0x0A
#define ERR_ROLE_ERR          0x0B             /* 当前运行角色不支持该命令（App 收擦写命令） */

/* ---------- CRC-16-CCITT 定义 ---------- */
#define CRC16_INIT            0xFFFF
#define CRC16_POLY            0x1021

/* ---------- BKP 跨复位“请求进 IAP”标志（App 写入，Boot 启动读取） ----------
 * DATAR1/DATAR2 互反双字，防上电随机值误判；读取即清零。
 * 备份域不随 NVIC_SystemReset 清零（仅备份域复位/掉电丢失）。 */
#define IAP_BKP_FLAG_MAGIC1   0xB007           /* BKP->DATAR1 */
#define IAP_BKP_FLAG_MAGIC2   0x4FF8           /* BKP->DATAR2 = ~MAGIC1 */

/* ---------- 函数声明 ---------- */
uint16_t IAP_CRC16_Calc(const uint8_t *data, uint16_t len);
void     IAP_SendFrame(uint8_t cmd, const uint8_t *data, uint8_t len,
                       uint8_t flag, uint8_t status);
void     IAP_SendNAK(uint8_t errcode);
uint8_t  IAP_ProcessFrame(uint8_t seqno, const uint8_t *data, uint16_t len);
void     IAP_SetBootFlag(void);                /* 写 BKP 进 IAP 标志（GO_IAP 升级入口） */
uint8_t  IAP_EnterBootMode(void);
void     IAP_JumpToApp(void);

#endif /* __BSP_IAP_H */
