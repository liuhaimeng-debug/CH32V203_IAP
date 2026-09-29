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
 * App 区 48KB（0x08004000 ~ 0x08010000）全为可写区，无保留页。：
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
#define ERR_ROLE_ERR          0x0B             /* 当前运行角色不支持该命令（如 Boot 收 0x21） */

/* ---------- CRC-16-CCITT 定义 ---------- */
#define CRC16_INIT            0xFFFF
#define CRC16_POLY            0x1021

/* ---------- Flash 升级状态标志（存放于 Boot 区末页 0x08003F00） ----------
 * 0x08003F00 ~ 0x08003FFF（256 字节），位于 BOOT 区（16KB）尾部。
 * BOOT 自身代码占用 ~12KB，留有 >3KB 裕量，且完全不占用 48KB App 空间。
 * 状态定义：
 *  - IAP_FLASH_ERASED_ECC (0xE339E339) / IAP_FLASH_ERASED_FF (0xFFFFFFFF)：
 *    出厂空片或调试器（WCH-Link/ISP）整片擦写态；若向量有效则直接允许跳转 App，
 *    保证离线/在线单步调试和直接烧录不受阻碍。
 *  - IAP_UPGRADE_DIRTY (0x5A5A0001)：
 *    升级脏标志。在 ERASE_APP 开始前/完成时写入。若写固件期间意外掉电，
 *    重启后 Boot 读到此标志强制留在 Boot，杜绝部分写入导致设备变砖卡死。
 *  - IAP_UPGRADE_VALID (0x5A5A0002)：
 *    升级完成标志。上位机全部固件写入、CRC 校验一致且收到 JUMP_APP 后写入。
 */
#define IAP_FLAG_PAGE_ADDR        0x08003F00UL
#define IAP_UPGRADE_DIRTY         0x5A5A0001UL
#define IAP_UPGRADE_VALID         0x5A5A0002UL
#define IAP_FLASH_ERASED_ECC      0xE339E339UL
#define IAP_FLASH_ERASED_FF       0xFFFFFFFFUL

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
uint8_t  IAP_CheckBootFlag(void);              /* 读+清 BKP 进 IAP 标志，命中返回 1 */
uint8_t  IAP_EnterBootMode(void);
void     IAP_JumpToApp(void);

#endif /* __BSP_IAP_H */
