# CH32V203 IAP 项目文档（总入口）

## 概述

基于 WCH CH32V203（RISC-V，CH32V203C8T6，20KB RAM / 64KB Flash）的 IAP（在线升级）系统。
Bootloader + 两个 App 固件 + Python 上位机，通过 UART4（115200 8N1）完成 App 升级。

- 开发环境：MounRiver Studio（IDE）+ WCH 内置 GCC8 RISC-V 工具链（命令行脚本构建）
- 上位机：Python 3 + pyserial + PyQt5，PyInstaller 打包为单文件 exe
- FreeRTOS V10.4.6（GCC/RISC-V 端口）仅用于 `CH32V203_RTOS_APP` 工程

## 仓库结构

```
CH32V203_IAP/
├── CH32V203_BOOT/        # 纯 BOOT（裸机）：启动判定 IAP / 跳 App，IAP 协议栈执行方
├── CH32V203_APP/         # 裸机 App（1Hz LED + IAP 轮询支持，仅支持 GO_IAP，不写 Flash）
├── CH32V203_RTOS_APP/    # FreeRTOS App（IAP 协议栈 + start/led/usart4 任务）
├── iap_host/             # Python 上位机（GUI + 打包 + tools/ 调试脚本）
├── docs/
│   ├── protocol.md       # IAP 协议 v2 权威文档
│   ├── build-debug.md    # 项目专属构建/烧录/调试要点（通用工具链流程见全局 skill）
│   └── host.md           # 上位机使用 / 打包 / 脚本
└── 调试实录-2026-09-22-APP跳转卡死排查.md   # 历史记录
```

各工程目录内部结构相同：`Bsp/{inc,src}` / `Core/` / `User/` / `Peripheral/` / `Startup/` / `Ld/`。
仅 BOOT 与两个 App 工程含 IAP 协议栈 `bsp_iap.h/.c`；驱动文件为 `bsp_gpio` / `bsp_usart` / `bsp_iap`。

## 关键设计约定（踩坑记录，改动前必读）

1. **Flash 分区与执行窗口**：BOOT `0x08000000`；App 区 `0x08004000 ~ 0x08010000`
   （48KB 全为可写区，无保留页；Boot 侧另有升级状态标志页 `0x08003F00`，见第 5 条）；**执行窗口是别名地址 `0x00004000`**（CH32V203 代码必须在
   0x00000000 起别名窗口运行）。App 的 Link.ld（`APP_EXEC_START_ADDR`）与 Boot 的入口校验、
   跳转目标均用 `0x00004000`，不要与编程窗口混用。
2. **GO_IAP(0x21) 流程**：App 收到后先写 BKP 跨复位标志（DATAR1=0xB007 / DATAR2=0x4FF8
   互反双字），回 0xA1，延时 50ms 后 `NVIC_SystemReset`；Boot 启动时 `IAP_CheckBootFlag()`
   读取并清零，与 `IAP_EnterBootMode()`（App 入口无效 / `0x08003F00` 升级标志为 DIRTY 或未识别值）取或决定是否停留 IAP。
3. **角色隔离**：App 不写 Flash，擦写/跳转类命令统一回 NAK `ERR_ROLE_ERR 0x0B`；
   GET_INFO 角色标识 `...|APP:vX|...` / `...|BOOT:vX|...`，上位机靠此区分当前角色。
4. **跳转走软中断**：Boot 跳 App 统一经 `ch32v20x_it.c` 的 SW_Handler `jr`；
   不要在 main 直接 `jr` 或 `__disable_irq`（实测会引发 App 非法指令 → HardFault 死循环）。
5. **升级状态标志页 `0x08003F00`（2026-09-28 新增，防掉电变砖）**：Boot 区末页 256B；App 区仍无保留页、48KB 全可写。
   `ERASE_APP` 先写 `DIRTY 0x5A5A0001`；`JUMP_APP` 校验入口有效后写 `VALID 0x5A5A0002`；启动按「入口向量 + 标志」联合判定：
   DIRTY / 未识别 → 留在 IAP；VALID 或擦除态（`0xE339E339` / `0xFFFFFFFF`）+ 入口有效 → 跳 App（擦除态这条保证 WCH-Link/ISP 直烧后仍能启动）。
   Boot 只擦写「App 区 + Boot 区末页标志页」；**App 侧不写 Flash**。跳转/擦写期间内核取指挂起，会拖死调试器（OpenOCD `unable to resume`）。

6. **OpenOCD `program` 命令是整片擦除**（WCH 驱动 auto-erase，通用机制见全局 skill
   `ch32v20x-debug`）：BOOT.hex 与 App hex 地址段不重叠但**互斥**，任何顺序连烧两次都会
   导致"Verified OK 但不响应"（后烧的把先烧的分区擦成 `0xE339E339`）。
   - **正确做法**：BOOT 走 `program`（整片擦+烧），App 走 **IAP 升级通道**
     （`iap_host/tools/upgrade_headless.py`，只写 `0x08004000~0x0800FEFF`，不碰 BOOT）；
     `flash write_image`/`flash erase_address` 在 App 区不可用（报 `No flash at address 0x08004000`）。
   - 验证烧录结果必须 **dump Flash 与 elf 二进制逐字节比对**，不能只看 `Verified OK`。

详细机制与状态机见 `docs/protocol.md`（第 5、6、8 章）。

## 构建速查

```bash
# 三个工程统一走 MRS 生成的 obj/makefile（无独立 build 脚本）
cd CH32V203_BOOT/obj && export PATH="<WCH RISC-V 工具链>/bin:$PATH" && make -f makefile -B
# → obj/BOOT/CH32V203_BOOT.{elf,hex,bin}；APP / RTOS_APP 仅目录与产物名不同
python iap_host/iap_host.py          # 上位机 GUI；打包命令见 docs/host.md
```

通用工具链流程（探测/编译/烧录/GDB/排错速查表）→ 全局 skill `ch32v20x-debug`；项目专属内容（构建、D6 确认、引脚诊断）→ `docs/build-debug.md`。

## 文档索引

| 文档 | 内容 |
|------|------|
| `docs/protocol.md` | IAP 协议 v2 完整定义：帧格式、命令/错误码、Flash 分区、GO_IAP、角色隔离、排错清单（权威文档） |
| `docs/build-debug.md` | 项目专属构建/烧录/调试要点：构建脚本、D6 启动文件确认、串口引脚诊断（通用工具链流程见全局 skill `ch32v20x-debug`） |
| `docs/host.md` | 上位机 GUI 使用、PyInstaller 打包、tools/ 调试脚本 |
| `调试实录-2026-09-22-APP跳转卡死排查.md` | App 跳转卡死的历史排查记录 |

## BSP 编码规范

BSP 驱动统一放各工程 `Bsp/` 目录：

| 项目 | 说明 |
|------|------|
| `.h` / `.c` | 放 `Bsp/inc/`、`Bsp/src/`，命名 `bsp_xxx.h` |
| 头文件保护 | `#ifndef __BSP_XXX_H` |
| 引用 | `#include "bsp_xxx.h"`，编译参数含 `-I .../Bsp/inc` |
| 初始化 | `BSP_xxx_Init()` / `USART4_Init()` 等，在 `main()` 初始化阶段调用 |

### 文件编码

**项目所有源文件（`.c` / `.h` / `.py` / `.md`）统一为 UTF-8（无 BOM）**。
（2026-09-30 已完成编码统一：GBK 的 `bsp_gpio.*`、App/RTOS App 的 `User/main.c` 转 UTF-8；
带 BOM 的 `bsp_iap.c`、`docs/protocol.md` 去除 BOM。）
新增或修改文件一律保存为 UTF-8（无 BOM），不要顺手整文件转码（diff 会爆炸）。

## Git 规范

- `.gitignore` 排除 `obj/`、`build/`、`dist/`、`__pycache__/`、`*.wvproj`/`*.launch` 等 IDE 与构建产物
- 提交前确认编码：所有源文件（`docs/`、`iap_host/`、固件工程）均为 UTF-8（无 BOM）；勿整文件转码
- 修改 `bsp_iap.*` / `iap_host.py` 协议相关代码时，同步更新 `docs/protocol.md`
