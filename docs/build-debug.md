# CH32V203 项目专属构建 / 烧录 / 调试要点

> 通用工具链流程（MRS 工具链探测、`obj/makefile` 编译、OpenOCD 烧录、GDB 在线调试、
> 通用排错速查表）已由全局 skill `ch32v20x-debug` 覆盖
> （`~/.claude/skills/ch32v20x-debug/Skill.md`），本文档不再重复。
> 本文档只保留 CH32V203_IAP 项目专属内容。
> IAP 协议见 `docs/protocol.md`；上位机见 `docs/host.md`。

## 1. 本机环境

- MRS 安装根目录：`D:\MounRiver\MounRiver_Studio2`（工具链各二进制的相对路径见 skill 第 0 节）
- WCH VS Code 扩展：`C:\Users\liuha\.vscode\extensions\winerealms.wch-vscode-0.0.8\`
  - 功能：扫描 `.wvproj` 工程，支持 Build / Download / Debug
  - 用户配置（`settings.json`）：

```json
{ "wchVscode.mounRiverStudioPath": "D:\\MounRiver\\MounRiver_Studio2" }
```

## 2. 启动文件与芯片系列（重要）

**CH32V203C8T6 属于 D6 系列，必须用 `startup_ch32v20x_D6.S`，宏定义也必须是 `CH32V20x_D6`。**

| 启动文件 | 对应芯片 | 是否使用 |
|---|---|---|
| `startup_ch32v20x_D6.S` | CH32V203C6/C8/F6/F8/G6/G8/K8（64K Flash / 20K RAM） | 使用：本工程 BOOT / APP / RTOS_APP 都用这个 |
| `startup_ch32v20x_D8.S` | CH32V203RB（128K Flash，含 ETH/TIM5/OSC32KCal） | 排除 |
| `startup_ch32v20x_D8W.S` | CH32V208（含 BLE/ETH，向量表更长） | 排除 |

三者**向量表长度与中断号都不同**（`ch32v20x.h` 中 `CH32V20x_D6/D8/D8W` 分支）：

- D6：`UART4_IRQn = 61`、`DMA1_Channel8_IRQn = 62`（恰好接在 `USBFSWakeUp_IRQn = 60` 之后）
- D8：`ETH_IRQn = 61` … `UART4_IRQn = 66`、`DMA1_Channel8_IRQn = 67`

**混用的后果**（本项目实测）：用 D8 的启动文件 + D6 的头（或反之）会让
`NVIC_EnableIRQ(UART4_IRQn)` 指向错误的向量槽 → 串口 IDLE 中断不触发 → IAP 通信完全无响应；
同时 `HSE_VALUE` 分支也可能不同，串口波特率随之错误。

**如何确认工程用的是 D6**：

```powershell
# 1) 看编译产物里实际编进去的启动文件（应为 startup_ch32v20x_D6.o）
# 2) 看 .wvproj 的 excludeResources（BOOT 排除 D8/D8W → 只剩 D6）
Select-String -Path <proj>\<proj>.wvproj -Pattern 'startup'
# 3) 看宏定义：整个工程没有任何 -DCH32V20x_D8*
#    Peripheral/inc/ch32v20x.h 会在 CH32V20x_D6/D8/D8W 都未定义时自动 #define CH32V20x_D6
```

## 3. 项目构建

三个工程（BOOT / 裸机 App / RTOS App）**统一由 MounRiver Studio 生成 `obj/makefile`**，
没有独立的 `build_*.sh` 脚本。构建方式二选一：

### 3.1 命令行构建（已验证可行）

```bash
# 任一工程，进入 MRS 生成的 obj 目录
cd CH32V203_BOOT/obj
export PATH="<WCH RISC-V 工具链 bin 目录>:$PATH"   # 路径见 skill 第 0 节
make -f makefile -B                               # 产物 obj/BOOT/CH32V203_BOOT.{elf,hex,bin}
```

各工程仅目录与产物名不同：

| 工程 | obj 目录 | 产物 |
|------|----------|------|
| CH32V203_BOOT | `CH32V203_BOOT/obj` | `obj/BOOT/CH32V203_BOOT.{elf,hex,bin}` |
| CH32V203_APP | `CH32V203_APP/obj` | `obj/APP/CH32V203_APP.{elf,hex,bin}` |
| CH32V203_RTOS_APP | `CH32V203_RTOS_APP/obj` | `obj/CH32V203_RTOS_APP/CH32V203_RTOS_APP.{elf,hex}` |

注意事项：

- `makefile` 是 MRS 2.5.0 自动生成（`-include ../makefile.init`），**不要手改**；
  重新打开工程时 MRS 会重新生成。
- `makefile.init` 由 MRS 产出、未提交到仓库。若命令行直接 `make` 报缺文件，
  先在 MRS / VS Code 扩展里对工程点一次 Build，让 MRS 生成 `makefile.init` 与完整 `obj/`，
  之后即可脱离 IDE 重复命令行构建。
- `make -B` 强制全量重编；日常增量编译去掉 `-B` 即可。

### 3.2 IDE 构建

MounRiver Studio 扫描 `.wvproj` 工程，支持 Build / Download / Debug；
VS Code 扩展（`winerealms.wch-vscode`）同样调用 MRS Build（配置见第 1 节）。

工程编译参数（`-march=rv32im -mabi=ilp32 -Os`，
宏 `-DCH32V20x_D6 -DCH32V203_C8T6 -DUSE_STDPERIPH_DRIVER`）由 MRS 工程配置决定，
通用参数说明见 skill 第 1 节。

## 4. 烧录（项目分区策略）

通用 OpenOCD 烧录命令与陷阱（`program` 整片擦除、hex 路径风格、高地址段等）见 skill 第 2、4 节。
本项目多固件分区的关键约束：

- **BOOT 走 `program`**（整片擦 + 烧，hex 只覆盖 BOOT 区 `0x0000~0x2EFC`）；
- **App 走 IAP 升级通道**：`python iap_host/tools/upgrade_headless.py COM5 <hex>`
  （只写 `0x08004000~0x0800FEFF`，物理上不碰 BOOT）；
- 不要连续两次 `program`（互擦），也不要试图用 `flash write_image` 逐段烧 App——实测
  `flash erase_address` 在该地址报 `No flash at address 0x08004000`，不可用。

机制详解（整片擦除互斥、dump Flash 逐字节比对验证）见 `CLAUDE.md` 关键设计约定第 6 条。

## 5. 串口参数与诊断

- UART4 波特率 **115200**，8-N-1，DMA1 Channel8 循环接收 + IDLE 中断分行
  （`CH32V203_BOOT/Bsp/src/bsp_usart.c` `USART4_Init()`）
- 引脚：PB0 TX（AF_PP）、PB1 RX（FLOATING）、PB10 RS-485 方向控制（EN 低电平 = 接收）
- LED：PB13 灭 = 正常运行 / 亮 = IAP 模式；PB14 = App 1Hz 闪烁（App 在跑）

协议自检脚本（原始字节抓包、GO_IAP 测试、无 GUI 升级）见 `docs/host.md` 第 4 节。

## 6. 项目专属排错

通用排错速查表见 skill 第 4 节，以下为本项目专属条目：

| 问题 | 原因 | 解决 |
|------|------|------|
| 编译后 FLASH 溢出 | Link.ld 的 FLASH 区太小 | 检查 `Ld/Link.ld`，BOOT 区 16K 够用（当前 12.5K） |
| 写块后 Boot 无响应 | Flash 状态位残留 / DMA 时序 | 见 `bsp_iap.c` 中 `IAP_FlashErasePageSafe` 的注释 |
| 烧录 App 后 LED 不闪 | (1) App 未真正写入（verify/program 中断）(2) App 启动即崩（诊断版 HardFault 会**常亮 ERR_LED 并总等待**，不再复位）(3) BOOT 卡死在 `Usart4_Send` 的 TC 等待（历史 bug，已用固定尾延时修复） | 看 LED：PB13 灭 = IAP、PB13 亮且 PB14 闪 = App 在跑；确认 OpenOCD `Verified OK`。【注意】**PA12 不参与 IAP 判定**，App 有效时复位后 BOOT 直接跳走不停在 IAP —— 重刷前先用 WCH-Link/ISP 擦除 App 区 |
| `PermissionError(13)` 串口占用 | 其他程序占用串口（上位机侧） | 先关闭占用程序再测 |