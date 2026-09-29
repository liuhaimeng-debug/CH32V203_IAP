# CH32V203 IAP 通信协议与交互逻辑

> 本文档为 IAP 协议的**权威文档**（SSOT）：帧格式、命令/错误码、Flash 分区、CRC 算法、
> 各角色状态机、GO_IAP 与角色隔离机制。
> 配套工程：`CH32V203_BOOT`（Bootloader 固件）+ `iap_host/`（Python 上位机）。
> 上位机操作与打包见 `docs/host.md`；工具链/烧录/调试见 `docs/build-debug.md`。
> 协议相关代码（`bsp_iap.c`、`iap_host.py`）变更时必须同步更新本文件。

---

## 1. 通信物理层

| 项 | 值 | 说明 |
|---|---|---|
| 串口 | UART4 | PB0=TX(AF_PP)，PB1=RX(浮空)，PB10=485 方向控制 |
| 波特率 | 115200 | 8 数据位 / 无校验 / 1 停止位 |
| 接收 | DMA1 Channel 8 + IDLE 中断 | 双行环形缓冲（2×256B），累积解析 |
| 发送 | `Usart4_Send()` 逐字节轮询 TXE | 发送前后各 `Delay_Ms(5)` 切换 485 方向 |
| 单包数据上限 | `IAP_MAX_DATA_LEN = 272` B | WRITE_BLK 载荷数据实际最大 `256` B（整页，`FLASH_ROM_WRITE` 要求 256B 整数倍） |
| 超时 | `IAP_TIMEOUT_MS = 500` | Boot 侧单帧等待上限 |

---

## 2. 帧格式

### 2.1 请求帧（上位机 → Boot）

```
[5A A5] [LENH LENL] [CMD] [SEQ] [PAYLOAD...] [CRC_L CRC_H]
```

| 字段 | 字节 | 说明 |
|---|---|---|
| 同步头 | 2 | 固定 `0x5A 0xA5` |
| LENH LENL | 2 | `LEN = CMD(1) + SEQ(1) + PAYLOAD(len)`；高字节在前 |
| CMD | 1 | 命令码（见 §3） |
| SEQ | 1 | 序列号（Boot 仅回显，不强校验） |
| PAYLOAD | 0~n | 命令载荷 |
| CRC_L CRC_H | 2 | CRC-16，低字节在前 |

**CRC 覆盖范围**：从 `LENH` 起至 `PAYLOAD` 末尾，共 `4 + len(payload)` 字节。
（对应 Boot `USART4_DataPack_Process`：`crc = IAP_CRC16_Calc(buf+off-4, frame_len+2)`，`frame_len = 2+len`。）

**真实帧长度**：`2(sync) + 2(LEN) + 1(CMD) + 1(SEQ) + len(payload) + 2(CRC)` = `8 + len(payload)`。

### 2.2 响应帧（Boot → 上位机）

```
[5A A5] [LENH LENL] [CMD|0x80] [SEQ] [FLAG] [STATUS] [DATA...] [CRC_L CRC_H]
```

| 字段 | 字节 | 说明 |
|---|---|---|
| 同步头 | 2 | `0x5A 0xA5` |
| LENH LENL | 2 | `LEN = FLAG(1) + STATUS(1) + DATA(len)` |
| CMD | 1 | 请求命令码 `| 0x80`（ACK_PROGRESS/NAK 特殊，见 §3） |
| SEQ | 1 | 回显请求 SEQ |
| FLAG | 1 | `0x79`=ACK，`0x1F`=NAK |
| STATUS | 1 | ACK 时固定 `0x00`；NAK 时也置 `0x00`，错误码放 DATA |
| DATA | 0~n | 响应数据；NAK 时为 1 字节错误码 |
| CRC_L CRC_H | 2 | CRC-16，低字节在前 |

**CRC 覆盖范围**：从 `LENH` 起至 `DATA` 末尾，共 `2 + len(data) + 2` 字节（即 `crc_len + 4`，`crc_len = 2+len`）。

**真实帧长度**：`2(sync) + 2(LEN) + 1(CMD) + 1(SEQ) + len_val(FLAG+STATUS+DATA) + 2(CRC)` = **`8 + len_val`**。
> ⚠ 上位机曾在此处少算 2 字节（CMD+SEQ），导致 `ACK_VER` 等帧越界，已修正。
> ### 上位机 CRC 计算范围误区（2026-09-23 补充）
>
> 设备端校验 CRC 时只覆盖 **从 `LENH` 起至载荷末尾** 的字节，不包含同步头 `5A A5`。上位机若在构造请求帧或校验响应帧时，把 `5A A5` 也算进 CRC，会导致设备对每一帧的 CRC 必然不通过，设备静默丢帧，上位机表现为全部请求无应答（不是偶发超时，而是确定性失败），十分容易被误判为串口/硬件问题。
>
> **正确做法**：CRC 输入范围始终从 `LENH` 字节开始。例如上位机构造请求帧时的 CRC 输入是 `frame[2:]`，校验响应帧时的输入是 `frame[2:8+plen]`，均跳过前两个同步字节。
>
> **曾经的实例**：`iap_host/tools/upgrade_headless.py` 中的 `raw_query()` 曾经错误地把整帧（含 `5A A5`）传给 `crc16_ccitt()`，导致设备 100% 丢帧、上位机验证阶段全部返回 `NONE`。该问题已经修正（改为 `crc16_ccitt(fr[2:])`）。同类裸帧函数今后新增时，务必对齐这一点。
> ### 链接基址双窗口与上位机防御规则（2026-09-23 补充）
>
> CH32V203 的 App 区存在两个地址窗口：**执行窗口 `0x00004000`**（BOOT 跳转/代码执行用，属 `0x00000000` 别名区）与**编程/校验窗口 `0x08004000`**（IAP 烧写与 CALC_CRC 校验用）；两者指向同一物理 Flash 的不同别名。
>
> 上位机载入 `.hex` 后必须把映像归一化到编程窗口 `APP_START (0x08004000)` 再下发，且**仅接受这两种已知链接基址**（由 `iap_host.normalize_app_image()` 强制校验）。若误载链接基址既非 `0x00004000` 也非 `0x08004000` 的映像（例如 BOOT 映像、链接脚本写错的工程），旧逻辑的 `off = max(0, base - lo)` 会把映像前部静默截掉后写入 App 区，且主机/设备各自对「实际写入字节」算 CRC——**CRC 仍一致、界面全绿但内容错位**，属最难排查的一类事故。因此非法基址一律拒绝载入（抛 `ValueError`）。




### 2.3 CRC-16 算法

```
CRC-16-CCITT (FALSE)
  多项式 poly = 0x1021
  初值   init = 0xFFFF
  无输入/输出反转，无异或出
```

软件实现（`IAP_CRC16_Calc` / 上位机 `crc16_ccitt`）：
```c
crc = 0xFFFF;
for each byte b:
    crc ^= b << 8;
    for 8 bits:
        crc = (crc & 0x8000) ? ((crc<<1)^0x1021) : (crc<<1);
        crc &= 0xFFFF;
```

---

## 3. 命令码表

| 命令 | 请求码 | 响应码 | 载荷（请求） | 数据（响应） | 说明 |
|---|---|---|---|---|---|
| GET_INFO | `0x01` | `0x81` | — | 设备信息字符串 | Boot 回 `CH32V203C8T6\|BOOT:v1.0.3\|Flash:64KB`，App 回 `CH32V203C8T6\|APP:v1.0.0\|Flash:64KB`（上位机据此判角色） |
| GET_VER | `0x02` | `0x82` | — | `[主][次]` 2B | 协议版本，当前 `01 00` |
| ERASE_APP | `0x03` | `0x83` | — | — | 擦除 App 区，复位写指针 |
| START_DL | `0x10` | `0x83` | `[Addr:4B][Size:4B]` 大端 | `[Addr:4B][Size:4B]` 回显 | 声明下载区间，激活写指针 |
| WRITE_BLK | `0x11` | `0x91` | `[Addr:4B][Data:N]` | `[Addr:4B]` 回显 | 写一块，单包 N≤256 |
| CALC_CRC | `0x12` | `0x92` | — | `[CRC_L CRC_H]` | 计算已写区 Flash CRC |
| JUMP_APP | `0x20` | `0xA0` | — | — | 跳转 App（**不写 Flash**，见 §5） |
| GO_IAP | `0x21` | `0xA1` | — | — | **仅 App 支持**：写 BKP 标志 → 0xA1 → 50ms → `NVIC_SystemReset` 进 Boot IAP；Boot 收到回 NAK `0x0B`（已在升级模式） |
| 进度上报 | — | `0xFE` | — | `[written:3B]` 大端 | 每 16 包（1920B）主动上报一次，非回显 0x80 |
| NAK | — | `0xF0` | — | `[errcode]` 1B | 任意命令出错时返回 |

**START_DL 地址校验（严格幂等方案）**：
- 起始地址须 `∈ [APP_START_ADDR, APP_END_ADDR)`
- `size > 0`
- `addr + size ≤ APP_END_ADDR`（App 区 48KB 全为可写区，无保留页）
- 不满足 → NAK(`ERR_ADDR_OUT`)

**WRITE_BLK 写指针逻辑（严格幂等，支持重传）**：
- 必须先 START_DL，否则 NAK(`ERR_APP_NOT_READY`)
- 接收地址 `addr` 必须 == 当前写指针 `g_iap_dl_addr`，否则 NAK(`ERR_ADDR_OUT`)
- `plen ≤ 256`（整页）且 `g_iap_dl_written + plen ≤ g_iap_dl_size`；`START_DL` 的 size 按 256B 向上取整申报，末块 0xFF 补齐到整页
- `FLASH_ROM_WRITE` 要求 256B 整数倍长度 + 256B 对齐地址，故数据粒度固定 256B（`CHUNK=256`）
- Flash 写成功 → 推进写指针 `g_iap_dl_addr += plen`、回 ACK(回显 addr)
- Flash 写失败 → **写指针不推进**，回 NAK(`ERR_WRITE_FAIL`)；上位机重传同 `addr` 即幂等恢复

---

## 4. 错误码表

| 码 | 名称 | 含义 |
|---|---|---|
| `0x01` | ERR_UNKNOWN_CMD | 未知命令 |
| `0x02` | ERR_CRC_FAIL | 请求帧 CRC 校验失败（Boot 直接丢弃，不回 NAK） |
| `0x03` | ERR_ADDR_OUT | 地址越界 / 写指针错位 |
| `0x04` | ERR_LEN_ERR | 长度非法（>256B、超出 size、len<4 等） |
| `0x05` | ERR_ERASE_FAIL | 擦除失败 |
| `0x06` | ERR_WRITE_FAIL | Flash 写入失败 |
| `0x07` | ERR_SEQ_ERR | 序列号错误（预留） |
| `0x08` | ERR_BUF_OVERFLOW | 接收缓冲溢出 |
| `0x09` | ERR_TIMEOUT | 超时 |
| `0x0A` | ERR_APP_NOT_READY | 未 START_DL 就写块 / App 未就绪 |
| `0x0B` | ERR_ROLE_ERR | 当前运行角色不支持该命令（Boot 收 `0x21` / App 收擦写跳转类命令） |

---

## 5. Flash 分区（64KB 芯片 CH32V203C8T6，Flash 末 0x08010000）

```
0x08000000 ┌────────────────────────┐
           │   BOOT 代码（~12KB）   │  0x08000000 ~ 0x08003EFF
0x08003F00 ├────────────────────────┤  ← 升级状态标志页（256B，Boot 独占）
           │   升级状态标志 256B    │  0x08003F00 ~ 0x08003FFF
0x08004000 ├────────────────────────┤  ← App 的 .init（j handle_reset）
           │   App 可写区 48KB      │  0x08004000 ~ 0x0800FFFF  （全为可写区，无保留页）
0x08010000 └────────────────────────┘  APP_END_ADDR
```

> 历史上 App 区末页 0x0800FF00 的 256B 曾用作 AppReady 标志页（擦除/写入 0xA5A5A5A5），已废除。
> 现 App 区 48KB（0x08004000 ~ 0x0800FFFF）全为可写区，`START_DL` 校验上限为 `APP_END_ADDR`。
> **2026-09-28 起，升级状态标志改放 BOOT 区末页 `0x08003F00`**（256B，由 Boot 独占维护），
> 不占用 App 区任何字节（BOOT 代码约 12KB，距 0x08003F00 仍有 >3KB 裕量）。

**升级状态标志 `0x08003F00`（3 状态，防掉电变砖，2026-09-28 新增）**：

| 值 | 名称 | 写入时机 | 启动时含义 |
|---|---|---|---|
| `0x5A5A0001` | `IAP_UPGRADE_DIRTY` | `ERASE_APP(0x03)` 处理开头（擦 App 之前） | 上次升级未完成（擦/写中途掉电或中断）→ **留在 IAP** |
| `0x5A5A0002` | `IAP_UPGRADE_VALID` | `JUMP_APP(0x20)` 校验入口有效后、回 `0xA0` 之前 | 上次升级完整成功 → 允许跳 App |
| `0xE339E339` / `0xFFFFFFFF` | 擦除态 | 出厂空片 / WCH-Link·ISP 整片擦写后 | 未经历过 IAP 升级 → 入口向量有效即跳 App（兼容调试器直烧） |
| 其它任意值 | 未识别 | — | 保守判定 → **留在 IAP** |

写标志统一走 `IAP_FlashWriteFlag()`（擦 1 页 + 编程 1 页 + 回读校验），复用 App 区同一套带忙等待超时保护的擦写函数；
写标志失败时 `ERASE_APP` 直接回 NAK(`ERR_ERASE_FAIL`)，不进入擦除流程。

**进入 IAP 的条件**（`IAP_CheckBootFlag() || IAP_EnterBootMode()`，ERR_LED(PB13) 常亮进 IAP）：
0. **BKP 升级标志命中**（`IAP_CheckBootFlag`，读取即清零）：App 收 `GO_IAP(0x21)` 后写
   `BKP->DATAR1=0xB007 / DATAR2=0x4FF8`（互反双字，防随机值误判）→ 回 0xA1 → 50ms →
   `NVIC_SystemReset()`。标志仅跨软复位保持、断电丢失（自愈）；只对紧随的这一次启动生效。
1. **App 区无有效代码**：App 向量表[0] `*(0x00004004)`（= `_start`，执行窗口）不在
   `[0x00004000, 0x00010000)`（执行窗口）内，或最低位非 0。
   空片 / 被 `ERASE_APP` 擦除后该字读为 `0xE339E339`（ECC 擦除态，不是 0xFFFFFFFF），必然命中 → 继续停在 IAP。
2. **升级标志为 `DIRTY`（或未识别值）**（`IAP_EnterBootMode`，2026-09-28 新增）：入口向量虽然有效，
   但 `*(0x08003F00)` 表明上次升级没写完 → 继续停在 IAP（**防掉电变砖核心**：擦/写途中断电，
   重启后也不会去跑残缺 App，而是留在 IAP 等上位机重刷）。
3. 否则（标志为 `VALID`，或擦除态 + 入口有效）→ 跳转 App（PB13 灭，App 闪 RUN_LED(PB14)）。

> ⚠ **PA12（传感器）不再参与 IAP 判定**：它是板上传感器接口、另有用途（历史上曾用作
> "拉低进 IAP"的升级请求脚，已移除）。因此 **App 有效时复位/上电后 BOOT 会立即跳走**，
> 要重刷固件：上位机发 `GO_IAP(0x21)`（新版 App 支持，见 §3/§6），或先擦除 App 区（WCH-Link/ISP）。

> ⚠ **已废除的 AppReady 标志门控**：历史上在 Flash 末页 `0x0800FFC0` 写 `0xA5A5A5A5`
> 表示"App 就绪"，BOOT 靠它决定是否跳转。副作用致命，故删除：
> ① 擦/编程 Flash 期间 Flash 控制器独占总线、内核取指被挂起，与调试器（WCH-Link）
>    抢 Flash 会直接报 `Error: [wch_riscv.cpu.0] unable to resume` / `dmstatus=0x00000c82`；
> ② 用调试器或 ISP 直接烧录 App 时标志页仍是 `0xFF`，BOOT 会永远停在 IAP（App 永不运行）。
> 现在：BOOT 只在 IAP 会话内擦写 Flash（App 区 + Boot 区末页 `0x08003F00` 升级状态标志），跳转前后不碰 Flash；
> **App 侧完全不写 Flash**（进 IAP 标志走 BKP 寄存器，App 还应答查询与 GO_IAP）。


> ⚠⚠ **App 必须链接在执行窗口 0x00004000（不能是 0x08004000）**：CH32V203 的
> 代码只能在 0x00000000 起的执行别名窗口运行（0x0800xxxx 仅用于烧写/校验）。
> WCH 官方 EVT IAP 例程（USB_UART）的 APP 即链接在 0x00005000、SW_Handler 跳
>  x5000。曾把 App 链接在 0x08004000 并从 BOOT 跨窗口 jr 0x08004000，实测 APP
> 启动即挂死（errLED 长亮、runled 不闪、IAP 无响应）——CRC 校验能过因为 Flash
> 内容本身正确，坏的只是执行窗口。BOOT 的入口校验与跳转也必须用执行窗口。

**跳转 App**（`IAP_JumpToApp`）：
- 校验入口 `*(0x00004004) ∈ [0x00004000, 0x00010000)`（执行窗口）且 bit0 = 0
- 关 UART4 IDLE 中断 → 关 UART4 RX DMA → 关 UART4 NVIC → 关全局中断
- **必须用 WCH 官方软中断方式跳转**（`NVIC_EnableIRQ(Software_IRQn)` +
  `NVIC_SetPendingIRQ(Software_IRQn)` → `SW_Handler` 里 `jr 0x4000`）。
  ⚠⚠ **绝不能用普通 `jr` 从 main 直接跳**：BOOT 的 main 在 startup 的
  `mret`（mstatus.MPP=00）之后运行于**用户模式**，jr 不会切换特权级，App 启动里的
  `csrw mstatus/mtvec/mepc` 等 M 态 CSR 写会全部触发非法指令异常（GDB 实测
  mcause=2、mepc=App 的 csrw、mtval=对应指令字），经 BOOT 的 mtvec 落回
  HardFault → `NVIC_SystemReset` → 复位死循环（errLED 长亮、runled 不闪）。
- **不修改 sp / s0 / gp**：App 的 `handle_reset`（`startup_ch32v20x_D6.S`）自己会设置
  `sp = _eusrstack(0x20005000)`、`gp`、`mstatus = 0x1800`（MPP=11 机器模式）、`mtvec`，
  并搬运 `.data` / 清零 `.bss`。注意 `0x00004000` 处是 `j handle_reset` 指令，**不是栈顶**。
- ⚠⚠ **App 的 startup 里不得再写 CSR 0xbc0（流水线/预取配置）**：该寄存器复位后只允许
  写一次，BOOT 启动时已写过 0x1f；App 再写会非法指令（GDB 实测 mtval=0xbc029073）。
  APP1 的 startup 已注释掉这两行，禁止恢复。
- ⚠ **禁止用 `while(TC==RESET)` 代替发送尾延时**：CH32V203 的 TC 在"读 STATR 后再访问
  DATAR"时被硬件清除，而 UART4 的 RX 路径（DMA 读 DATAR）会随时清掉刚置位的 TC，
  造成 Boot 死等（实测复现），`Usart4_Send` 必须用固定 `Delay_Ms(5)` 尾延时。

---

## 6. 交互状态机（完整刷写流程）

```
上位机                          Boot
  │  连接串口                     │
  │  GET_VER ───────────────────► │  回 0x82 [主 次]
  │  GET_INFO ──────────────────► │  回 0x81 设备字符串
  │  ERASE_APP ─────────────────► │  擦 App 区 → 回 0x83
  │  START_DL(addr,size) ───────► │  校验区间 → 回 0x83，激活写指针
  │  WRITE_BLK(addr,chunk) ─────► │  addr==写指针 & Flash 写成功
  │                              │    → 回 0x91 [回显 addr]，推进写指针
  │                              │    每 1920B 额外回 0xFE 进度
  │    （逐包循环，至 size 写完） │
  │  CALC_CRC ──────────────────► │  计算已写区 Flash CRC → 回 0x92 [CRC_L CRC_H]
  │  JUMP_APP ──────────────────► │  回 0xA0 → 50ms 后跳转 App（不写 Flash）
  │
  │  上位机本地同样对 .hex 数据算 CRC-16，与 0x92 比对，一致才算成功
```

**从 App 运行态发起升级（GO_IAP 链路）**：

```
上位机                            设备（App 运行中）
  │ GET_INFO ────────────────────► │ 回 0x81 "...|APP:vX|..."   ← 识别为 App
  │ GO_IAP(0x21) ────────────────► │ 写 BKP 标志 → 回 0xA1 → 50ms → NVIC_SystemReset
  │  （等 ~0.6s，GET_INFO 探测）   │ BOOT 启动：IAP_CheckBootFlag 命中→清零→停在 IAP
  │ GET_INFO ────────────────────► │ 回 0x81 "...|BOOT:vX|..."   ← 确认进入 Boot
  │ （此后走上图既有升级流程）        │
```

**失败重传策略**：
- 单包 NAK → 上位机原地重试（同 addr，最多 3 次）；写指针未推进，重传幂等。
- 整段失败（CRC 不一致 / 地址错位）→ 上位机重新 ①擦除 → START_DL 重刷。

---

## 7. 上位机侧逻辑（iap_host.py）

| 模块 | 职责 |
|---|---|
| `IAP.request(cmd, payload)` | 构造请求帧 + 收 1 响应帧，返回 `(cmd,seq,flag,status,data)` |
| `IAP.check_ok(cmd, payload)` | 同上，但遇 NAK 直接抛 `RuntimeError(错误码翻译)` |
| `run_full_update(...)` | 擦除 → START_DL → 逐包写入(256B/包，整页) → CALC_CRC 比对 → JUMP_APP |
| `load_hex_to_app_image()` | 解析 Intel HEX，按 App 起始地址裁剪出待写映像 + 页数 |
| `normalize_app_image(image, lo)` | **2026-09-23 新增**：把 `load_hex_to_app_image()` 的结果归一到编程窗口 `APP_START`，并校验链接基址。仅接受两种已知链接基址——执行窗口 `APP_EXEC_BASE (0x00004000)` 与编程窗口 `APP_START (0x08004000)`；其余基址一律拒绝（防止非法基址映像被静默截断后写到 App 区，且因主机/设备各自对「实际写入字节」算 CRC 而导致 CRC 一致但内容错位的事故）。校验不过抛 `ValueError`。GUI 载入路径 `action()` 与无头升级 `upgrade_headless.py` 均改用此函数。 |
| `_read_response()` | 累积缓冲解析响应帧，`frame_len = 8 + len_val`（已修正越界 bug） |
| `App` (Tkinter) | GUI：串口连接 / 固件文件 / 查询状态（角色+设备信息+协议版本）/ 一键升级（自动 GO_IAP→擦写→CRC 校验→跳转）/ 进度条 / 日志 |

> 链接基址双窗口与防御规则详见 §2.2 末尾「链接基址双窗口与上位机防御规则」（2026-09-23 补充）。

**写包粒度**：`CHUNK = 256` B（整页），页数 = `ceil(app_size / 256)`，进度条按页显示。
上位机 `_read_response()` 会跳过 Boot 每 16 包主动上报的 `0xFE` 进度帧、丢弃 SEQ/CMD
不符的滞留旧帧与 CRC 坏帧，避免响应错位；WRITE_BLK 的 ACK 回显地址为 4 字节**大端**。
`request()` 发送前固定留 `TX_GUARD_S = 10ms` 的 RS-485 换向间隙（覆盖 Boot `Usart4_Send`
前后各 2ms 的方向延时 + 进度帧连发窗口 ≈7ms），防止请求帧撞进 Boot 发送窗口被
总线竞争打烂（Boot 对 CRC 不过帧静默丢弃，上位机表现为超时）；`check_ok` 对超时
自动原地重发（幂等命令，最多 3 次）；`jump_app` 收不到 0xA0 时以 GET_INFO 角色探测判定
"仍在 Boot"与"已跳入 App"。

---

## 8. 关键对齐校验点（排错清单）

1. **CRC 覆盖范围**：请求 `4+len(payload)`、响应 `2+len(data)+2` 字节，勿多勿少。
2. **响应帧长公式**：`8 + len_val`（含 CMD+SEQ 各 1 字节，历史上曾漏算）。
3. **字节序**：地址/长度字段在 START_DL/WRITE_BLK 载荷内为**大端**（高字节在前）；LEN 字段为高字节在前；CRC 为**低字节在前**。
4. **写指针严格相等**：`addr == g_iap_dl_addr`，否则 NAK；写失败不推进指针（幂等重传前提）。
5. **两块标志页各归其位（2026-09-28 更新）**：App 区**无保留页**（历史 App 区末页 `0x0800FF00` 的 AppReady 标志已废除，48KB 全可写）；BOOT 侧新增 **Boot 区末页 `0x08003F00`** 3 状态升级标志（`DIRTY 0x5A5A0001` / `VALID 0x5A5A0002` / 擦除态），只在 Boot 的 IAP 会话内被擦写，是「擦除 → 写入 → 跳转」全链路的防掉电变砖门控。
   是否跳转 App 由「BKP 升级标志 + App 入口有效性 + `0x08003F00` 标志状态」决定（§5），所以用 WCH-Link/ISP
   直接烧录 App 也能正常启动；而 App 侧任何 Flash 擦/编程都会与调试器抢 Flash 控制器，
   导致 `Error: [wch_riscv.cpu.0] unable to resume`（`dmstatus=0x00000c82`）。
6. **Flash 写特性**：CH32V203 Flash 带 ECC，**擦除态读回为 `0xE339E339`（不是 0xFFFFFFFF）**；
   写入只能 1→0，已编程字须先擦除才能再写。STATR 无 PGERR(0x04) 位（读出为脏值，禁止判定），
   有效标志仅 BSY(0x01)/WR_BSY(0x02)/WRPRTERR(0x10)/EOP(0x20)；擦除启动位 CR_STRT=0x00000040。
7. **请求帧解析容错**：帧可跨 DMA/IDLE 分片到达（解析器 4 状态机累积）；
   Boot 侧压缩 parse_buf 时 state 3 半帧必须保留 LENH 起的 4 字节帧头，否则 CRC 基准指针越界。
8. **BKP 进 IAP 标志**：`DATAR1=0xB007`/`DATAR2=0x4FF8` 必须成对写、互反校验；Boot **读取即清零**；
   读写前须使能 `RCC_APB1Periph_PWR|BKP` 时钟并 `PWR_BackupAccessCmd(ENABLE)`（PWR_CTLR_DBP）；
   ⚠ 硬件必测两点：`NVIC_SystemReset` 后 DATAR1/2 **保持**、断电后**丢失**。
   若实测被系统复位清零，则 BKP 方案失效，需回退 noinit RAM 方案。
