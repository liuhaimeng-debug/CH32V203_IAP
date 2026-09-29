# iap_host 上位机使用指南

> 完整协议定义见 `docs/protocol.md`（权威文档）。

## 1. 文件布局

| 文件 | 说明 |
|------|------|
| `iap_host/iap_host.py` | 主程序：PyQt5 GUI + 协议收发（单文件，含 `normalize_app_image()` HEX 链接基址校验） |
| `iap_host/iap_host.spec` | PyInstaller 打包配置，产物 `dist/iap_host.exe`（单文件） |
| `iap_host/test_iap_host_rx.py` | 收发协议自检 |
| `iap_host/tools/probe_iap.py` | 原始字节抓包：逐帧分类 NO_SYNC/SHORT_HEAD 等，定位物理层问题 |
| `iap_host/tools/test_goiap.py` | GO_IAP(0x21) 复位进 IAP 测试 |
| `iap_host/tools/upgrade_headless.py` | 无 GUI 完整升级（擦除→写块→CRC→跳转），脚本化/CI 场景 |
| `iap_host/requirements.txt` | `pyserial`、`PyQt5`、`pyinstaller` |

## 2. GUI 使用

```
python iap_host/iap_host.py
```

- 串口区：COM 下拉 + 波特率（默认 115200）+ 刷新
- 升级流程：载入 HEX → START_DL → WRITE_BLK × N → CALC_CRC → JUMP_APP；
  收到 `ACK_PROGRESS 0xFE` 更新进度条（每 16 包一次）
- HEX 归一化校验（`normalize_app_image`）：拒绝链接基址不为 `0x08004000`（编程窗口）的映像，
  防止烧到错误 Flash 位置
- GO_IAP 按钮：向 App 发 0x21，设备写 BKP 标志并复位进 Boot，之后可在 Boot 模式下升级

## 3. 打包

```
cd iap_host
pyinstaller iap_host.spec        # 或 pyinstaller --onefile iap_host.py
# 产物：dist/iap_host.exe
```

## 4. tools/ 脚本用法

```
python iap_host/tools/probe_iap.py COM5        # 原始抓包诊断（需设备已在 IAP/正常模式）
python iap_host/tools/test_goiap.py COM5        # 验证 GO_IAP 复位进 IAP
python iap_host/tools/upgrade_headless.py COM5 完整映像.hex   # 无 GUI 升级
```

## 5. 排错速查

- 无响应/NO_SYNC → 先用 `probe_iap.py` 判断是物理层还是协议层；核对 UART4 115200 8N1、
  RS-485 方向引脚 PB10（低电平接收）
- NAK `0x0B` → 角色不符：擦写类命令发给了 App（App 不写 Flash，见 `docs/protocol.md` 角色隔离）
- 完整对齐校验清单 → `docs/protocol.md` 第 8 章「关键对齐校验点」
