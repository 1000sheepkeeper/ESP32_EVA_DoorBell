# ESP32_EVA_DoorBell — ESP32 智能门铃系统

由 **两部 ESP32** 组成的低功耗智能门铃：
按下室外门铃端的按键，室内主机会播放门铃语音并在屏幕上显示敲门动画。

- **门铃从机**（`gatt_server_service_table/`）：安装在门外的门铃端，电池/低功耗供电。
  平时深睡眠省电，按键唤醒后开启 BLE 广播，通过 GATT 通知把"门铃事件"发给主机。
- **门铃主机**（`gatt_client/`）：安装在室内的显示端（ESP32-WROOM-32），
  作为 BLE 中心设备自动连接从机，收到通知后从 SD 卡播放音频（I2S）并在 ST7735 屏上显示动画。

两个目录是**相互独立**的 ESP-IDF 工程，各自拥有自己的 `sdkconfig` 与固件，分开编译、分开烧录。

## 目录结构

```
ESP32_EVA_DoorBell/
├── README.md                           # 本文件：整个项目的工作流程说明
├── gatt_server_service_table/          # 门铃从机工程（BLE GATT 服务表 + 低功耗）
│   ├── main/                           # 源码：属性表、门铃逻辑、深睡眠
│   ├── sdkconfig.defaults*             # 各芯片配置默认值
│   └── README.md                       # 从机详细说明
└── gatt_client/                        # 门铃主机工程（BLE 中心 + 音频/显示）
    ├── main/                           # 源码：GATT 客户端、媒体播放（SD/I2S/LVGL）
    ├── components/                     # 自带组件（LVGL 等）
    └── README.md                       # 主机详细说明
```

## 系统组成

| 部件 | 硬件 | 职责 |
| --- | --- | --- |
| 门铃从机 | ESP32 开发板 + GPIO33 按键（高电平有效，片内下拉） | 低功耗待机、按键唤醒、BLE 广播、发门铃通知 |
| 门铃主机 | ESP32-WROOM-32 + ST7735 屏（128×160，LVGL 9.3.0）+ I2S 功放（如 MAX98357A）+ FAT32 SD 卡 | 自动连接从机、播放 SD 音频、显示 GIF 动画、本地按键/串口触发 |

SD 卡根目录需放置：`idle.gif`（待机动画）、`knock.gif`（敲门动画）、`doorbell.wav`（门铃语音，16 位 PCM WAV）。

## 总体工作流程

```text
┌────────────────────┐                                  ┌────────────────────┐
│  门铃端（从机）      │                                  │  室内端（主机）      │
│  gatt_server_...   │                                  │  gatt_client       │
└─────────┬──────────┘                                  └─────────┬──────────┘
          │                                                       │
          │ ① 深睡眠待机（省电）                                    │ ① 上电: 初始化屏幕/SD/LVGL
          │                                                       │    播放 idle.gif 待机动画
          │ ② 有人按 GPIO33 按键 ──唤醒──> BLE 初始化               │ ② 初始化 BLE 主机
          │    并开始广播 (ESP_GATTS_DEMO)                         │
          │                                                       │ ③ 扫描到从机
          │                     ◄────── 连接 ──────                │    建立 BLE 连接
          │                                                       │ ④ 发现服务 0x00FF
          │ ③ 订阅使能 (CCCD 写 0x0100)                            │    订阅特征 0xFF01 通知
          │    立即发送通知 [0x01, 按键状态, 0, 0] ────►            │ ⑤ 收到通知 (首字节 0x01)
          │                                                       │    播放 knock.gif ×2
          │ ④ 断开/超时 ──关闭BLE──> 回到深睡眠                     │    + doorbell.wav(I2S)
          │                                                       │ ⑥ 播放结束 → 恢复 idle.gif
          │                                                       │ ⑦ 断开 → 重新扫描连接
          └──────────────────────────────────────────────────────────────┘
```

### 逐步说明

1. **从机低功耗待机**：门铃端平时处于深睡眠（RTC 外设 + GPIO33 高电平唤醒），耗电极低。
2. **按键唤醒**：按下门铃按键，从机被 GPIO33 唤醒；经 50ms 防抖确认后初始化 BLE 并开始广播。
3. **主机自动连接**：室内主机开机后初始化屏幕、SD 卡、LVGL 并播放待机动画，同时启动 BLE 扫描，
   发现广播名为 `ESP_GATTS_DEMO` 的从机后自动连接。
4. **订阅通知**：主机搜索服务 `0x00FF` 和通知特征 `0xFF01`，写入 CCCD 使能从机通知。
5. **门铃触发通知**：从机在通知使能后立即上报一次按键状态；
   之后每次按下 GPIO33 按键都会发送 4 字节通知 `[0x01, 电平, 0x00, 0x00]`。
6. **主机响应**：收到首字节为 `0x01` 的通知后，媒体任务从 SD 卡读取 `doorbell.wav` 经 I2S 播放，
   同时循环播放两轮 `knock.gif`；两者结束后恢复 `idle.gif` 待机动画。
7. **回到待机**：BLE 断开后，从机关闭蓝牙栈回到深睡眠；主机继续扫描并等待下一次连接。

### 主机端的本地触发（调试/备用）

除 BLE 门铃通知外，主机还支持两种本地触发方式，便于无门铃端时调试：

- **GPIO15 按键**：按下（高电平，50ms 软件防抖）触发门铃音频 + 敲门动画
- **串口触发**：在串口监视器输入任意字符即触发门铃音频 + 敲门动画

## BLE 协议约定

| 项目 | 值 |
| --- | --- |
| 从机广播设备名 | `ESP_GATTS_DEMO` |
| 服务 UUID | `0x00FF` |
| 通知特征 UUID | `0xFF01`（读/写/通知） |
| 通知数据 | `[0x01, GPIO33 电平, 0x00, 0x00]`（首字节 `0x01` = 门铃触发） |
| 射频功率 | 广播与链路均 +9 dBm |

## 构建与烧录

两个工程完全独立，分开编译：

```bash
# 通用环境（ESP-IDF v6.0.2）
source /home/sk/.espressif/v6.0.2/esp-idf/export.sh

# —— 门铃从机 ——
cd gatt_server_service_table
idf.py set-target esp32 && idf.py build
idf.py -p /dev/ttyUSB0 -b 115200 flash monitor

# —— 门铃主机 ——
cd ../gatt_client
idf.py set-target esp32 && idf.py build
idf.py -p /dev/ttyUSB0 -b 115200 flash monitor
```

> 注意：两个工程使用同一条串口时，需先烧录一台再切换到另一台；
> 从机与主机是两块不同的 ESP32 电路板。
> WSL2 + USB 串口直通环境请固定 `-b 115200` 波特率。

## 联调步骤

1. 主机 SD 卡放入 `idle.gif`、`knock.gif`、`doorbell.wav`（FAT32），上电预计先显示待机动画
2. 从机上电（或短按按键唤醒），主机日志出现 `Connected` 与 `Notification register successfully`
3. 按下门铃端 GPIO33 按键 → 主机播放敲门动画与门铃音 → 结束后回到待机动画
4. 观察从机串口：断开后应打印 `Entering deep sleep...`，再按按键可唤醒重启

## 环境与版本

- ESP-IDF **v6.0.2**（ESP32 rev v3.1，2MB flash）
- 开发环境：WSL2 + USB 串口直通（CP210x，`/dev/ttyUSB0`）
- 主机图形库：LVGL 9.3.0（`components/lvgl`）
