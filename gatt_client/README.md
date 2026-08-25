# ESP32 BLE 智能门铃主机

本工程基于 ESP-IDF 6.0.2，运行于 ESP32-WROOM-32 主机。程序通过 BLE
连接门铃从机，收到门铃触发通知后，从 SD 卡读取语音并通过 I2S 播放，
同时在 ST7735 屏幕上播放对应动画；无门铃事件时持续播放待机动画。

## 主要功能

- 自动扫描并连接设备名为 `ESP_GATTS_DEMO` 的 BLE 从机。
- 搜索服务 UUID `0x00FF` 和通知特征 UUID `0xFF01`。
- 自动写入 CCCD，开启从机通知。
- 收到首字节为 `0x01` 的通知后启动门铃媒体序列。
- 从 SD 卡播放 `doorbell.wav`。
- 门铃触发时播放两轮 `knock.gif`。
- 播放结束后自动恢复循环播放 `idle.gif`。
- 屏幕画面通过 ST7735 硬件寄存器旋转 180°。
- BLE 主动扫描、默认连接及当前连接链路均使用 `+9 dBm` 发射功率。
- BLE 断开后自动重新扫描并连接。

## 系统工作流程

```text
启动
  │
  ├─ 初始化 NVS
  ├─ 初始化 ST7735 显示屏
  ├─ 挂载 SD 卡到 /sdcard
  ├─ 启动 LVGL 并播放 idle.gif
  └─ 初始化 BLE 主机
       │
       ├─ 扫描 ESP_GATTS_DEMO
       ├─ 建立 BLE 连接
       ├─ 搜索服务 0x00FF
       ├─ 订阅特征 0xFF01
       └─ 等待通知
            │
            └─ 收到 0x01
                 ├─ 播放 knock.gif
                 ├─ 播放 doorbell.wav
                 └─ 两者结束后恢复 idle.gif
```

BLE 回调只负责将事件写入媒体队列。GIF 解码、SD 卡读取和 I2S 播放均在
独立任务中执行，不会在蓝牙回调中进行阻塞式文件操作。

## SD 卡文件

启动前需要在 SD 卡根目录放置以下文件：

```text
/idle.gif
/knock.gif
/doorbell.wav
```

文件用途：

| 文件 | 用途 |
| --- | --- |
| `idle.gif` | 无门铃事件时循环播放的待机动画 |
| `knock.gif` | 收到门铃事件后播放两轮的触发动画 |
| `doorbell.wav` | 收到门铃事件后播放的语音或提示音 |

### WAV 格式要求

- RIFF/WAVE 格式。
- 未压缩 PCM 编码。
- 16 位采样深度。
- 支持单声道或双声道。
- 采样率从 WAV 文件头自动读取。
- 双声道音频会混合为单声道，再复制到两个 I2S 时隙输出。

建议使用 FAT32 格式的 SD 卡，并尽量避免文件严重碎片化。如果文件缺失或
格式不受支持，程序会在串口日志中输出错误信息。

## 显示功能

- 显示控制器：ST7735。
- 分辨率：128 × 160。
- 色彩格式：RGB565。
- 图形库：LVGL 9.3.0。
- 背景图：编译时嵌入固件的 `background.rgb565`。
- GIF 文件：运行时从 SD 卡读取。
- 当前显示方向：相对原始方向旋转 180°。

## 音频功能

音频通过 ESP-IDF 标准 I2S 驱动输出，可连接 MAX98357A 等 I2S 数字功放。
程序会根据 WAV 文件采样率配置 I2S，并使用 DMA 输出音频。

播放过程如下：

1. 打开 `/sdcard/doorbell.wav`。
2. 解析 RIFF、`fmt ` 和 `data` 数据块。
3. 创建并配置 I2S 通道。
4. 从 SD 卡分块读取 PCM 数据。
5. 将音频写入 I2S DMA。
6. 播放完成后释放 I2S 通道并返回待机状态。

## BLE 配置

| 配置项 | 当前值 |
| --- | --- |
| 从机设备名 | `ESP_GATTS_DEMO` |
| 服务 UUID | `0x00FF` |
| 通知特征 UUID | `0xFF01` |
| 门铃事件值 | 通知首字节 `0x01` |
| 扫描方式 | 主动扫描 |
| 扫描间隔 | 50 ms |
| 扫描窗口 | 30 ms |
| 发射功率 | `+9 dBm` |
| 本地 MTU | 500 字节 |

## 分区与 Flash

LVGL、GIF 解码器和蓝牙协议栈会使固件超过默认的 1 MB factory 分区。
工程已启用 ESP-IDF 的大容量单应用分区：

- Flash 容量：2 MB。
- 应用分区：约 1.5 MB。
- 分区方案：Single factory app (large)。
- 不支持 OTA 双分区升级。

## 依赖组件

工程通过 `main/idf_component.yml` 使用以下托管组件：

```yaml
lvgl/lvgl: "9.3.0"
```

当 `/home/sk/espprj/tft` 项目存在时，工程也可以复用该项目已经下载的
LVGL 组件。

## 编译与下载

进入工程目录：

```bash
cd /home/sk/espprj/gatt_client
```

设置目标芯片并重新生成配置：

```bash
idf.py set-target esp32
idf.py reconfigure
```

编译固件：

```bash
idf.py build
```

下载并打开串口监视器：

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

实际串口设备名可能是 `/dev/ttyUSB0`、`/dev/ttyACM0` 或其他名称。
退出串口监视器使用 `Ctrl+]`。

## 常见问题

### 固件超过应用分区

确认以下配置已经生效：

```text
CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y
CONFIG_PARTITION_TABLE_FILENAME="partitions_singleapp_large.csv"
```

修改配置后执行：

```bash
idf.py reconfigure
idf.py build
```

### SD 卡无法挂载

- 检查 SD 卡是否为 FAT32。
- 检查 MOSI、MISO、SCLK 和 CS 接线。
- 确认 SD 卡模块工作电压与 ESP32 兼容。
- 所有模块必须共地。

### 有动画但没有声音

- 检查 `doorbell.wav` 是否为 16 位 PCM WAV。
- 检查 I2S 功放的 BCLK、LRC/WS 和 DIN 接线。
- 检查功放供电、扬声器和静音控制。

### 有声音但没有动画

- 检查 `idle.gif` 和 `knock.gif` 文件名及大小写。
- 确认 LVGL 文件系统配置的盘符为 `S:`，路径为 `/sdcard`。
- 检查串口中是否存在 GIF 加载失败信息。

### 找不到门铃从机

- 确认从机广播名称为 `ESP_GATTS_DEMO`。
- 确认从机服务 UUID 为 `0x00FF`。
- 确认主从机距离和供电正常。
- 检查主机日志中是否出现扫描启动或连接失败信息。

## 引脚分配表

以下引脚均为当前 ESP32 门铃主机的连接定义。所有外设必须与 ESP32 共地。

| 外设 | 信号 | ESP32 GPIO | 方向/说明 |
| --- | --- | ---: | --- |
| SD 卡 | MOSI | GPIO26 | 主机输出到 SD 卡 |
| SD 卡 | MISO | GPIO19 | SD 卡输出到主机 |
| SD 卡 | SCLK | GPIO25 | SDSPI 时钟 |
| SD 卡 | CS | GPIO27 | SDSPI 片选，低电平有效 |
| ST7735 | SCLK / SCL | GPIO14 | TFT SPI 时钟 |
| ST7735 | MOSI / SDA | GPIO13 | 主机输出到显示屏 |
| ST7735 | RST / RES | GPIO21 | 显示屏硬件复位 |
| ST7735 | DC / A0 | GPIO22 | 命令/数据选择 |
| ST7735 | CS | GPIO32 | 显示屏片选，低电平有效 |
| ST7735 | BL / LED | GPIO33 | 背光控制，高电平点亮 |
| I2S 功放/DAC | BCLK | GPIO4 | I2S 位时钟 |
| I2S 功放/DAC | LRC / WS | GPIO5 | I2S 左右声道/字选择时钟 |
| I2S 功放/DAC | DIN | GPIO18 | ESP32 I2S 音频数据输出 |
| 调试串口 UART0 | TX0 | GPIO1 | ESP32 默认串口日志输出 |
| 调试串口 UART0 | RX0 | GPIO3 | ESP32 默认串口输入/下载 |
