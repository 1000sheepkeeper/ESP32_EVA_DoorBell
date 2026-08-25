# ESP32 门铃从机（BLE GATT 服务表）

> 本工程属于 **ESP32_EVA_DoorBell**（ESP32 智能门铃系统）的**从机（门铃端）**部分，
> 与 [../gatt_client](../gatt_client)（门铃主机）配套使用。

基于 ESP-IDF 官方 [gatt_server_service_table](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/ble/gatt_server_service_table) 示例改造而成，运行于 **门铃端的 ESP32**（电池/低功耗场景）。

核心思想：使用 **GATT 属性表（Attribute Table）** 一次性定义全部服务与特征，
通过 `esp_ble_gatts_create_attr_tab()` 创建，避免像 BLUEDROID 传统方式那样逐条添加属性。
本工程在官方示例基础上增加了：

- **门铃按键逻辑**：GPIO33 按键（高电平有效）触发门铃通知
- **低功耗管理**：广播超时 / 客户端断开后自动进入深睡眠，按键唤醒
- **按键防抖与误唤醒过滤**：唤醒后延时 50ms 确认按键仍按下，否则重新入睡

## 功能特性

| 特性 | 说明 |
| --- | --- |
| GATT 服务 | 服务 UUID `0x00FF`，含 3 个特征：A（`0xFF01`，读/写/通知）、B（`0xFF02`，读）、C（`0xFF03`，写） |
| 属性表 | `gatt_db[]` 一次性定义全部属性，`ESP_GATTS_CREAT_ATTR_TAB_EVT` 回调中获取句柄并启动服务 |
| 门铃通知 | 客户端使能 CCCD 通知后，服务器发送 4 字节通知数据，其中字节 1 为 GPIO33 按键实时状态 |
| 深睡眠 | 广播 30 秒无连接、连接断开、或初始化失败时，关闭 BLE 栈并进入深睡眠；GPIO33 高电平唤醒 |
| 按键防抖 | 唤醒后延时 50ms 校验按键电平，误唤醒自动回到深睡眠 |
| 广播 | 使用 RAW 广播数据（`ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT`），设备名 `ESP_GATTS_DEMO` |
| 其他 | 本地 MTU 500，发射功率 +9 dBm，支持 Prepare Write（长写） |

## 与主机的约定（BLE 协议）

| 项目 | 值 |
| --- | --- |
| 广播设备名 | `ESP_GATTS_DEMO` |
| 服务 UUID | `0x00FF` |
| 通知特征 UUID | `0xFF01` |
| 通知数据格式 | `[0x01, GPIO33 按键电平, 0x00, 0x00]`，首字节 `0x01` 表示门铃触发事件 |

主机端（`gatt_client`）扫描到该设备后，搜索服务 `0x00FF`、订阅特征 `0xFF01`，
收到首字节为 `0x01` 的通知即播放门铃音与动画。

## 硬件要求

- 支持 BLE 的 ESP32 系列开发板（ESP32 / C2 / C3 / S3 均已提供 `sdkconfig.defaults`）
- 按键接 **GPIO33**：按下为高电平，使用片内下拉（代码已配置 GPIO 内部下拉）
- 烧录用 USB 串口线

## 构建与烧录

```bash
# 1. 加载 ESP-IDF 环境（以 ESP-IDF v6.0.2 为例）
source /home/sk/.espressif/v6.0.2/esp-idf/export.sh

# 2. 选择目标芯片（按实际硬件选择）
idf.py set-target esp32           # esp32c2 / esp32c3 / esp32s3 ...

# 3. 编译
idf.py build

# 4. 烧录并监视串口
idf.py -p /dev/ttyUSB0 -b 115200 flash monitor
```

> 本机（WSL2 + USB 串口直通）烧录请固定使用 `-b 115200` 波特率。

## 使用说明（门铃工作流程）

1. 上电后设备初始化 BLE 并开始广播（设备名 `ESP_GATTS_DEMO`）
2. 门铃主机（或 nRF Connect 等手机工具）连接设备
3. 在特征 A（`0xFF01`）上**使能通知（CCCD 写 `0x0100`）**
4. 服务器立即发送通知：`[0x01, 按键状态, 0x00, 0x00]`
5. 按下 GPIO33 按键，再次发送通知，主机即可收到带按键状态的数据并触发门铃响应
6. 断开连接或广播 30 秒无人连接，设备自动进入深睡眠；再按一下按键即可唤醒重启

## 目录结构

```
gatt_server_service_table/
├── CMakeLists.txt                  # 工程构建配置
├── README.md                       # 本说明文件
├── sdkconfig.defaults              # 通用配置默认值
├── sdkconfig.defaults.esp32c2      # ESP32-C2 配置默认值
├── sdkconfig.defaults.esp32c3      # ESP32-C3 配置默认值
├── sdkconfig.defaults.esp32s3      # ESP32-S3 配置默认值
├── main/
│   ├── CMakeLists.txt              # 组件构建配置（依赖 bt/nvs_flash/esp_driver_gpio/esp_timer）
│   ├── gatts_table_creat_demo.c    # 全部源码：GATT 服务表、门铃逻辑、深睡眠管理
│   └── gatts_table_creat_demo.h    # 属性索引枚举等声明
└── tutorial/
    └── Gatt_Server_Service_Table_Example_Walkthrough.md  # 官方英文教程
```

## 关键代码位置

| 功能 | 位置 |
| --- | --- |
| GATT 属性表定义 | `main/gatts_table_creat_demo.c` 中 `gatt_db[]` |
| 服务/特征 UUID | 服务 `0x00FF`，特征 `0xFF01/0xFF02/0xFF03` |
| 门铃通知发送 | `ESP_GATTS_WRITE_EVT` 中 CCCD 写处理分支 |
| 深睡眠入口 | `enter_deep_sleep()` / `deep_sleep_task()` |
| 按键唤醒与防抖 | `app_main()` 中 `ESP_SLEEP_WAKEUP_EXT0` 分支 |
| 广播超时回睡 | `adv_timeout_cb()` |

## 注意事项

- `sdkconfig` 为各目标芯片编译生成的文件，不入库；芯片相关配置在 `sdkconfig.defaults.*` 中维护
- 修改按键引脚需同步修改 `GPIO_BUTTON` 宏与 `esp_sleep_enable_ext0_wakeup()` 的引脚参数
- 本工程仅支持 BLE，开机会调用 `esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)` 释放经典蓝牙内存
