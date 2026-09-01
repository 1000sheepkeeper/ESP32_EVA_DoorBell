# 智能门铃 BLE 卡死问题交接记录

更新时间：2026-09-01
适用工程：ESP-IDF v6.0.2、ESP32、Bluedroid BLE

本文用于把当前智能门铃项目交接给下一次代码修复与验证工作。项目由两个相互独立、但使用同一 BLE 协议的工程组成：

- `gatt_client/`：室内主机，作为 BLE GATT Client，负责扫描、连接、订阅通知，并驱动屏幕、SD 卡和 I2S 音频。
- `gatt_server_service_table/`：门铃从机，作为 BLE GATT Server，负责按键事件、广播和通知发送，并包含本地音频功能。

本记录只允许进行源码检查、静态检查和本地编译。不要执行 `idf.py flash`、`esptool`、`idf.py monitor` 或任何硬件下载/联调操作。

## 1. 原始问题

从机按下门铃按键并发送事件后，主机可以完成连接和部分 GATT 操作，但随后不再继续工作。主机串口曾出现如下日志：

```text
I (24665) DOORBELL_MASTER: Disconnected, remote b4:bf:e9:10:82:9a, reason 0x08
I (24675) DOORBELL_MASTER: Resuming scan...
I (24685) DOORBELL_MASTER: Scanning start successfully
I (24695) DOORBELL_MASTER: Doorbell device found: ESP_GATTS_DEMO
I (24695) DOORBELL_MASTER: Connecting to doorbell...
I (24705) DOORBELL_MASTER: Scanning stop successfully
I (48445) DOORBELL_MASTER: Connected, conn_id 0, remote b4:bf:e9:10:82:9a
I (48445) DOORBELL_MASTER: Connection TX power set to +9 dBm
I (48445) DOORBELL_MASTER: Open successfully, MTU 23
I (48445) DOORBELL_MASTER: Packet length update, status 0, rx 251, tx 251
I (48595) DOORBELL_MASTER: Connection params update, status 0, conn_int 32, latency 0, timeout 400
I (49475) DOORBELL_MASTER: Service discover complete, conn_id 0
I (49475) DOORBELL_MASTER: Service search result, conn_id = 0, is primary service 1
I (49475) DOORBELL_MASTER: Service found
I (49475) DOORBELL_MASTER: Service search complete
I (49485) DOORBELL_MASTER: Notification register successfully
I (49555) DOORBELL_MASTER: MTU exchange, status 0, MTU 500
I (49635) DOORBELL_MASTER: Descriptor write successfully (notify enabled)
```

`reason 0x08` 是 BLE supervision timeout，表示链路在规定时间内没有收到有效通信。它更像是前面 GATT 流程没有完成或回调任务被阻塞后的结果，不能单独当作根因。

上面是此前一轮测试的记录。本轮收到的完整启动及运行片段显示了更明确的现象：没有证据表明芯片在运行中复位；主机是在等待通知超过 15 秒后，按照当前恢复逻辑主动断开连接。详见第 9 节。此前的 `0x08` 仍应作为另一轮测试中的链路监督超时单独分析，不能与本轮的 `0x16` 混为一谈。

## 2. 已定位的首要根因

原从机代码使用 `saved_conn_id == 0` 判断“没有有效连接”。在 ESP-IDF Bluedroid 中，`conn_id = 0` 是合法连接 ID，而且通常就是第一次连接的 ID。日志已经明确显示主机连接 ID 为 0。

因此，在主机写入 CCCD 后，从机虽然已经收到“开启通知”，但通知发送条件仍然失败，逻辑上会跳过 `esp_ble_gatts_send_indicate()`。主机随后一直等待 `ESP_GATTC_NOTIFY_EVT`，从机和主机都没有完成预期的事件闭环，最终可能触发 supervision timeout。

原从机在 `ESP_GATTS_CONF_EVT` 中还直接执行 `vTaskDelay()`，随后调用关闭连接。GATT 回调运行在 Bluedroid 的事件任务中，在回调中阻塞会延迟其他 ATT、GAP 和断开事件，增加卡死和超时概率。

## 3. 当前已经完成的代码修改

以下内容已经写入工作区，但仍应由下一模型做一次完整代码审查。当前工作树本来就包含用户自己的配置、媒体和生成文件，不要用 `git reset` 或 `git checkout` 回滚它们。

### 3.1 从机 `gatt_server_service_table/main/gatts_table_creat_demo.c`

1. 增加 `INVALID_CONN_ID = UINT16_MAX`，不再把 0 当作无效连接 ID。
2. 增加 `ble_connected`、`saved_gatts_if`、`connection_generation` 等状态，发送和延迟关闭前检查连接代次、GATT 接口、连接 ID 与特征句柄，避免迟到回调操作新连接。
3. 将按键事件保存在 `pending_notify` 中。主机尚未连接或尚未写 CCCD 时先延迟通知并启动按需广播；CCCD 写成功后再发送待处理事件。
4. CCCD 写请求先返回 ATT response，再发送待处理通知，避免在写响应之前发送通知导致协议栈丢包。
5. 区分 notification (`0x0001`) 和 indication (`0x0002`)，保存 `notify_need_confirm`。
6. `ESP_GATTS_CONF_EVT` 不再 `vTaskDelay()`。使用 `esp_timer` 延迟约 200 ms 后异步关闭 GATT 连接，避免阻塞 Bluedroid 回调任务。
7. 对通知发送、CCCD response、服务启动、连接参数更新和广告状态增加日志及错误检查。
8. 当前代码还包含按需广播、GPIO 按键轮询、音频播放器和音量按键等改动；这些功能的硬件行为尚未验证。
9. `main/CMakeLists.txt` 已加入 `audio_player.c`、`esp_driver_i2s` 和 `esp_timer` 依赖。

关键位置（行号会随后续编辑变化）：

- 无效连接 ID与状态：约 80-124 行
- 广告重试与异步广告状态：约 297-635 行
- 异步通知关闭：约 335-403 行
- 通知发送条件：约 409-453 行
- CCCD 写处理：约 719-777 行
- 通知确认处理：约 789-815 行
- 连接/断开状态清理：约 819-887 行

### 3.2 主机 `gatt_client/main/gattc_demo.c`

1. 增加 `INVALID_CONN_ID = UINT16_MAX`，允许合法的 `conn_id = 0`。
2. 服务、特征、通知注册、CCCD 查找和 CCCD 写入失败时记录错误，并尝试断开/恢复扫描，而不是把 `connect` 永久留在 true。
3. 特征和描述符结果释放后显式置空，降低重复回调导致 double-free 或悬空指针的风险。
4. CCCD 显式写入小端字节 `{0x01, 0x00}`。
5. 通知事件校验链路状态、连接 ID、特征句柄和数据指针，忽略迟到或格式错误的通知。
6. 增加 `ESP_GATTC_CANCEL_OPEN_EVT` 和 `ESP_GATTC_CLOSE_EVT` 的处理日志。
7. 本轮后续又加入扫描状态枚举（`IDLE/STARTING/ACTIVE/STOPPING`）、等待停止完成后再发起连接、扫描失败重试定时器，以及连接建立/GATT 配置超时定时器。
8. 主机通知回调只把事件放入媒体队列；SD 卡文件读取、WAV 解析、I2S 和 LVGL 操作在独立任务执行。

关键位置：

- 扫描与连接状态变量：约 54-82 行
- 扫描重试、连接超时和失败恢复：约 157-436 行
- GATT 连接、发现、订阅和通知：约 438-845 行
- GAP 扫描启动/停止和连接发起：约 848-995 行

## 4. 当前已完成的本地验证

### 4.1 主机工程

已在 ESP-IDF v6.0.2 环境中执行：

```bash
cd /home/sk/espprj/gatt_server_service_table/gatt_client
source /home/sk/.espressif/v6.0.2/esp-idf/export.sh
IDF_COMPONENT_MANAGER=0 idf.py build
```

结果：2026-09-01 再次编译、链接、生成 ESP32 binary 和分区大小检查均成功。构建输出：

```text
build/gatt_client.bin
binary size 0x109180
smallest app partition 0x177000
free 0x6de80 (29%)
Project build complete.
```

本次主机 ELF SHA256：

```text
b272af8f3cd3868dd180d26f06dbe7a0ef41f91d67aaba5aef7fde4f4e4b6805  build/gatt_client.elf
```

构建过程中出现若干 ESP-IDF Kconfig 的非致命 NOTE（旧配置中 bool 默认值为数字 0），没有导致构建失败。

### 4.2 从机工程

本次直接在从机原工程执行 `idf.py build` 时，只读的 `build/log` 目录导致 idf.py 无法写入日志并以 exit code 2 退出；输出中没有出现源码 compiler error。随后使用独立临时构建目录执行：

```bash
cd /home/sk/espprj/gatt_server_service_table/gatt_server_service_table
IDF_COMPONENT_MANAGER=0 idf.py -B /tmp/gatt_server_service_table_build_20260901 build
```

结果：编译、链接、生成 binary 和分区大小检查均成功。构建输出为 `gatt_server_service_table.bin`，binary size `0xba920`，最小 app 分区 `0x100000`，剩余 `0x456e0`（27%）。临时构建 ELF SHA256：

```text
ef08a4a728450b4c68fbe1d8bb0b7822be76b54a82e01a945a5b866d52426a1c  /tmp/gatt_server_service_table_build_20260901/gatt_server_service_table.elf
```

构建过程只有 ESP-IDF Kconfig 的非致命 NOTE（例如旧 bool 默认值和不可见配置项），没有 compiler error、linker error 或分区超限。

原工程的只读 `build/` 未删除、未覆盖；临时目录仅用于软件编译验证。

### 4.3 静态检查

当前 `git diff --check` 已通过，未发现空白错误。

### 4.4 明确未做的事情

- 没有烧录任何 ESP32。
- 没有打开串口 monitor。
- 没有执行真实 BLE 连接、按键、广播、通知、断开或 supervision timeout 联调。
- 没有验证实际 GPIO、ST7735、SD 卡、MAX98357A/I2S 接线和音频播放。

## 5. 尚未完成或需要优先复核的部分

### 5.1 主机扫描状态机仍需严格审查

当前新增的状态机比原始代码更完整，但它涉及多个异步事件，必须确认所有转换都能回到可扫描状态：

1. `esp_ble_gap_start_scanning()` 和 `esp_ble_gap_stop_scanning()` 都是异步 API。只能在对应 `*_COMPLETE_EVT` 成功后把状态视为真正启动或停止。
2. 扫描启动 API 或 `SCAN_START_COMPLETE_EVT` 失败时，必须留下可重试路径，不能只设置一个永远无人消费的 pending 标志。
3. 命中设备后应停止扫描，等待 `SCAN_STOP_COMPLETE_EVT`，再调用 `esp_ble_gattc_enh_open()`；不要在扫描仍可能活动时并发发起连接。
4. 停止扫描失败时要避免 `scan_state` 假装为空闲，也不要让 `connect=true` 与 `scan_active=true` 同时永久残留。
5. `CONNECT_EVT`、`OPEN_EVT`、`CANCEL_OPEN_EVT`、`CLOSE_EVT`、`DISCONNECT_EVT` 可能迟到或顺序交错。每个回调都要用当前尝试的远端地址、连接 ID和状态核对，不能让旧事件清除新连接。
6. `enh_open` 没有在合理时间内产生结果时，需要调用 `esp_ble_gattc_cancel_open()` 或物理断开，并有超时兜底恢复扫描。
7. 若收到 `CLOSE_EVT` 但没有 `DISCONNECT_EVT`，应确认是否仍需清理状态并重新扫描；不能只打印日志后留下卡住状态。
8. 检查定时器的创建、停止和回调线程安全性，确保 reset 时不会释放仍被回调使用的特征/描述符内存。

建议下一模型先画出下面这条单向流程，再逐个对照代码：

```text
SCAN_IDLE
  -> SCAN_STARTING
  -> SCAN_ACTIVE
  -> SCAN_STOPPING
  -> OPEN_PENDING / OPEN_IN_FLIGHT
  -> CONNECTED
  -> DISCOVERING / SUBSCRIBED
  -> NOTIFY_RECEIVED
  -> DISCONNECT or failure recovery
  -> SCAN_IDLE
```

任意失败都必须最终进入 `SCAN_IDLE` 后通过一次受控的 `start_scanning` 重试，不能在回调中递归快速重试。

### 5.2 从机广播和连接关闭状态仍需复核

1. `adv_state` 当前应只在 `ESP_GAP_BLE_ADV_START_COMPLETE_EVT` 成功回调后进入 `ADV_ACTIVE`；广告启动 API 返回成功本身不等于已经在播发，失败或丢失完成事件时必须仍有重试路径。
2. 广告停止完成事件可能与新的按键事件交错，要确认不会把刚启动的广告误清为 inactive。
3. `pending_notify`、`notification_sent` 和 `notify_close_pending` 是单事件模型。连续快速按键是否允许覆盖、合并或丢弃事件，需要明确协议预期。
4. 非确认 notification 的 `ESP_GATTS_CONF_EVT` 在 Bluedroid 中可能表示发送结果，而不是对端 ATT 确认；当前延迟关闭设计应保持短且非阻塞，并处理失败状态。
5. 连接断开时必须停止通知关闭定时器、清除连接 ID 和 GATT 接口，并避免旧定时器关闭新连接。
6. 当前代码已从原来的深睡眠/GPIO33 设计改成按需广播/GPIO16 加本地音频。必须确认这是最终硬件方案；如果最终仍需要深睡眠，应重新设计而不是只修改注释。

### 5.3 文档与代码的硬件约定不一致

仓库中的旧 README 仍有 GPIO33、深睡眠等描述，而当前从机代码使用 GPIO16 作为门铃键、GPIO17/15 作为音量键，并保持 BLE 运行。下一模型应在确认代码目标后统一：

- `gatt_server_service_table/README.md`
- 仓库根目录 `README.md`
- 两端源码中的 GPIO、广播和低功耗注释

这项工作可以只改文档和注释，但不能在没有明确硬件方案时擅自改变协议或引脚。

### 5.4 媒体功能只完成编译验证

主机的 `doorbell_media.c`、从机的 `audio_player.c` 已纳入工程并能参与编译，但以下内容仍没有硬件证据：

- SD 卡挂载、`idle.gif`/`knock.gif`/`doorbell.wav` 文件读取；
- LVGL 绘制和 ST7735 方向；
- I2S 时钟、MAX98357A 输出、音量和 WAV 格式兼容性；
- BLE 通知与媒体任务并发时的内存和栈余量。

下一模型只能做代码审查和编译，不应把这些功能描述成已经实机验证。

## 6. 建议的下一步修复顺序

### 第一步：确认工作树和目标协议

```bash
cd /home/sk/espprj/gatt_server_service_table
git status --short --untracked-files=all
git diff -- gatt_client/main/gattc_demo.c gatt_server_service_table/main/gatts_table_creat_demo.c
```

保留用户已有的配置、音频、GIF、脚本和 build 产物，不执行 destructive git 命令。先确认最终设计是“从机常驻 BLE/按需广播”还是“按键唤醒后深睡眠”，再决定是否需要恢复低功耗代码。

### 第二步：收敛主机状态机

1. 统一一个扫描状态变量和一个连接尝试对象，记录远端地址、是否已有物理链路、是否已有 GATT 虚拟连接。
2. 所有扫描启动/停止 API 都检查同步返回值，并只在完成回调中推进状态。
3. 命中目标后只做一次 stop 请求；stop 成功后才 open。
4. 为 open、服务发现、CCCD 写入设置有限超时；超时执行 cancel/断开，随后通过一个入口恢复扫描。
5. 对每个迟到回调校验远端地址和 conn_id；不要用单纯的 bool 清除新连接状态。
6. 对 `DISCONNECT_EVT`、`CANCEL_OPEN_EVT`、`CLOSE_EVT` 和扫描完成事件写出明确的恢复表，并避免回调互相重复启动扫描。
7. 如果当前状态机过于复杂，优先保留一个简单、串行、可证明的实现，再加入有限重试，而不是继续堆叠标志位。

### 第三步：复核从机通知闭环

1. 搜索所有 `saved_conn_id`、`conn_id == 0` 和 `INVALID_CONN_ID` 使用，确认 0 只被当作合法值。
2. 确认 CCCD 的 0x0001/0x0002/0x0000 解析和写响应都正确。
3. 确认 `send_doorbell_notify()` 在 `ble_connected && notify_enabled` 时一定调用 `esp_ble_gatts_send_indicate()`，并记录返回值。
4. 确认发送失败会保留事件或按协议丢弃，且不会在 GATT 回调中阻塞。
5. 确认通知完成后的异步关闭只作用于同一连接代次；断开路径能停止定时器。
6. 检查广告启动失败、停止失败和按键重复触发时的恢复行为。

### 第四步：只做本地软件验证

先使用当前构建目录做普通增量编译；如果需要验证干净构建，复制到 `/tmp`，不要删除用户的 `build/`：

```bash
source /home/sk/.espressif/v6.0.2/esp-idf/export.sh

cd /home/sk/espprj/gatt_server_service_table/gatt_client
IDF_COMPONENT_MANAGER=0 idf.py build

cd /home/sk/espprj/gatt_server_service_table/gatt_server_service_table
IDF_COMPONENT_MANAGER=0 idf.py build
```

如果从机原 `build/` 因权限或历史生成文件失败，使用临时副本重新配置和编译。编译失败时优先修复真正的首个 compiler error；连续多次失败且问题变成环境/权限问题时停止，不要继续尝试下载。

每次代码修改后至少执行：

```bash
git diff --check
```

并检查：

- 两个工程都完成 C 编译、链接和 binary 生成；
- 没有新的未处理 compiler error；
- 应用分区大小检查通过；
- 关键 API 的返回值和类型与 ESP-IDF v6.0.2 头文件一致；
- 没有遗留 `conn_id == 0` 的无效判断、GATT 回调中的长时间阻塞或重复 free。

## 7. 软件验证时应观察的预期日志

硬件联调由用户后续完成；下一模型只能把这些作为代码设计目标，不能声称已经观察到：

### 主机目标路径

```text
Scanning start successfully
Doorbell device found: ESP_GATTS_DEMO
Scanning stop successfully
Connection request submitted
Connected, conn_id 0, ...
Open successfully, ...
Service discover complete, ...
Service found
Notification register successfully
Descriptor write successfully (notify enabled)
Notification received (doorbell trigger)
Disconnected, ...
Resuming scan...
```

### 从机目标路径

```text
Doorbell pressed ...
advertising start successfully
ESP_GATTS_CONNECT_EVT, conn_id = 0
notify enable
Delivering deferred doorbell notification
Doorbell notification sent to host
ESP_GATTS_CONF_EVT, status = 0, conn_id 0, ...
Notification delivered; closing connection 0
ESP_GATTS_DISCONNECT_EVT, ...
```

日志中的具体时间和 reason 可能因环境不同而变化；重点是 CCCD 写入后必须出现通知发送和主机 `ESP_GATTC_NOTIFY_EVT`，并且连接关闭不能依靠阻塞回调完成。

## 8. 交接时的边界

- 本文记录的是当前工作区状态，不是硬件测试报告。
- 编译成功只说明源码、依赖和链接关系在当前配置下成立，不代表 BLE 时序、射频链路、按键电平或音频硬件已经正确。
- 不要清理或回滚用户已有的 dirty worktree 文件。
- 不要执行下载、烧录、串口监视或网络依赖安装。
- 完成下一轮代码修改后，应把实际修改、编译命令、编译结果和剩余风险追加到本文或最终回复中。

## 9. 2026-09-01 最新主机日志分析

### 9.1 关键时间线

以下时间均为主机串口日志中的毫秒时间戳，时间原点是本次上电后的启动日志。

| 时间 | 日志/状态 | 含义 |
| ---: | --- | --- |
| 0 | `rst:0x1 (POWERON_RESET)` | 本次串口片段开头的上电复位；不是片段后半段发生的复位。 |
| 1,837 | 扫描启动成功 | 主机进入正常扫描。 |
| 1,867 | 找到 `ESP_GATTS_DEMO`，提交连接 | 发现从机并停止扫描，开始连接尝试。 |
| 2,907 | `Disconnected ... reason 0x3e` | 第一次连接建立失败，进入重扫。 |
| 2,957 | 找到设备并提交第二次连接 | 第一次失败后的再次扫描/连接尝试。 |
| 3,837 | 再次 `reason 0x3e` | 第二次连接建立失败，仍属于连接尝试失败。 |
| 7,547 | `Connected, conn_id 0` | 第一次成功建立链路；`conn_id = 0` 是合法值。 |
| 8,577-8,597 | 服务发现、特征找到、通知注册成功 | GATT 服务和通知特征已找到，客户端已注册通知。 |
| 8,657 | MTU exchange 成功 | MTU 协商完成。 |
| 8,817 | `Descriptor write successfully (notify enabled)` | CCCD 写入成功，主机请求从机启用 notification。仅表示订阅配置完成。 |
| 22,547 | `BLE setup timed out; cancelling connection attempt` | 从 `Connected` 起正好约 15,000 ms；当前主机计时器到期。 |
| 22,577 | `Disconnected ... reason 0x16` | 主机超时回调主动断开后收到的物理断链事件。 |
| 26,847 | 第二次成功 `Connected, conn_id 0` | 重扫后重新建立链路。 |
| 28,027 | 第二次 CCCD 写成功 | 再次完成订阅，但仍没有收到通知。 |
| 41,847 | 第二次 `BLE setup timed out` | 同样是 `26,847 + 15,000 ms`。 |
| 41,867 | 第二次 `reason 0x16` | 同样是本地主动断开。 |
| 71,897、101,907、131,917 | `Scan timeout, restarting...` | `SCAN_DURATION_S = 30` 的扫描周期结束后重新开始扫描；不是 MCU 重启。 |

### 9.2 为什么主机会“卡一段时间”

当前主机在 `main/gattc_demo.c` 中使用 `CONNECTION_SETUP_TIMEOUT_US = 15 s`。计时器会在命中设备准备连接时启动，并在 `CONNECT_EVT`/`OPEN_EVT` 中重新启动；当前 `ESP_GATTC_WRITE_DESCR_EVT` 成功处理只打印 CCCD 成功日志，代码注释明确选择“保持计时，直到实际收到通知”。只有合法的 `ESP_GATTC_NOTIFY_EVT`、且 payload 首字节为 `0x01` 时才调用 `disarm_connection_timeout()`。

因此，日志所示行为是：

1. 主机完成物理连接、服务发现、通知注册和 CCCD 写入。
2. 主机没有看到有效的门铃通知，于是继续等待，而不是立即播放或断开。
3. 15 秒计时器到期，`connection_timeout_timer_cb()` 发现 `link_up == true`，调用 `esp_ble_gap_disconnect()`。
4. 断开完成后主机恢复扫描。

这段等待期看起来像“卡死”，但从日志看 CPU 仍在运行，且状态机正在按设计等待超时。超时回调发起断链后通常会等待最多 `CONNECTION_ABORT_GRACE_US = 2 s` 的清理事件；本次两次测试都在约 30 ms 内收到断链回调，所以没有实际等待满 2 秒。日志文案 `BLE setup timed out; cancelling connection attempt` 对已经建立的链路并不准确：此分支实际执行的是主动断链，不是取消尚未建立的连接。下一轮代码修复应把日志改成能区分 `open` 超时与“已订阅但未收到事件”的超时，避免误判。

### 9.3 为什么 `Descriptor write successfully` 后仍没有响应

CCCD（Client Characteristic Configuration Descriptor）写成功只证明 ATT 写操作完成，不能证明从机已经发送通知，也不能证明主机已经收到通知。本片段中完全没有：

```text
Notification received (doorbell trigger)
Indication received (doorbell trigger)
Doorbell pressed; queued SD audio and knock animation
```

所以至少在这份主机日志覆盖的时间段内，主机没有接受到有效的 `ESP_GATTC_NOTIFY_EVT`。这里要区分两个层面：连接、MTU、服务发现和 CCCD 写入属于 BLE 控制面；门铃特征值 `0xFF01` 的 `0x01` notification 才是数据面事件。用户所说的“收到信号/部分响应”可能只是控制面已经完成，不能替代数据面通知证据。仅凭主机日志还不能区分下面两种情况：

- 从机没有调用 `esp_ble_gatts_send_indicate()`，或调用失败；
- 从机已经发送，但通知句柄、CCCD 类型、连接代次或链路时序导致主机丢弃/过滤。

必须同时取得从机同一轮测试的串口日志，重点搜索 `ESP_GATTS_CONNECT_EVT`、`notify enable`、`Delivering deferred doorbell notification`、`Doorbell notification sent to host`、`Failed to send notification` 和 `ESP_GATTS_CONF_EVT`。如果从机日志没有 `Doorbell notification sent to host`，优先检查从机是否实际刷入了包含 `INVALID_CONN_ID = UINT16_MAX` 修复的版本，以及按键事件是否设置了 `pending_notify`。如果从机显示发送成功而主机仍无通知，再检查两端连接地址、特征值句柄和通知/indication 属性是否一致。

用户描述“主机能接收到信号并做出部分响应”与本片段并不完全一致：日志没有 `Notification received`、`Doorbell pressed; queued...` 或媒体任务日志。可能是截取时遗漏了相关行，也可能是主机本地 GPIO15/串口触发了媒体功能而不是从机 BLE 通知。下一轮应把主机和从机完整日志按同一时间段保存，不能用“看到屏幕/音频部分响应”替代 BLE `NOTIFY_EVT` 证据。

### 9.4 `reason` 值和“自行重启”的判定

本轮日志中出现的 reason 含义如下：

- `0x3e`：`Connection Failed to be Established`，表示连接尝试未建立成功。主机随后重新扫描，这是连接恢复流程，不是系统复位。
- `0x16`：`Connection Terminated by Local Host`。它与超时回调调用 `esp_ble_gap_disconnect()` 的时间和顺序完全吻合，表示主机主动终止链路。
- 日志开头的 `hci cmd send: disconnect ... rsn:0x13` 是协议栈打印的断开命令/原因参数，不能当作 ESP32 复位原因。随后实际 `GATTC_DISCONNECT_EVT` 的 `reason 0x16` 才是本次回调报告的断链原因。
- 旧一轮日志中的 `0x08` 是 supervision timeout，与本轮的主动断开不同；需要单独保留硬件/射频或协议栈时序方面的调查。

本次提供的片段只有开头一组：

```text
rst:0x1 (POWERON_RESET)
boot:0x13 (SPI_FAST_FLASH_BOOT)
...
```

在后续 131 秒日志中没有第二个 `rst:`、bootloader banner、`Guru Meditation`、`Backtrace`、`abort()`、WDT 或 brownout 信息。因此不能据此断言主机“自行重启”。如果用户在串口上确实观察到重启，下一次必须从复位前至少数秒一直保留到复位后的完整输出，并记录新的 `rst:` 行、`esp_reset_reason()`、panic/WDT backtrace 或 brownout 信息；当前片段不足以确定复位原因。

启动时另有一条独立警告：检测到的 flash 为 4 MB，而 binary header 声明 2 MB，系统按 binary header 使用 2 MB。它可能造成分区/资源容量风险，但当前没有证据表明它导致本次 BLE 等待或主动断链，不应与本问题混为一谈。

### 9.5 固件版本一致性

本轮主机日志标记的应用编译时间为 `Aug 30 2026 22:18:25`，ELF SHA256 前缀为 `b272af8f3...`。本次重新构建后的 `gatt_client/build/gatt_client.elf` 完整 SHA256 仍为 `b272af8f3cd3868dd180d26f06dbe7a0ef41f91d67aaba5aef7fde4f4e4b6805`，与日志前缀一致；这说明该主机日志很可能来自当前构建产物，但仍应以完整 hash、构建记录和实际烧录记录最终确认。不能仅根据日志中的编译时间与文件时间差异断言主机固件过期。原工程从机 ELF 的旧 hash 为 `9c8bd512964a784edc52b67411bd15079a5655615718364b19503122a1cfffee`，本次按当前源码在临时目录重新构建后的 hash 为 `ef08a4a728450b4c68fbe1d8bb0b7822be76b54a82e01a945a5b866d52426a1c`；实际运行中的从机版本尚未确认。

本次主机命令是增量构建；ESP-IDF 的应用描述对象可能沿用之前编译时写入的 `__DATE__/__TIME__`，所以日志中的 `Aug 30 22:18:25` 不能单独作为本次构建墙上时间。若需要严格建立源码与固件的时间对应关系，后续应在独立目录做一次干净主机构建，并同时记录完整 ELF SHA256、binary SHA256 和构建输出。

后续硬件联调前应分别重新构建主机和从机，记录各自的编译时间、完整 ELF SHA256 或 binary 校验值，并确认烧录的两端版本与源码提交/工作树一致；本交接阶段不执行烧录。

### 9.6 本轮问题的结论和优先级

可以直接由这份日志确定的结论是：

1. 主机没有在日志片段中收到门铃数据面 notification。
2. 主机在连接建立约 15 秒后因等待超时调用了主动断链，`reason 0x16` 是该动作的结果。
3. 之后的 30 秒周期日志是扫描任务重新开始，不是 ESP32 重新启动。

最优先的待验证假设是从机没有真正完成通知发送。旧版从机的 `conn_id == 0` 哨兵缺陷仍能完整解释“CCCD 成功、但无 notification”；即使从机已换成修复版，也必须用 `send_indicate()` 返回值和连接/句柄日志排除同一回调内发送时序、CCCD 自动响应和句柄不匹配问题。主机当前 ELF 哈希与测试日志前缀一致，说明不能只把问题归咎于主机固件未更新；两端版本必须分别核实。

如果用户看到的是屏幕或音频“部分响应”，先确认它来自 BLE `ESP_GATTC_NOTIFY_EVT`，还是主机自身 GPIO15/串口测试入口。只有“从机发送成功 + 主机收到 `0x01` notification + 媒体队列入队”三段日志同时出现，才能判定 BLE 门铃数据链路已经闭环。

## 10. 当前状态重新归类

### 已完成

- 已定位并修复从机把合法 `conn_id = 0` 当作无效连接的逻辑缺陷（源码静态层面）。
- 已将从机通知延迟发送、CCCD 响应顺序、通知发送返回值和连接代次校验加入代码。
- 已移除从机 `ESP_GATTS_CONF_EVT` 中的阻塞式 `vTaskDelay()`，改为异步定时关闭。
- 已为主机扫描、连接、GATT 发现、CCCD 写入、迟到事件和恢复路径加入状态及错误处理。
- 主机工程此前已本地编译成功；从机此前也已在临时构建目录编译、链接成功。详情见第 4 节。
- 通过本轮日志确认主机的 15 秒等待和 `0x16` 主动断链是确定行为，而非无证据的“随机卡死/重启”。

### 未完成或未证实

- 尚未证明当前硬件上的从机确实运行了包含上述修复的最新 binary。
- 主机日志的 ELF SHA 前缀与当前 `gatt_client` 构建产物一致，但从机运行 binary 尚未与当前源码/构建 hash 对齐确认；两端固件版本一致性仍需核实。
- 尚未拿到与主机日志对应的从机串口输出，无法确定通知是在从机发送前丢失，还是发送后在主机侧丢失。
- 尚未决定连接配置完成后主机应长期保持连接等待下一次门铃，还是只等待一次事件后关闭。当前 15 秒 setup timer 把“没有门铃事件”当作连接失败，协议语义仍需明确。
- 主机通知收到后的媒体任务、SD 读、LVGL 和 I2S 只做过代码/编译层检查，没有硬件验证。
- 早期 `0x3e` 连接建立失败、旧一轮 `0x08` supervision timeout 以及 4 MB/2 MB flash header 不匹配仍有残余风险，但它们没有被当前片段证明是同一根因。

## 11. 下一轮修复步骤（交给后续模型）

### 11.1 先锁定协议语义

明确以下两种模式中的一种，并据此修改计时器：

1. **一次性门铃事件模式**：从机按键后广播，主机连接并订阅，必须在限定时间收到一个事件；若超时可断开，但应使用单独的“事件等待超时”名称和日志。
2. **常驻订阅模式**：主机连接并完成 CCCD 后保持链路，等待未来按键；CCCD 写成功后应立即停止 setup timer，不能因为暂时没有事件而每 15 秒断开。

在没有从机通知闭环证据前，不要仅靠延长 15 秒数值掩盖问题。若选择一次性模式，仍应保留有限超时作为故障恢复；若选择常驻模式，建立独立的链路保活/断链恢复策略。

### 11.2 复核主机代码

- 在 `ESP_GATTC_WRITE_DESCR_EVT` 成功后按协议决定是否调用 `disarm_connection_timeout()`；或者改为启动命名清晰的事件等待计时器。
- 将 `connection_timeout_timer_cb()` 的日志和分支拆成 `open timeout`、`GATT setup timeout`、`notification wait timeout`，例如已连接链路使用 `Notification wait timed out; disconnecting active link`，避免把已连接链路称为“cancelling connection attempt”；这主要是可观测性修正，不改变 `0x16` 的实质含义。
- 给连接尝试和超时计时器增加代次/阶段 token；在计时器回调再次核对当前连接、`link_up` 和“通知是否已收到”，处理通知与 15 秒回调同时到达的竞态，避免 `disarm_connection_timeout()` 已执行但回调仍继续主动断链。
- 收到通知前后打印连接 ID、特征句柄、数据长度和首字节；收到合法 `0x01` 后确认 `disarm_connection_timeout()` 确实执行。
- 保存实际写入的 CCCD descriptor handle，并在 `ESP_GATTC_WRITE_DESCR_EVT` 中同时校验 handle、conn_id、远端地址和当前连接代次；当前代码只校验 conn_id，迟到的同连接/旧请求回调仍可能被误接受。
- 给同一远端地址的每次 open 尝试分配 token，避免旧 `CONNECT_EVT`/`OPEN_EVT` 在新尝试期间被当作当前连接；仅比较地址不足以排除这种竞态。
- 检查 `CLOSE_EVT`、`DISCONNECT_EVT`、`CANCEL_OPEN_EVT` 与扫描完成事件的交错顺序，确保每条失败路径最终只触发一次受控扫描。
- 保持 `conn_id == 0` 为合法值，继续检查远端地址、GATT 接口、句柄和连接代次，防止迟到事件污染新连接。

### 11.3 复核从机代码

- 确认按键任务在未连接、未订阅和发送失败时设置 `pending_notify`，并在 `ESP_GATTS_WRITE_EVT` 收到 CCCD `0x0001` 后打印并执行 `Delivering deferred doorbell notification`。
- 确认 `send_doorbell_notify()` 的条件允许 `saved_conn_id == 0`，调用的连接 ID、GATT 接口和 `IDX_CHAR_VAL_A` 句柄来自同一连接代次。
- 记录 `esp_ble_gatts_send_indicate()` 返回值；失败时保留事件并通过非阻塞路径恢复，不在 GATT 回调中执行长时间延时。
- 当前 CCCD 和特征值属性表使用 `ESP_GATT_AUTO_RSP`。因此正常 CCCD 写入时 `param->write.need_rsp` 可能为 false，应用层的 `esp_ble_gatts_send_response()` 分支不会执行；若需要拒绝非法 CCCD 值，必须改用 `ESP_GATT_RSP_BY_APP` 并完整构造响应，否则不要把应用层 `response_status` 日志当成实际 ATT 响应结果。
- 即使保留自动响应，也不要在同一个 `ESP_GATTS_WRITE_EVT` 回调中紧接着调用 `send_doorbell_notify()` 就假定 ATT 响应已经在线路上完成。记录实际返回值；必要时用 `esp_timer`/独立任务延迟数毫秒发送，且不在回调中阻塞。
- 核对从机特征声明确实包含 `ESP_GATT_CHAR_PROP_BIT_NOTIFY`，CCCD 写响应状态为 `ESP_GATT_OK`，并确认 notification（`0x0001`）与 indication（`0x0002`）语义没有混用。
- 检查通知发送后的异步关闭定时器不会关闭新连接；断开时停止旧定时器并清理连接 ID/接口。
- 检查按需广播启动/停止的异步状态、60 秒广告窗口和快速重复按键行为。
- 重点复核 `ADV_STARTING`/`ADV_STOPPING` 与 `ESP_GATTS_CONNECT_EVT` 的交错：当前连接事件会直接把广告状态改为 `ADV_IDLE`，而 `stop_doorbell_advertising()` 只处理 `ADV_ACTIVE`；若启动完成事件迟到，状态可能永久停在 `STARTING` 或出现陈旧完成事件，导致后续按键无法重新广播。
- `pending_notify` 是单个 bool，连续按键会合并为一个待发送事件；确认这是协议允许的行为，或改为计数/队列。跨 button task、GATT callback 和 esp_timer task 的共享状态也应明确锁或事件队列边界，避免数据竞争。

### 11.4 收集下一轮硬件日志（仅供用户后续联调）

本交接阶段不执行下载或串口监视。用户后续联调时应同时保存主机和从机日志，并让两端使用同一轮源码构建产物。最小证据链应包含：

```text
从机: ESP_GATTS_CONNECT_EVT, conn_id = 0
从机: notify enable
从机: Delivering deferred doorbell notification
从机: Doorbell notification sent to host (ret = ESP_OK)
主机: Descriptor write successfully (notify enabled)
主机: Notification received (doorbell trigger)
主机: Doorbell pressed; queued SD audio and knock animation
```

若从机先打印发送失败，先修从机；若从机打印成功而主机没有 `NOTIFY_EVT`，再集中检查句柄、CCCD、连接状态和链路断开竞态。若出现真正复位，完整保留复位前后日志，不要只截取断开附近的几行。

## 12. 下一轮软件验证方法

以下仅限本地源码编译和静态检查，不包含烧录、下载或硬件测试：

```bash
source /home/sk/.espressif/v6.0.2/esp-idf/export.sh

cd /home/sk/espprj/gatt_server_service_table/gatt_client
IDF_COMPONENT_MANAGER=0 idf.py build

cd /home/sk/espprj/gatt_server_service_table/gatt_server_service_table
IDF_COMPONENT_MANAGER=0 idf.py build

cd /home/sk/espprj/gatt_server_service_table
git diff --check
```

验证顺序：

1. 先编译主机；若出现错误，只修复首个真正的 compiler error，再重编译。
2. 再编译从机；若原 `build/` 因历史权限/只读日志失败，在 `/tmp` 中复制工程做干净构建，不删除或清理用户现有目录。
3. 确认两个工程都完成 C 编译、链接、binary 生成和分区大小检查；记录应用 binary 大小及剩余空间。
4. 用 `rg` 检查没有遗留 `saved_conn_id == 0`、`conn_id != 0` 之类错误哨兵判断，也没有在 GATT 回调中保留 `vTaskDelay()`/长阻塞。
5. 检查 `esp_ble_gatts_send_indicate()`、`esp_ble_gap_disconnect()`、定时器 API 的返回值和参数类型与 ESP-IDF v6.0.2 头文件一致。
6. 最后运行 `git diff --check`，确认没有空白错误或无关文件被回滚。

本阶段不执行 `idf.py flash`、`esptool`、`idf.py monitor`，也不把“编译通过”描述成 BLE、按键、音频或复位问题已经在硬件上解决。若连续多次编译失败且首个错误变成环境、权限或依赖问题，应停止尝试并把具体错误交给用户处理。

### 12.1 本轮实际执行结果

- 主机 `gatt_client`：`IDF_COMPONENT_MANAGER=0 idf.py build` 成功，binary `0x109180`，应用分区余量 29%，ELF SHA256 为 `b272af8f3cd3868dd180d26f06dbe7a0ef41f91d67aaba5aef7fde4f4e4b6805`。
- 从机原 `build/`：第一次命令因 `build/log` 只读退出，属于构建目录权限问题，不是源码编译错误。
- 从机 `/tmp/gatt_server_service_table_build_20260901`：干净构建成功，binary `0xba920`，应用分区余量 27%，ELF SHA256 为 `ef08a4a728450b4c68fbe1d8bb0b7822be76b54a82e01a945a5b866d52426a1c`。
- `git diff --check` 和新文档 trailing-whitespace 检查通过。
- 静态 `rg` 检查未发现 `saved_conn_id == 0` 或 `conn_id != 0` 这类错误哨兵比较；源码中的 `vTaskDelay()` 只出现在按键/媒体触发任务，未出现在 GATT 回调。
- 未执行 `idf.py flash`、`esptool`、`idf.py monitor`，没有进行硬件下载或 BLE 实机复测。
