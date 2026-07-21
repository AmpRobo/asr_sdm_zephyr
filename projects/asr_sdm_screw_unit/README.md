# ASR SDM Screw Unit 固件

本项目运行在 Seeed XIAO RP2350 上，通过 BCAN-S01 串口转 CAN 模块接入 CAN 总线，并通过 UART1 控制 Dynamixel 舵机。

```text
Linux 上位机 / USB-CAN
          ↕
       CAN 总线
          ↕
      BCAN-S01
          ↕ UART0
    XIAO RP2350
    ├─ UART1 → Dynamixel 舵机
    ├─ SPI0  → ICM45686 IMU
    └─ I2C0  → HP206F 气压计
```

- 通信模块架构和 PROTOL 格式：[modules/comm/README.md](../../modules/comm/README.md)
- BCAN-S01 厂商手册：[BCAN-S01 用户使用手册 1.52](../../datasheet/BCAN-S01用户使用手册1.52.pdf)
- 环境安装说明：[仓库根 README](../../README.md)

## 1. 当前配置基线

| 项目 | 当前值 |
|---|---|
| MCU 串口 | UART0 |
| XIAO UART0 TX | D6 / GP0 |
| XIAO UART0 RX | D7 / GP1 |
| UART 参数 | 115200、8N1、无流控 |
| BCAN-S01 输入电源 | 4.75～5.25 V，项目使用 5 V |
| BCAN-S01 TTL 电平 | 3.3 V |
| 转换模式 | `PROTOL` |
| 转换方向 | `BOTHWAY` |
| CAN 波特率 | 100 kbit/s |
| CAN 帧类型 | 标准数据帧 `NDTF` |
| CAN 接收过滤 | `OFF` |
| AT 命令回显 | `OFF` |
| 固件请求 CAN ID | `0x123` |
| 固件响应 CAN ID | `0x124` |
| 业务帧 DLC | 固定为 8 |
| 气压计 | HP206F，I2C0（D13/GP17=SCL，D14/GP16=SDA） |
| 气压计 OSR | 1024，周期 500 ms，日志间隔 5000 ms |

> BCAN-S01 出厂默认为 `TRANS` 透明转换模式。生产固件不会自动发送 AT 指令，使用新模块、更换模块或恢复出厂设置后，必须先按本文完成一次配置。

## 2. BCAN-S01 接线

### 2.1 固件运行接线

| XIAO RP2350 | BCAN-S01 | 说明 |
|---|---|---|
| D6 / GP0 / UART0 TX | pin 4 `RX` | MCU 向模块发送 |
| D7 / GP1 / UART0 RX | pin 3 `TX` | MCU 接收模块数据 |
| 5 V | pin 2 `VIN` | 模块输入电源 |
| GND | pin 1 `GND` | TTL/输入电源侧公共地 |
| — | pin 13 `CANH` | 接 CAN 总线 CANH |
| — | pin 12 `CANL` | 接 CAN 总线 CANL |



### 2.2 使用 USB-TTL 配置模块

进入 AT 模式时建议暂时断开 XIAO 的 D6/D7，防止 XIAO TX 与 USB-TTL TX 同时驱动 BCAN-S01 RX。

| USB-TTL | BCAN-S01 |
|---|---|
| TX（3.3 V TTL） | pin 4 `RX` |
| RX（3.3 V TTL） | pin 3 `TX` |
| GND | pin 1 `GND` |
| 外部 5 V | pin 2 `VIN` |

串口工具设置为：

```text
115200 baud
8 data bits
1 stop bit
No parity
No flow control
```

## 3. 进入 AT 模式并配置 BCAN-S01

### 3.1 进入 AT 模式

进入阶段的两个字符串都不能带换行：

```text
发送：+++     不带 CR、LF 或 CRLF
发送：AT      在 3 秒内发送，仍不带换行
```

推荐在 `+++` 后等待约 500 ms，再发送 `AT`。成功后应收到包含以下内容的响应：

```text
AT MODE
```

从进入 AT 模式成功开始，后续 AT 命令均需要以 `CRLF`（字节 `0D 0A`）结尾。

> 很多串口终端按 Enter 时会自动追加 CR/LF。发送最初的 `+++` 和 `AT` 时，必须选择“无换行”或 raw send（原始发送）模式。

### 3.2 查询当前参数

逐条发送以下命令，每条末尾带 CRLF：

```text
AT+VER?
AT+UART?
AT+MODE?
AT+DIRECTION?
AT+CAN?
AT+CANLT?
```

项目期望 UART 参数为：

```text
115200,8,1,NONE,NFC
```

如果 UART 已经是该值，不需要重复修改串口参数。

### 3.3 写入生产参数

逐条发送以下命令，每条末尾带 CRLF，并确认模块返回成功：

```text
AT+MODE=PROTOL
AT+DIRECTION=BOTHWAY
AT+CAN=100,0,NDTF
AT+E=OFF
AT+CANLT=OFF
```

各参数含义：

| 命令 | 含义 |
|---|---|
| `AT+MODE=PROTOL` | 使用固定 13 字节协议转换模式。`PROTOL` 是设备定义的实际拼写 |
| `AT+DIRECTION=BOTHWAY` | UART→CAN 和 CAN→UART 双向转换 |
| `AT+CAN=100,0,NDTF` | CAN 为 100 kbit/s，基准 ID 为 0，标准数据帧 |
| `AT+E=OFF` | 关闭 AT 命令回显 |
| `AT+CANLT=OFF` | 关闭模块 CAN 接收过滤，由 MCU 固件按 ID 过滤 |

在 `PROTOL` 模式中，每个 UART 记录都携带自己的 CAN ID 和帧信息。项目实际使用的 `0x123` 请求 ID 和 `0x124` 响应 ID由 MCU 固件决定。

### 3.4 查询确认并退出

再次发送：

```text
AT+UART?
AT+MODE?
AT+DIRECTION?
AT+CAN?
AT+E?
AT+CANLT?
```

确认结果与以下基线一致：

```text
UART      = 115200,8,1,NONE,NFC
MODE      = PROTOL
DIRECTION = BOTHWAY
CAN       = 100,0,NDTF
E         = OFF
CANLT     = OFF
```

退出 AT 模式：

```text
AT+EXAT
```

然后对 BCAN-S01 **完整断电并重新上电**，使配置生效。

### 3.5 CFG、RST 和恢复出厂

厂商手册说明：

- pin 8 `CFG` 拉低可进入配置模式；
- pin 10 `RST` 为复位引脚；
- pin 7 `RESTORE` 拉低 5 秒可恢复默认参数。

当前项目尚未验证 `CFG` 和 `RST` 的准确上电顺序、保持时间及释放时序，因此标准配置流程使用串口 `+++`/`AT`，不使用未经验证的 CFG/RST 脉冲步骤。

恢复出厂也可以在 AT 模式中执行：

```text
AT+RESTORE
```

恢复出厂后必须重新写入本文的生产参数。

## 4. 编译和烧录

参考仓库根目录下的readme.md。

```sh
west flash --build-dir build/asr_sdm_screw_unit
```

## 5. Linux SocketCAN 上位机调试

以下步骤假设 USB-CAN 设备已经由 Linux SocketCAN 驱动识别。接口名不一定是 `can0`，先执行：

```sh
ip -br link
```

### 5.1 配置 CAN 接口

以 `can0` 为例：

```sh
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 100000 restart-ms 100
sudo ip link set can0 up

ip -details -statistics link show can0
```

检查：

- 接口状态为 `UP`；
- bitrate 为 `100000`；
- 接口没有进入 `BUS-OFF`；
- RX/TX error 计数没有持续增长。

### 5.2 监听总线

监听全部 CAN 帧：

```sh
candump -tz can0
```

只监听固件响应 ID `0x124`：

```sh
candump -tz can0,124:7FF
```

建议在一个终端持续运行 `candump`，在另一个终端执行 `cansend`。

### 5.3 当前业务 payload

CAN DATA 固定为 8 字节：

```text
Byte 0..1  保留，通常为 00 00
Byte 2     CMD：02=READ，03=WRITE
Byte 3     PARAM
Byte 4..7  参数数据，具体布局由 PARAM 决定
```

所有示例中，`#` 后必须有 16 个十六进制字符，即正好 8 字节。

### 5.4 读取 IMU

```sh
cansend can0 123#0000020A00000000
```

成功时应收到 ID `0x124`、DLC 8 的响应：

```text
Byte 0..1  accel X 的整数部分，int16 little-endian
Byte 2..3  accel Y 的整数部分，int16 little-endian
Byte 4..5  accel Z 的整数部分，int16 little-endian
Byte 6..7  温度的整数部分，int16 little-endian
```

当前响应只传输 Zephyr `sensor_value.val1`，不包含 `val2` 小数部分。

### 5.5 Dynamixel 扭矩控制

开启 JOINT1 扭矩：

```sh
cansend can0 123#0000030801000000
```

关闭 JOINT1 扭矩：

```sh
cansend can0 123#0000030800000000
```

### 5.6 设置目标位置

将 JOINT1 目标位置设为 2048：

```sh
cansend can0 123#0000030600080000
```

换算：

```text
2048 = 0x00000800
32 位 little-endian = 00 08 00 00
```

### 5.7 读取当前位置

```sh
cansend can0 123#0000020600000000
```

成功时应收到 ID `0x124`、DLC 8 的响应：

```text
Byte 0..3  当前原始位置，int32 little-endian
Byte 4..7  00 00 00 00
```

例如当前位置为 2048 时，响应 payload 为：

```text
00 08 00 00 00 00 00 00
```

### 5.8 WRITE 命令没有 ACK

当前协议的 WRITE 命令不发送确认帧，包括：

- 扭矩开启/关闭；
- 目标位置写入；
- LED 写入。

因此，发送 WRITE 后没有收到 `0x124` 是正常行为，不能仅据此判断命令失败。应结合以下方式确认：

- USB CDC 日志；
- 舵机实际动作；
- 后续读取当前位置；
- CAN 接口错误统计。

READ 命令只有在模块已就绪且回调成功时才会产生响应。IMU 或 Dynamixel 未就绪时，也可能没有 `0x124` 响应。


## 6. HP206F 气压计

### 6.1 接线

| XIAO RP2350 | HP206F | 说明 |
|---|---|---|
| D14 / GP16 | pin 5 `SDA` | I2C 数据线 |
| D13 / GP17 | pin 6 `SCL` | I2C 时钟线 |
| 3.3 V | pin 2 `VDD` | 供电 |
| GND | pin 1 `GND` | 地 |
| — | pin 3 `INT1` | 未连接 |

### 6.2 I2C 上拉电阻

HP206F 数据手册要求 SDA 和 SCL 各有一个 **10 kΩ 上拉电阻到 3.3 V**。当前控制板原理图**没有外部上拉电阻**，因此固件已开启 RP2350 内部上拉（约 50 kΩ）作为替代。

如果内部上拉导致通信不稳定（如数据异常、偶发读取失败），建议在 D13 和 D14 上各焊接一个 **10 kΩ 电阻到 3.3 V**。

### 6.3 参数

| 项目 | 值 |
|---|---|
| I2C 地址 | `0x76` |
| 总线 | I2C0（不是 `xiao_i2c`） |
| I2C 频率 | 由 HP206C 驱动配置为 100 kHz |
| 过采样率（OSR） | 1024 |
| 采样周期 | 500 ms |
| 日志间隔 | 5000 ms |

### 6.4 日志输出

正常工作时每 5 秒输出一条数据：

```text
[asr_barometer] Barometer ready: HP206F compatible sensor, OSR=1024, period=500 ms
[asr_barometer] barometer: pressure=90.999 kPa, temperature=27.25 C
```

- 压力单位：kPa，Zephyr sensor API 标准单位
- 温度单位：°C
- 读取失败时按 5 秒窗口限流日志，避免刷屏
