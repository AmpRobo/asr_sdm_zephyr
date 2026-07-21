# ASR SDM Communication Module (`modules/comm`)

## 架构

```
                 BCAN-S01 PROTOL
                            |
                            v
        bcan_protol.c + bcan_protol_codec.c
                            |
                            v
+------------------------------------------------------+
|              protocol_core.c                          |
|  (transport-independent 8-byte protocol processor)    |
+------------------------------------------------------+
    |                              |
    v                              v
on_led_write / on_imu_read / on_motor_set / on_dynamixel_*
```

## 文件职责

| 文件                           | 职责                                                                                         |
| ----------------------------- | -------------------------------------------------------------------------------------------- |
| `protocol_core.h/.c`          | 传输无关的业务核心。解析 8 字节 ASR 协议消息，维护 unit_status，调用硬件回调，返回结果对象。             |
| `transport_uart_aa55.h/.c`    |  UART-AA55 传输层。`[0xAA][0x55][LEN][DATA...][CHK]` 帧格式，中断驱动 RX，poll-based TX。       |
| `bcan_protol.h/.c`            | BCAN-S01 PROTOL 传输层。13 字节固定帧，poll-based UART I/O，帧同步 + 解码验证。                     |
| `bcan_protol_codec.c`         | PROTOL 编解码纯函数（无 UART 依赖），通过 `bcan_protol_internal.h` 供模块内和测试使用。               |
| `bcan_protol_internal.h`      | 内部 codec API 头文件（不对外暴露）。                                                             |
| `can_protocol_transport.h/.c` | CAN 业务适配层。CAN ID 过滤、DLC 校验、调用 `protocol_core_process`、可选回复。                     |
| `can_protocol_thread.h/.c`    | CAN 协议后台线程。                                                                              |
| `comm_thread.h/.c`            | UART-AA55 后台线程（legacy）。                                                                |

## 线格式

### UART-AA55

```
[0xAA] [0x55] [LEN] [DATA_0 .. DATA_N-1] [CHK]
CHK = LEN ^ DATA_0 ^ ... ^ DATA_N-1
LEN = 固定 8 (ASR_COMM_MSG_SIZE)
```

### BCAN PROTOL (13 字节固定帧)

```
Byte 0:      frame_info (bit 7=EXT, bit 6=RTR, bit 3-0=DLC)
Byte 1..4:   CAN ID (大端)
Byte 5..12:  CAN DATA (8 字节)
```

当前同步方式：等待 `frame_info == 0x08`（标准数据帧 DLC=8），再读取后续 12 字节。PROTOL 无专用帧头，此方式存在理论误同步风险。

### ASR 业务 payload (8 字节固定)

```
Byte 0..1:   保留
Byte 2:      CMD (0x02=READ, 0x03=WRITE)
Byte 3:      PARAM
Byte 4..7:   参数值 (小端，具体布局由 PARAM 决定)
```

## Screw Unit 生产集成基线

当前 `projects/asr_sdm_screw_unit` 使用以下配置：

| 项目          | 值                         |
| ------------- | -------------------------- |
| UART          | UART0，115200、8N1、无流控 |
| XIAO 引脚     | D6/GP0=TX，D7/GP1=RX       |
| BCAN-S01 模式 | `PROTOL`                   |
| 转换方向      | `BOTHWAY`                  |
| CAN 波特率    | 100 kbit/s                 |
| CAN 帧        | 标准数据帧，DLC=8          |
| 模块过滤      | `CANLT=OFF`                |
| 请求 CAN ID   | `0x123`                    |
| 响应 CAN ID   | `0x124`                    |

## 请求与响应语义

- CAN ID 不等于项目请求 ID 的帧会被静默忽略。
- 业务请求 DLC 必须严格等于 8。
- READ 只有在业务处理成功并设置 `has_response` 后才发送响应。
- WRITE 当前不发送 ACK，即使命令被接收也不会返回 `0x124`。
- IMU READ 响应由四个 `int16` little-endian 值组成，依次为 accel X/Y/Z 和温度的整数部分。
- JOINT1 位置 READ 响应的 Byte 0..3 是 `int32` little-endian 当前位置，Byte 4..7 为 0。
- 响应 payload 不统一回显请求 CMD/PARAM，必须按具体 READ 参数解释。

## Kconfig 传输选择

UART0 一次只能由一个 transport 拥有。通过 Kconfig `choice` 确保互斥：

```kconfig
choice ASR_PROTOCOL_TRANSPORT
    config ASR_TRANSPORT_UART_AA55     # UART-AA55
    config ASR_TRANSPORT_BCAN_PROTOL   # BCAN-S01 PROTOL
endchoice
```

| 配置项                               | 说明                                       |
| ------------------------------------ | ------------------------------------------ |
| `ASR_CAN_PROTOCOL_REQUEST_CAN_ID`    | 本节点响应的 CAN ID                        |
| `ASR_CAN_PROTOCOL_RESPONSE_CAN_ID`   | 响应使用的 CAN ID                          |
| `ASR_BCAN_PROTOL_RX_QUEUE_DEPTH`     | CAN 帧队列深度（预留，当前轮询模式未使用） |
| `ASR_BCAN_INTERBYTE_TIMEOUT_MS`      | PROTOL 字节间超时 (ms)                     |
| `ASR_CAN_PROTOCOL_THREAD_STACK_SIZE` | CAN 协议线程栈大小                         |
| `ASR_CAN_PROTOCOL_THREAD_PRIORITY`   | CAN 协议线程优先级                         |

## 线程模型

当前使用单个后台线程轮询 UART0：

```
CAN protocol thread
  → bcan_protol_receive()  (poll UART, decode PROTOL)
  → can_protocol_handle_frame()  (CAN ID filter, DLC check)
  → protocol_core_process()  (business dispatch)
  → if has_response: bcan_protol_send()  (encode + send)
```

UART-AA55 使用 `comm_thread`，模式相同但使用中断驱动 RX。
