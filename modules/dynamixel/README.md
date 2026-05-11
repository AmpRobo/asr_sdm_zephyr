# Dynamixel 硬件连接

## 舵机总线（RP2350 -> RS-485 收发器 -> 舵机）

- `D12 / GP20 / UART1 TX` -> 收发器 `DI`
- `D11 / GP21 / UART1 RX` -> 收发器 `RO`
- `D2 / GP28` -> 收发器 `DE` 和 `/RE`（两脚并在一起）
- 收发器 `VCC` -> `3.3V` 兼容供电
- 收发器 `A / B` -> 舵机 `A / B`
- `GND` -> 收发器 `GND`
- 舵机 `V+` 只接外部舵机电源，不接 XIAO 或收发器 `VCC`

## 舵机默认参数

- `ID = 1`
- `Baud Rate = 57600`
- `Operating Mode = 3`
- `Status Return Level = 2`
