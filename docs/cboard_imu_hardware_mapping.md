# C 型板 IMU 硬件映射

| 功能 | 外设/引脚 | 当前用途 |
|---|---|---|
| BMI088 SPI | SPI1: SCK PB3, MISO PB4, MOSI PA7 | mode 3 传输 |
| Accelerometer CS | PA4 | 低有效 |
| Gyroscope CS | PB0 | 低有效 |
| Accelerometer DRDY | PC4 | GPIO 输入，当前轮询状态 |
| Gyroscope DRDY | PC5 | GPIO 输入，当前轮询状态 |
| IMU heater | TIM10_CH1 / PF6 | PWM 温控 |
| 温控基准 | ADC1 internal temperature | MCU 温度 +10°C，最高 45°C |
| IST8310 | 板级有 reset/DRDY 定义 | 当前无 I2C3 驱动，不启用 |

BMI088 accelerometer ID 为 `0x1E`，gyroscope ID 为 `0x0F`。CAN 侧仍只允许 CAN1；IMU 适配不初始化或选择 CAN2。
