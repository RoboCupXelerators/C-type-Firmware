# C 型板 IMU 源码审计

审计对象为 `firmware/robomaster_c_xelerator_t2` 当前正式源码，而非文件名推断。

- 实际惯性器件是 Bosch BMI088。固件检查 accelerometer chip ID `0x1E` 和 gyroscope chip ID `0x0F`；2026-07-13 实机采集两者均正确。
- 板级定义包含 IST8310 reset/DRDY，但当前工程没有 I2C3 初始化、IST8310 驱动或磁校准，磁力计不参与姿态；发布的 `mag` 为零，source 为 `GYRO_RELATIVE`。
- BMI088 驱动来源为 RoboMaster Development-Board-C 示例的寄存器、middleware 和初始化配置，当前封装为 `bmi088_driver.c/.h`。初始化逐项检查 chip ID、soft reset、量程/ODR 配置及 SPI 结果。
- 姿态算法为工程内可审计的 6DoF quaternion/Mahony-like 更新，不链接闭源 `AHRS.lib`。静止 gyro bias 由 1000 个合格样本计算。
- SPI1、TIM10 heater、ADC1 MCU 温度、`cboard_imu` 和协议文件均列入 CMake。`main.c` 初始化 SPI1/TIM10 后调用 `cboard_imu_init()`；`main_loop.c` 周期调用 `cboard_imu_update()`，USB 命令只复制最新快照。
- 快照在短临界区内发布，并以 sequence/atomic lock 防止 Pi 读取半新半旧状态。
- 采样使用 1 ms poll、BMI088 DRDY 判定；最近实测有效频率 `808.49 Hz`，Pi 完整快照 `43.70 Hz`。
- DRDY 引脚已配置为输入，但当前采样结构是主循环 poll，不依赖 FreeRTOS 旧任务或 EXTI 回调。
- 当前正式树没有 CubeMX `.ioc`；GPIO/SPI/TIM 配置以生成后的 `Core/Src` 和 `Core/Inc/main.h` 为权威证据。
- 温控链路存在：TIM10 CH1/PF6 heater PWM，目标由 MCU 温度 +10°C 计算且上限 45°C，50°C 过温锁存。
- 当前轴变换为 identity 且 `hardware_verified=false`。这不是最终机器人轴结论。

2026-07-13 长时间静止复验发现旧 6DoF Mahony 积分反馈会在无磁力计时向不可观测 yaw 注入虚假速率。当前源码已移除该积分反馈，保留静止 bias 和比例倾角校正；刷写后的无扰动 120 秒复验以 `-1.604°/min` 通过。
