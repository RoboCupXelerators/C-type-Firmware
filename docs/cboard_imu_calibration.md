# C 型板 IMU 校准与温控

启动后先进行 heater warm-up，再收集 1000 个静止有效样本。每个样本检查加速度模长、gyro 运动阈值和有限值；使用 Welford 均值/方差计算三轴 gyro bias。运动会重置收集并标记 `REJECTED_MOVING`，15 秒校准超时进入 degraded，初始化或 SPI 失败不会发布看似有效的零姿态。

校准结果只保存在 RAM，不写 BMI088 永久寄存器、不写 Flash、不修改任何电机零点。`ZeroYaw` 和 `SetYawReference` 同样只修改 RAM reference。

姿态模式为 `GYRO_RELATIVE`。无磁力计时 yaw 不可获得绝对长期约束；静止 bias 抑制短期漂移，accelerometer 只提供 roll/pitch 比例校正。2026-07-13 发现原 Mahony integral 长期 wind-up 后，正式源码移除了 6DoF 积分反馈，并增加“旧积分状态不得注入 yaw”的 host 回归测试。

温控使用 TIM10/PF6。目标为启动时 MCU 温度 +10°C（最高 45°C），50°C 触发锁存保护；温度未稳定时设置相应 flag，温控不得阻塞 USB/CAN。
