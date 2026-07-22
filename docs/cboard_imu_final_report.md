# C 型板 IMU 阶段报告

日期：2026-07-13。软件、固件修正、clean build、刷写、verify 和无扰动 120 秒 static acceptance 已完成；最终机器人轴向仍按总装冻结要求延后。

1. IMU：Bosch BMI088，accel ID `0x1E`、gyro ID `0x0F`。
2. 磁力计：板上有 IST8310 定义，但当前无 I2C3 驱动/校准，不参与融合。
3. 驱动：RoboMaster Development-Board-C 示例派生的 BMI088 register/driver/middleware。
4. 姿态：工程内 6DoF quaternion/Mahony-like；当前为 bias + proportional tilt correction，无不可观测 yaw integral。
5. SPI/GPIO：SPI1 PB3/PB4/PA7；accel CS PA4、gyro CS PB0；DRDY PC4/PC5；heater TIM10_CH1/PF6。
6. IMU 有效采样：最近 `808.46 Hz`。
7. 融合：与有效 accel-ready 样本同步，约 `808.46 Hz`。
8. Pi 快照：最近 `43.71 Hz`。
9. 轴映射：identity candidate，硬件未验证。
10. yaw 正方向：软件约定俯视逆时针为正，实机方向未验证。
11. yaw 零点：启动相对零；ZeroYaw/SetYawReference 仅改 RAM reference。
12. bias：1000 个静止有效样本 Welford 均值/方差；本次 Z bias `-0.00350685 rad/s`。
13. 温控：存在，TIM10/PF6；目标 MCU 温度 +10°C、最高45°C、50°C锁存保护。
14. C 模块：`bmi088_driver`、`cboard_imu_math`、`cboard_imu` 和 IMU protocol snapshot。
15. Pi 模块：集中在 `~/Software/src/utils/DM2325Bits.py`。
16. GetYaw：保持 float degree `[0,360)`，stale/fault 不返回伪 0。
17. GetIMUState：READY/calibrated/age/flags/counters/raw vectors 均已实机读取。
18. FieldMoveVector：保持接口，IMU/轴未验证时安全拒绝。
19. FieldTurn：同上。
20. 测试：Firmware `6/6`；Software 全量 `343/343`。
21. 固件 SHA256：ELF `fb00c3d8...e3e2`；BIN `bb8f2ba4...ac85`。
22. SWD：Programming Finished，Verified OK，reset 成功。
23. 被动底盘状态：runtime Disabled、online `0x00`、enabled `0`、fault `0`、CAN ESR `0`；未发送 CAN refresh。
24. 修正后正式 120 秒样本：5246 个 Pi 快照，C 板样本推进 97014。
25. IMU rate：`808.453 Hz`。
26. SPI error：`0→0`。
27. missed sample：`0→0`。
28. yaw 漂移：`-3.2079°/120s`，`-1.6040°/min`；最大观测步进 `0.00556°`。
29. gyro noise std X/Y/Z：`0.003731/0.004509/0.004321 rad/s`。
30. accel norm：均值 `9.83752 m/s²`，std `0.03813 m/s²`。
31. 温度：31.375→31.25°C，最高31.375°C。
32. health flags：仅 `0x00004000 AXIS_MAPPING_UNVERIFIED`。
33. static acceptance：`PASS_STATIC_AXIS_MAPPING_PENDING`，全部检查为 true；上一轮移动污染数据另行归档，未裁剪或伪造。
34. 电机非零运动：否；未 Enable，未发送 MIT 或非零 twist。
35. 下一步：等待 3D 打印底盘与 C 板最终安装方向确定后，再进行人工轴向旋转；当前不执行。
36. 文档：本目录 `cboard_imu_*` audit/mapping/coordinate/protocol/calibration/compatibility/build/flash/static/final 文件。
