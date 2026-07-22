# Software 兼容性

Pi 正式后端集中在 `~/Software/src/utils/DM2325Bits.py`，提供 `GetYaw`、`GetYawRad`、`GetYawContinuous`、`GetGyroscopeData`、`GetAccelerometerData`、`GetMagnetometerData`、`GetIMUState`、`IsIMUHealthy`、`GetIMUAgeMs`、`ZeroYaw` 和 `SetYawReference`。

`LunaPre.runtime` 的 `DM/DM2325` 路径实例化同一个 `DM2325Bits`，`Car` 和 lidar 共用该来源；lidar 只使用最新缓存，不建立第二个 CDC RX owner。FieldMove/FieldTurn 在 IMU 不健康或轴映射未确认时拒绝闭环并输出零目标，RobotMoveVector 的机体坐标架构不依赖 yaw。

Control Panel 使用 `GetHeadingCandidate()` 显示独立候选值，不覆盖正式 `robot_yaw_deg`，并保持 pose/motion invalid。候选同时保留 roll/pitch/yaw，方便最终竖装后集中选择，但当前 selected axis 仍只是未验证的 yaw。
