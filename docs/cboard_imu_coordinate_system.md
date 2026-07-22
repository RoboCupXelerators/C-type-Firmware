# C 型板 IMU 坐标约定

最终机器人坐标约定保持：+X 向前、+Y 向左、+Z 向上；正 yaw 为俯视逆时针；内部角度为 rad，兼容 `GetYaw()` 为 degree `[0, 360)`。

当前安装方向尚未确认。固件只保留一个集中式 `imu_mount_transform_t`：source axis、axis sign、mount yaw offset 和 hardware verified。当前配置为 identity、offset 0、`hardware_verified=false`，对外始终携带：

- `AXIS_MAPPING_UNVERIFIED`
- `ROBOT_DIRECTION_UNDEFINED`
- `DEFERRED_UNTIL_FINAL_ASSEMBLY`

因此当前 roll/pitch/yaw 只能作为 C 板姿态候选显示，不能作为最终机器人方向或运动闭环依据。最终装配前不得修改映射或宣布方向通过。
