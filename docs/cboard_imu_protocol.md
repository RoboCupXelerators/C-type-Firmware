# Pi-C IMU 协议

IMU 复用现有 framed USB CDC 协议：header `55 AA`、protocol version、message type、sequence、payload length、CRC16 和 tail。Pi 侧保持单 RX owner、sequence 匹配和 pending frame 管理，不使用 `reset_input_buffer()`。

消息：

- `0x26 MSG_GET_IMU_STATUS`：空 payload，只请求最新原子快照。
- `0x27 MSG_IMU_STATUS`：固定 156-byte little-endian payload。
- `0x28 MSG_IMU_ZERO_YAW`：显式 RAM reference 操作。
- `0x29 MSG_IMU_SET_REFERENCE`：little-endian float32 degree。

状态包含 sequence、C 板时间戳、sample/missed/SPI/error 计数、flags/state/source/calibration、chip IDs、age、温度、accel/gyro/mag、quaternion、roll/pitch/wrapped yaw/continuous yaw/gyro Z、bias/variance和时序诊断。C 与 Pi 均严格检查 payload 长度；Pi 拒绝 NaN/Inf，stale 或 fatal flags 不会静默变成 0°。

USB 请求只编码最新快照，不触发同步 BMI088 采样，也不为每个高频 IMU 样本主动发送 CDC 帧。
