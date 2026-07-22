# DM-S2325 chassis maximum-speed test

Date: 2026-07-14

## Limits

- Motor: DM-S2325-1EC, 24 V
- Manufacturer rated speed: 380 rpm
- Manufacturer no-load maximum: 560 rpm = 58.64 rad/s
- Wheel radius: 0.030 m
- Chassis geometry: front included angle 80 degrees, rear included angle 100 degrees
- Command limit: 2.29 m/s
- Measurement source: four motor encoders and chassis forward kinematics; no IMU

## Test

- Wheels off the ground
- Smooth ramp: 5.5 s
- Full-speed command hold: 20.0 s
- Samples: 5,067
- Mean full-speed estimate: 2.2583 m/s
- Peak speed estimate: 2.2625 m/s
- Fastest observed motor: 58.657 rad/s
- Maximum feedback age: 10 ms
- Maximum MOS temperature: 39 C
- Online mask: 0x0F throughout
- Enabled mask: 0x0F during motion, 0x00 after test
- Fault mask: 0x00 throughout
- Final runtime state: Disabled

The full-speed encoder estimate contained about -0.38 rad/s of yaw. This
indicates unloaded wheel-speed mismatch at the mechanical limit. The measured
peak is valid as a wheel-encoder kinematic estimate, but high-speed ground
driving should be tuned for straight-line tracking before use.
