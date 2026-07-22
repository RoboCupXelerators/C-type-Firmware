# 80/100 Degree Chassis Direction Fix

Date: 2026-07-14

## Confirmed geometry

- Wheel order: `FL, FR, RL, RR`
- Body frame: `+x` forward, `+y` left, `+wz` counterclockwise
- Front drive-vector included angle: 80 degrees
- Rear drive-vector included angle: 100 degrees
- Drive-vector angles: `130, 50, 220, -40` degrees

## Root cause

The drive-vector angles already encode that positive wheel motion has a negative
forward component on the left and a positive forward component on the right.
The previous `motor_sign=[-1,+1,-1,+1]` applied that direction a second time.
For `vx=+0.05 m/s`, it generated four positive motor commands and the physical
model predicted `vx=0`, `wz=0.5866 rad/s`, matching the observed in-place turn.

`motor_sign` is electrical polarity relative to each wheel's drive vector, so
the correct value is `[+1,+1,+1,+1]`. A forward target then produces motor
signs `[-,+,-,+]`.

## Verification

- Regression test failed on the old configuration at the FL sign assertion.
- Firmware host tests: 6 passed.
- Pi tests: 103 passed.
- Firmware BIN SHA256: `7a1b9defb9912131ded2d8081d844615566f941a449f62d9894327faccd3b3d7`.
- SWD programming and verification succeeded.
- Two zero-output Arm/Stop/Disable cycles passed.
- Physical test: `vx=+0.03 m/s`, 1 second; user confirmed forward motion.
- Final state: four motors Disabled, online mask `0xF`, fault/stale/ESR all zero.
