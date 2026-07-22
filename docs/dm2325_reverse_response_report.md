# DM2325 chassis reversal response report

Date: 2026-07-15

## 1. Original program localization

The production chain is:

`Software/src/control_panel.py` -> `LunaPre.runtime` ->
`Software/src/utils/DM2325Bits.py` -> USB CDC `CHASSIS_TWIST` ->
`C-type-Firmware/.../main_loop.c` -> `chassis_runtime.c` ->
`chassis_kinematics.c` -> `dm2325_can.c` -> CAN1 -> DM2325.

DM2325 feedback returns through `dm2325_can.c`, the C-board status protocol,
`DM2325Bits.py`, and the existing control panel. No second motor controller,
CAN driver, kinematics implementation, or web service was introduced.

## 2. Root-cause assessment

Confirmed primary cause: the old runtime applied one conservative chassis-axis
S-curve to every transition: 0.5 m/s2 acceleration and 2 m/s3 jerk. A +V to -V
request therefore traversed the full 2V delta using the normal ramp. Its prior
same-direction acceleration could survive briefly after the sign change, making
the chassis appear to wait before braking.

Confirmed contributing cause: limiting happened before non-orthogonal inverse
kinematics, so wheel feedback was unavailable to the reversal decision.

Excluded: an explicit wait-for-zero state, a reverse lock, an integral term,
position control, CAN2, and a second high-level ramp. The active motor command is
MIT with `p_des=0`, `kp=0`, `kd=1`, `t_ff=0`, and velocity target control.

Pending hardware characterization: real brake-to-zero time, reverse response
time, peak torque under load, bus voltage, motor current, tire slip, and thermal
rise. These values were not fabricated from the software model.

## 3. Original behavior flow

Pi twist -> chassis-axis S-curve -> 80/100-degree inverse kinematics -> four
velocity targets -> MIT CAN command every 10 ms. Positive acceleration could
remain after a reverse request because normal jerk limiting had to unwind it.

## 4. New behavior flow

Pi twist -> existing 80/100-degree inverse kinematics -> per-wheel requested
velocity -> feedback-aware limiter -> four MIT CAN commands every 10 ms.

When requested and measured wheel speeds have opposite signs, the next control
tick selects reverse braking, clears stale opposing acceleration, applies the
reverse jerk limit, crosses zero continuously, and accelerates in the requested
direction without a zero hold. Speed saturation, watchdog, stale feedback,
fault handling, zero-before-disable, and Disable confirmation remain active.

## 5. Modified files

C board: `Core/Inc/chassis_runtime.h`, `Core/Src/chassis_runtime.c`,
`Core/Inc/pi_c_protocol.h`, and `Core/Src/main_loop.c`.

Pi/web: `Software/src/utils/DM2325Bits.py`, `Software/src/control_panel.py`,
`Software/web/control_panel/index.html`, and
`Software/web/control_panel/static/app.js`.

Tests/diagnostics: `tests/c/test_chassis_runtime.c`,
`tests/test_chassis_protocol.py`, `tests/test_chassis_reverse_motion.py`,
`tools/chassis/motion.py`, and `tools/chassis/reverse_step_benchmark.py`.

## 6. Control mode

The control mode remains DM2325 MIT velocity control. The firmware sends four
CAN1 commands at a 10 ms period. Position gain and position target remain zero;
the verified damping gain remains 1.0. CAN2, persistent motor parameters,
control mode, baud rate, zero point, ESC ID, and Master ID were not changed.

## 7. Response profiles

All values below are motor-side units.

| Profile | Accel rad/s2 | Decel rad/s2 | Reverse brake rad/s2 | Normal jerk rad/s3 | Reverse jerk rad/s3 | Status |
|---|---:|---:|---:|---:|---:|---|
| safe | 8.0 | 12.0 | 18.0 | 40.0 | 120.0 | selectable |
| normal | 16.7 | 25.0 | 40.0 | 66.7 | 240.0 | default |
| competition_candidate | 40.0 | 60.0 | 100.0 | 200.0 | 600.0 | unverified candidate |

The candidate profile is available for staged testing but is not declared safe
for competition until current, voltage, traction, temperature, and mechanical
load are measured.

## 8. Feedback and diagnostics

| Field | Source | Unit | Update | Support |
|---|---|---|---|---|
| requested/limited wheel speed | C-board limiter | rad/s | 100 Hz | yes |
| measured speed | DM2325 CAN feedback | rad/s | feedback driven | yes |
| measured torque | DM2325 CAN feedback | Nm | feedback driven | yes |
| position | DM2325 CAN feedback | rad | feedback driven | yes |
| MOS/rotor temperature | DM2325 CAN feedback | deg C | feedback driven | yes |
| current | current protocol | A | none | unsupported |
| bus voltage | current protocol | V | none | unsupported |
| reverse timestamps/state | C-board runtime | ms/state | 100 Hz/event | yes |
| feedback age/fault/CAN ESR | C-board runtime | ms/code/register | request snapshot | yes |

Stale measured fields are reported as null and rendered as `--`; unsupported
current and voltage are never reported as zero.

## 9. Verification

- C/Python repository tests: `python3 -m pytest -q tests` -> 26 passed.
- Software compatibility tests -> 33 passed.
- Offline benchmark: all sign reversals changed output on cycle 0 of the new
  phase (the next 10 ms firmware tick); no artificial zero wait was observed.
- Clean ARM build succeeded. ELF SHA-256:
  `74811de40096f7b78ce3ac029e370efa9949515249adbf3f1c1030a14eefe94a`.
- BIN SHA-256:
  `44e7c7ffeaac4e1fb697fd549d136f4cc5b2360630f8ea8d39e0fa85a5fb028c`.
- OpenOCD programming, flash verification, and reset succeeded.
- Post-flash passive check: Disabled, enabled mask 0, fault mask 0, runtime
  fault 0, normal profile, 10 ms period, CAN ESR 0.
- Browser QA confirmed the existing control page renders the profile selector,
  hold-to-run buttons, telemetry table, and reversal timing fields.

Software benchmark artifacts are `docs/dm2325_reverse_step_software.csv` and
`docs/dm2325_reverse_step_software.json`.

## 10. Performance comparison

Before: one 0.5 m/s2, 2 m/s3 chassis-axis ramp handled acceleration,
deceleration, and reversal; prior positive acceleration could persist after a
negative command.

After: wheel-side normal profile uses dedicated 40 rad/s2 reverse braking and
240 rad/s3 reverse jerk. A sign change starts changing the limited output on the
next 10 ms control tick in C host tests and the reference benchmark.

Hardware brake-to-zero and full reverse times are
`not_measured_on_hardware`. The web event telemetry is the measurement path for
the user's staged test.

## 11. Risks

Fast reversal can increase DC-bus regeneration, current, tire slip, gearbox and
mount loads, and temperature. The current protocol does not expose motor current
or bus voltage, so those protections remain drive-managed and cannot be claimed
as software-measured. Begin with the safe or normal profile and an unloaded,
clear chassis area.

## 12. Staged hardware procedure

1. Open the web control page and verify all four motors are online with no fault.
2. Select `safe`, enable control, and hold forward briefly; release to stop.
3. Hold forward, then hold backward while watching measured speed, torque, and
   the three reversal latency fields.
4. Repeat with `normal` only after the safe result is stable.
5. Test lateral, rotational, and combined reversals separately.
6. Use `competition_candidate` only with electrical and mechanical monitoring.
7. Release the control or disable immediately on fault, slip, impact, abnormal
   sound, unexpected temperature, or communication staleness.

## 13. File audit

The deployed production driver remains the single
`Software/src/utils/DM2325Bits.py` file. Firmware remains under
`~/C-type-Firmware`; web integration remains in the existing `control_panel.py`
and its existing static files. No commit or push was made. Generated caches,
staging directories, browser artifacts, and the temporary clean-build directory
were removed after verification; source, tests, benchmark outputs, and this
report were retained.
