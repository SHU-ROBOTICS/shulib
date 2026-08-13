# First boot on a V5 brain — 2026-08-12

Raw capture from the V5 user port (`pros terminal`), unedited.

- Built from: `main` @ `d4fac9c`
- Binary: `bin/hot.package.bin`, sha256 `7e5552cab758c1120ab19dd9c2a72ddf...` (first 32)
- Brain: VEX V5 `08CC9E00`, PROS kernel 4.2.2, platform V1.1.5 (b18)
- HAL: **fake-backed** — no motors or sensors attached, nothing driven.

```text
Version:[1m        4.2.2[0m     Platform:  [1mV1.1.5 (b18)[0m     Uptime:[1m    0.000 s[0m
Compiled:   [1mAug 12 2026 21:48:20[0m     Directory:  [1m                      [0m


[t=   0.00] [C7] shulib v2 core wired: X-drive kinematics + Pilons odometry + fused localizer + motion scheduler + Chassis facade
[t=   0.00] [WARN][C7] HAL is fake-backed: hal/pros adapters are chunk R1 — this binary CANNOT drive hardware
[t=   0.00] [C7] facade alive: strafeAuthority=1.00 (X-drive: 1.00)
[t=   0.00] [C7] opcontrol(): idle — teleop drive() loop arrives with R1's adapters

```
