# motor_cal — open-loop speed calibration

Drives the chassis in a straight line at one fixed motor command so you can
time it over a measured metre by hand. A sweep across both axes gives the
command→speed curve that `SPEED_MAP_FORWARD` and `SPEED_MAP_STRAFE` in
`config.h` are built from.

Measurements, the protocol, and what is still missing are in
[RESULTS.md](RESULTS.md). Round 1 is in there: max forward is **~0.33 m/s**,
the curve is **flat above 60%**, and everything below 20% is unmapped.

## Why it is a separate sketch

The firmware blocks in `systemSetup()` until the BNO085 answers on I2C:

```cpp
while (!imu.begin(IMU_I2C_ADDR, IMU_REPORT_US)) { ... delay(1000); }
```

With the IMU unplugged it never gets past that, so it cannot drive at all.
This sketch touches neither the IMU nor the command parser.

`config.h`, `drive.h`, `drive.cpp`, `speedmap.h` and `speedmap.cpp` here are
**symlinks** to the firmware's own files. The mixing, pulse widths, pin mapping
and sign trims under test are byte-for-byte the ones that ship — do not replace
them with copies, or you will calibrate code you do not run. (The speed map is
only there to satisfy the linker; see below.)

## Use

```bash
./run.sh 20 F        # 20% forward: patch, compile, flash
./run.sh 30 L        # 30% left strafe
./run.sh 30 L -n     # patch and compile only, no flash
PORT=/dev/ttyACM1 ./run.sh 40 F
```

`F` and `L` are the firmware's `+Vx` and `+Vy`, which since the pin mapping was
fixed are real chassis directions — forward and left. See "Axes" below.

The cap is **80%**, `MOTOR_MAX_DEFLECTION`. Full deflection does not turn the
wheels correctly, so it is blocked in `config.h` for the firmware too. Asking
for more is a build error, not a silent clamp — a calibration rig that quietly
drives slower than it claims is worse than one that refuses.

**The flash itself resets the Mega, so the run starts ~1 s after the upload
finishes.** Have the bot on the floor, at the start line, pointed where you
want it, *before* you run the command.

To repeat a trial at the same setting, **tap RESET on the Mega** — no re-flash.
Re-opening the serial port does the same thing, via DTR.

Or edit the two constants at the top of `motor_cal.ino` and flash by hand:

```bash
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
```

## Stopping it

- Any character on the serial link stops the motors immediately.
- `RUN_MS` (15 s) is a hard backstop, so a missed catch is not a wall.
- Cutting power works too. The bot is open loop; nothing is holding state.

Opening a serial monitor **resets the board and starts a fresh run** — it is
not a safe way to watch a run in progress. Open it before the run, or not at
all.

## Axes: fixed in the firmware, not worked around here

Round 1 measured the firmware's frame as a quarter turn out from the chassis:

| old command | pin pulses (46,47,50,51) | bot actually went |
|---|---|---|
| `+Vx` | `(-,-,+,+)` | strafed **left** |
| `+Vy` | `(+,-,-,+)` | drove **backwards** |

Textbook mecanum needs left-vs-right opposite to translate along x, and
front-vs-rear opposite to strafe. Those patterns only fit if the shield's pin
order sits at different physical corners than the names say — so the fix was a
permutation of `PIN_LEFT_FRONT` and friends in `config.h`, leaving the mixing
and the mirror constants alone. Sign trims could not have done it: a quarter
turn is not two sign flips.

So this rig drives `+Vx` and `+Vy` directly and they point where they claim.
**Rotation is still unverified** — `+Wz` now puts the same pulse sign on all
four wheels, which is structurally right, but nothing has confirmed it turns
anticlockwise. That needs the IMU, or at least an eyeball, with the wheels off
the ground.

## What the percentage means

`SPEED_PCT` is a percentage of **full pulse deflection**, i.e. the servo
command, not a velocity:

```
pulse = MOTOR_US_NEUTRAL ± (pct/100) × (MOTOR_US_MAX − MOTOR_US_NEUTRAL)
      = 1500 ± (pct/100) × 800 µs
```

The sketch calls `Drive::setBodyDeflection()`, which takes that fraction
directly and **bypasses the speed map**. That is the point: the map is built
out of these runs, so a rig that read it would be measuring its own output.
It also means nothing I change in the map can disturb a sweep in progress.

All four wheels sit at the same deflection magnitude on both axes:

| axis | LF | RF | LR | RR |
|---|---|---|---|---|
| `F` forward (`+Vx`) | +f | +f | +f | +f |
| `L` left (`+Vy`) | −f | +f | +f | −f |

so neither sweep is saturated and the two are directly comparable. The sketch
prints a warning and you should discard the run if `Drive` ever scales the
command down anyway.

## After the runs

Round 1 already ruled out the single-constant approach, so there is no longer
one: a gain would have had to be 1.59 at 20% and 1.19 at 60% to fit the strafe
data, and the old `WHEEL_MAX_MPS` described a full-deflection speed that is
neither reachable nor useful. Both assumed a straight line through the origin,
and the measured curve is neither straight nor headed for the origin.

The tables are the deliverable. Paste the measured rows into
`SPEED_MAP_FORWARD` and `SPEED_MAP_STRAFE` in `config.h` — ascending, and
rising in both columns, which `speedmap.h` enforces at compile time — then run
`../test/run_tests.sh`. After that a `V` command over serial is in real m/s.
