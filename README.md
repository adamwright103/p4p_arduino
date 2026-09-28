# p4p_arduino

Low-level firmware for the P4P mecanum bot, running on an **Arduino Mega 2560**.

The Mega owns the hard-real-time work — motor mixing, the IMU, the safety
watchdog — and talks to a **Raspberry Pi** over USB serial. Path planning and
the docking filter live on the Pi; this firmware is deliberately dumb and fast.

## Hardware

| | |
|---|---|
| MCU | Arduino Mega 2560 (`arduino:avr:mega:cpu=atmega2560`) |
| IMU | BNO085 over I2C, addr `0x4A` — SDA = 20, SCL = 21 |
| Drive | 4× mecanum wheels on continuous-rotation servos — pins 46 (LF), 47 (LR), 50 (RR), 51 (RF) |
| Link | USB serial to the Pi, **115200 baud, 8N1** |

---

# Serial API

One command per line, `\n` terminated (a trailing `\r` is accepted). All
numbers are decimal ASCII. The link is full duplex: telemetry keeps streaming
while commands arrive.

## Coordinate frame

Right-handed, and it matches the IMU heading sign:

| axis | positive direction |
|---|---|
| `Vx` | **forward**, m/s |
| `Vy` | **left**, m/s |
| `Wz` | **anticlockwise** viewed from above, rad/s |

A positive `Wz` makes the reported `heading_rad` increase. Verified by
rotation test, not assumed.

## Commands — Pi to Mega

### `E` — enable

Arms the motors. **The bot boots disarmed and will not move until it receives
this.**

Arming deliberately zeroes any stored velocity, so `E` can never make the bot
lurch off on a stale command — a fresh `V` is always required after arming.

```
E
```

### `S` — stop

Disarms and zeroes immediately. This is the software e-stop. The bot ignores
`V` until the next `E`.

```
S
```

### `V` — set body velocity

```
V,<Vx>,<Vy>,<Wz>
```

Sets the commanded body twist. The bot **holds this velocity until a new `V`
arrives**, or until the watchdog expires, or until `S`.

Any number of decimal places from 0 to 6 is accepted (`strtod` parses them
natively). Leading `+`/`-`, and whitespace around fields, are fine.

```
V,1.0,0.5,1.0            1.0 m/s forward, 0.5 m/s left, 1.0 rad/s anticlockwise
V,0.25,0,0               straight ahead, slowly
V,0,0,-0.7854            rotate clockwise at 45 deg/s
V,0.000001,-2.5,0.1      6 dp, negatives
V,0,0,0                  hold station (still armed, watchdog still fed)
```

`V,0,0,0` is **not** the same as `S`: it keeps the bot armed and keeps feeding
the watchdog. Use it for "stay put, more coming". Use `S` to give up control.

### Lines the Mega ignores

Blank lines, and any line starting with `#`, are skipped silently and are not
counted as errors — so a captured telemetry log can be replayed at the bot
without tripping the error counter.

## Telemetry — Mega to Pi

Two kinds of line:

- **Anything starting with `#` is human-readable status.** The Pi-side parser
  must skip these. They appear at boot and on IMU trouble.
- **Everything else is a CSV data row**, sent at 50 Hz.

```
t_ms,heading_rad,yaw_rate_dps,vx,vy,wz,flags,resets,bad
```

| field | meaning |
|---|---|
| `t_ms` | `millis()` at the last IMU sample |
| `heading_rad` | continuous, unwrapped yaw. **Never wraps at ±π** — the Pi filter uses differences of it |
| `yaw_rate_dps` | calibrated gyro Z, degrees/s |
| `vx`, `vy`, `wz` | the velocity **actually applied** this cycle, SI, same frame as the command |
| `flags` | bitfield, see below |
| `resets` | count of spontaneous BNO085 resets. Should stay 0 |
| `bad` | count of malformed command lines rejected |

### The applied-velocity echo

`vx,vy,wz` are what the Mega is *acting on*, not what you asked for. They differ
from your command when:

- the bot is **disarmed** or the **watchdog has expired** → all three read `0`
- the command **exceeded what the wheels can do** → all three are scaled down by
  the same factor, and `flags` has `SAT` set

So "why isn't the robot doing what I said" is a diff, not a guessing game. This
also gives a clean place to add velocity ramping later — the echo will show the
ramp, the command won't.

### `flags` bitfield

| bit | value | name | meaning |
|---|---|---|---|
| 0 | 1 | `ARMED` | `E` received, `S` not yet |
| 1 | 2 | `TIMEOUT` | no `V` within 400 ms — motors zeroed |
| 2 | 4 | `SAT` | last command was scaled down to fit the wheels |

`flags=1` is the normal driving state. `flags=3` means armed but starved of
commands. `flags=0` means disarmed.

### Example session

```
# p4p_arduino starting
# BNO085 ready
# DISARMED -- send E to enable
t_ms,heading_rad,yaw_rate_dps,vx,vy,wz,flags,resets,bad
1476,0.0000,0.0,0.000,0.000,0.000,0,0,0        <- disarmed, flags=0
                                                >> E
1496,0.0000,0.0,0.000,0.000,0.000,3,0,0        <- armed but no V yet, flags=3
                                                >> V,0.2,0.0,0.0
1516,0.0000,0.1,0.200,0.000,0.000,1,0,0        <- driving, flags=1
1536,0.0002,0.3,0.200,0.000,0.000,1,0,0
   ... Pi goes quiet for 400 ms ...
1956,0.0410,0.2,0.000,0.000,0.000,3,0,0        <- watchdog fired, flags=3
                                                >> S
1976,0.0410,0.0,0.000,0.000,0.000,0,0,0        <- disarmed
```

## Safety model

Three independent layers, in order of severity:

1. **Boots disarmed.** Power-on and reset both land in a state where no `V` can
   move the bot. Opening a serial monitor resets the Mega via DTR, so this
   happens more often than you would think.
2. **Command watchdog, 400 ms** (`CMD_TIMEOUT_MS`). If no `V` arrives in that
   window the motors go to neutral. The bot **stays armed** — resuming commands
   resumes motion, no re-arm handshake needed — so a brief comms hiccup does not
   require operator intervention. `flags` shows `TIMEOUT` throughout.
3. **`S`**, the explicit e-stop. Disarms; only `E` recovers.

At 1 m/s, 400 ms is ~400 mm of open-loop travel before the watchdog bites.
Size your command rate accordingly — 20 Hz commands means 8 consecutive
missed messages before a stop.

The watchdog is evaluated in the **control task, not the parser**, so a flood of
inbound commands cannot starve it and a silent Pi cannot leave a stale velocity
latched on the motors.

---

# Calibration

> **`WHEEL_MAX_MPS` in `config.h` is an unmeasured placeholder (0.50 m/s).**
> Until it is measured, commanded velocities are *proportional but not
> accurate* — `V,1.0,0,0` will move the bot, but not at 1 m/s.

There are **no wheel encoders** on this chassis, so velocity is open loop.
Commanded speed is what the wheels are told to do, not what they achieve; it
will sag under load, on carpet, and as the battery drops.

`WHEEL_MAX_MPS` is the single constant that converts SI to PWM. Everything else
follows:

```
max forward / strafe = WHEEL_MAX_MPS
max rotation         = WHEEL_MAX_MPS / CHASSIS_L_PLUS_W    (= 0.24 m)
```

**Rotation calibrates itself for free.** The BNO085 already reports
`yaw_rate_dps`, so command a few pure rotations, read the steady-state rate, and
fit the line:

```
E
V,0,0,0.5      ... read yaw_rate_dps ...
V,0,0,1.0      ... read yaw_rate_dps ...
V,0,0,2.0      ... read yaw_rate_dps ...
S
```

If the measured rate is consistently `k ×` the commanded one, divide
`WHEEL_MAX_MPS` by `k`.

**Translation needs a tape measure.** Drive at a known `Vx` for a known time
and measure the displacement. Do it again for `Vy` — mecanum wheels slip more
sideways than forwards, typically 20–40% worse, and `STRAFE_GAIN` exists to
compensate once you have the number.

# Bring-up

Do this **with the wheels off the ground** the first time. One axis at a time,
checking against `config.h`'s three sign trims:

| step | command | expect | if wrong |
|---|---|---|---|
| 1 | `V,0.2,0,0` | all four wheels drive forward | flip `DRIVE_VX_SIGN` |
| 2 | `V,0,0.2,0` | bot strafes **left** | flip `DRIVE_VY_SIGN` |
| 3 | `V,0,0,0.5` | bot rotates **anticlockwise**, and `heading_rad` **increases** | flip `DRIVE_WZ_SIGN` |
| 4 | stop sending | motors neutral within 400 ms, `flags` gains `TIMEOUT` | check the watchdog |

If a *single* wheel runs backwards rather than a whole axis, that is wiring or
a mirrored motor — fix `MOTOR_MIRROR_LEFT` / `MOTOR_MIRROR_RIGHT`, or the
mounting, not the axis trims.

Step 3 is the one that matters most: it is the only direct check that the drive
and the IMU agree on which way is positive.

**Watch `resets` during the first drive test.** The Servo library's timer ISR
can disturb I2C timing once four motors are attached. If `resets` starts
climbing above 0 only when the motors are live, that is the cause.

---

# Layout

| file | role |
|---|---|
| `p4p_arduino.ino` | `setup()` / `loop()`, task scheduling, telemetry format |
| `config.h` | every tunable constant — pins, rates, limits, geometry, sign trims |
| `timing.h` | `RateTimer`, non-blocking fixed-rate timer. One per periodic task instead of `delay()` |
| `imu.h` / `.cpp` | BNO085 wrapper, continuous unwrapped heading |
| `drive.h` / `.cpp` | mecanum mixing, saturation, motor output. The only file that knows wheels exist |
| `command.h` / `.cpp` | serial command parser. Knows nothing about motors |
| `system_setup.h` / `.cpp` | everything `setup()` has to do |

`loop()` runs three things: `imu.poll()` and `cmd.poll()` every pass (both
non-blocking), a 50 Hz control task, and a 50 Hz telemetry task. **No `delay()`
in `loop()`** — add periodic work as another `RateTimer`.

## Dependencies

```bash
arduino-cli lib install "Adafruit BNO08x"   # pulls in Adafruit BusIO
arduino-cli lib install "Servo"
```

## Build and flash

```bash
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
```

## Talking to it by hand

```bash
stty -F /dev/ttyACM0 115200 raw -echo -hupcl clocal
cat /dev/ttyACM0 &            # watch telemetry
printf 'E\n'            > /dev/ttyACM0
printf 'V,0.2,0,0\n'    > /dev/ttyACM0
printf 'S\n'            > /dev/ttyACM0
```

The `-hupcl` matters: without it the Mega resets when a reader exits.

Only one process can hold `/dev/ttyACM0` at a time. If a flash fails as busy,
`fuser -v /dev/ttyACM0` names the culprit — usually an open Serial Monitor.

## Conventions

- No `delay()` in `loop()`. Add periodic work as another `RateTimer`.
- Constants go in `config.h`, not inline.
- New peripherals are brought up in `systemSetup()`.
- Mixing stays textbook; hardware quirks live in the mirror/sign constants so
  they stay visible.
