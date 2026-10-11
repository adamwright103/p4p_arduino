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
| Drive | 4× mecanum wheels on continuous-rotation servos — pins 51 (LF), 46 (LR), 47 (RR), 50 (RF). **Not the shield's pin order** — see `config.h` |
| Link | USB serial to the Pi, **500000 baud, 8N1** |
| IMU bus | I2C at **400 kHz** (Fast mode), reports at **200 Hz** |

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
V,0.25,0.15,0.5          0.25 m/s forward, 0.15 m/s left, 0.5 rad/s anticlockwise
V,0.22,0,0               straight ahead
V,0,0,-0.7854            rotate clockwise at 45 deg/s
V,0.209000,-0.238,0.1    6 dp, negatives
V,0,0,0                  hold station (still armed, watchdog still fed)
```

`V,0,0,0` is **not** the same as `S`: it keeps the bot armed and keeps feeding
the watchdog. Use it for "stay put, more coming". Use `S` to give up control.

#### Velocities are real m/s, within a narrow range

`Vx` and `Vy` go through a **measured** lookup table (`SPEED_MAP_FORWARD` and
`SPEED_MAP_STRAFE` in `config.h`), so a command means what it says — inside the
range that was measured, which is small:

| axis | slowest | fastest |
|---|---|---|
| forward / reverse | 0.21 m/s | **0.33 m/s** |
| left / right strafe | 0.13 m/s | **0.28 m/s** |

The firmware prints these at boot. Three things happen at the edges, and none
of them is an error the Pi can see in telemetry:

- **Too fast** — the command saturates at the top of the table. `V,1.0,0,0` is
  0.33 m/s, not 1.0. Nothing is reported; the bot simply travels slower than
  the Pi believes.
- **Too slow but non-zero** — the command snaps *up* to the slowest measured
  speed. Below that the servos are in their deadband and do not turn at all, so
  `V,0.02,0,0` would otherwise be a silent stall. Moving faster than asked is
  the lesser evil, but it is still wrong, so treat the slowest column as a hard
  floor on any approach speed.
- **Exactly zero** — zero deflection. Hold station means stationary.

A strafe is slower than a drive at the same wheel command and there is no gain
that fixes it, so plan paths accordingly: 0.3 m/s is reachable forwards and
not sideways. See "Calibration" below.

`Wz` is **not** calibrated. It is scaled through the forward curve as a proxy
and has never been measured on this chassis.

### Lines the Mega ignores

Blank lines, and any line starting with `#`, are skipped silently and are not
counted as errors — so a captured telemetry log can be replayed at the bot
without tripping the error counter.

## Telemetry — Mega to Pi

Two kinds of line:

- **Anything starting with `#` is human-readable status.** The Pi-side parser
  must skip these. They appear at boot and on IMU trouble.
- **Everything else is a CSV data row**, sent at 200 Hz.

```
t_ms,heading_rad,ax,ay,yaw_rate_dps
```

| field | meaning | format |
|---|---|---|
| `t_ms` | `millis()` at the last IMU heading sample | integer |
| `heading_rad` | continuous, unwrapped yaw. **Never wraps at ±π** — the Pi filter uses differences of it | 4 dp |
| `ax` | fused acceleration, sensor X, m/s² | 3 dp |
| `ay` | fused acceleration, sensor Y, m/s² | 3 dp |
| `yaw_rate_dps` | calibrated gyro Z, degrees/s | 1 dp |

Comma separated, no spaces, `\r\n` terminated. Every field is a decimal ASCII
number; `ax`, `ay`, `yaw_rate_dps` and `heading_rad` may be negative.

### Example session

```
# p4p_arduino starting
# BNO085 ready
# speed range, forward 0.209 to 0.327 m/s, strafe 0.132 to 0.275 m/s
# outside that: faster saturates, slower snaps up, zero is zero
# DISARMED -- send E to enable
t_ms,heading_rad,ax,ay,yaw_rate_dps
1476,0.0000,0.004,-0.011,0.0        <- stationary, noise floor only
                                     >> E
                                     >> V,0.2,0.0,0.0
1481,0.0000,0.138,-0.007,0.1        <- accelerating forward
1486,0.0001,0.151,-0.004,0.3
1491,0.0002,0.094,0.002,0.4
1496,0.0004,0.031,-0.004,0.4        <- up to speed, ax falls back to ~0
1501,0.0006,0.008,0.009,0.3
                                     >> S
1506,0.0008,-0.142,0.012,0.2        <- decelerating: ax negative
```

Note what `ax` does: it is *acceleration*, so it spikes on the speed change
and decays to the noise floor during steady driving. It is not a speedometer —
integrating it to get velocity will drift, which is the Pi filter's problem to
solve.

A row is purely IMU state — nothing the Mega did to the motors appears in it.
The Pi already knows what it commanded, and the firmware no longer echoes it
back. See "What the row does not tell you" below for the consequences.

### Everything in a row is an on-chip fused report

Nothing here is raw sensor data integrated on the Mega. Three BNO085 reports
are enabled at `IMU_REPORT_US` (200 Hz by default):

| quantity | SH-2 report | what the chip does |
|---|---|---|
| `heading_rad` | `SH2_GAME_ROTATION_VECTOR` | quaternion fused from accelerometer + gyro. **No magnetometer** — datasheet §2.2.2 |
| `ax`, `ay` | `SH2_LINEAR_ACCELERATION` | accelerometer fused with gyro, **gravity removed** — datasheet §2.1.1 |
| `yaw_rate_dps` | `SH2_GYROSCOPE_CALIBRATED` | gyro with the on-chip bias correction applied |

The game rotation vector is the right orientation output for this bot. The full
`SH2_ROTATION_VECTOR` adds the magnetometer to correct yaw drift, but indoors,
on a chassis carrying four motors, the magnetic field is neither constant nor
trustworthy, and the datasheet notes the corrections arrive as *jumps* in yaw.
The game vector trades absolute heading for a smooth relative one. Since the
docking filter only ever uses heading *differences*, drift that the filter
never sees is cheaper than a step change that it does.

`ax`/`ay` are gravity-removed on purpose: at a few degrees of chassis tilt,
gravity leaks into the horizontal axes at ~0.17 m/s² per degree, which is the
same order as real driving acceleration and indistinguishable from it. Set
`IMU_ACCEL_GRAVITY_REMOVED = false` in `config.h` to stream the calibrated
accelerometer (`SH2_ACCELEROMETER`) instead, gravity included.

**`ax`/`ay` are in the sensor frame, not the body frame.** They are passed
through untouched, so they only line up with `Vx`/`Vy` if the chip is mounted
square to the chassis. See "Bring-up" below.

### Why 200 Hz needs 400 kHz I2C and 500000 baud

200 Hz is three bottlenecks deep, and raising `STREAM_RATE_HZ` alone would
quietly hit all three. The 5 ms budget per row:

| | cost at 200 Hz | share of 5 ms |
|---|---|---|
| I2C, three reports, 400 kHz | ~1.2 ms | 24% |
| I2C, three reports, 100 kHz (the Wire default) | **~4.9 ms** | **98%** |
| Serial, ~46 byte row, 500000 baud | ~0.9 ms | 18% |
| Serial, ~46 byte row, 115200 baud | ~4.0 ms | 80% |

The I2C figure is the one that forces the issue. The Adafruit library has no
interrupt line in its I2C path, so every `getSensorEvent()` first reads a
4-byte SHTP header to ask whether anything is waiting, then reads the cargo in
≤32-byte chunks (the AVR `Wire` buffer), re-reading a 4-byte header on each
chunk after the first. One batched transfer of timebase + game rotation vector
+ acceleration + gyro is ~43 bytes, so it costs a header probe plus two
chunked reads. At 100 kHz that is ~4.9 ms and does not fit in the period at
all; at 400 kHz it is ~1.2 ms. 400 kHz is Fast mode, the BNO085's documented
ceiling (datasheet §1.2.2, §6.5.1) — there is no headroom above it.

The serial figure matters less but still bites: at 115200 an 80%-full link
means `Serial.print` blocks on a full 64-byte TX buffer most rows, and that
blocking happens inside `loop()`, stealing time from `imu.poll()`. At 500000
a whole row fits in the TX buffer and the call never blocks.

`Wire.setClock()` is called *after* `imu.begin()` in `systemSetup()`, because
`Adafruit_I2CDevice::begin()` calls `Wire.begin()`, which resets the prescaler
back to 100 kHz. Setting it earlier silently does nothing.

### Verify the rate on real hardware

None of the above is measured — it is a byte count against a clock, and it
ignores `Servo`'s timer ISR, which is known to disturb I2C on this chassis.
Count the rows:

```bash
timeout 10 cat /dev/ttyACM0 | grep -c ','     # expect ~2000
```

If it comes in low, the honest fix is 100 Hz: set `STREAM_RATE_HZ = 100` and
`IMU_REPORT_US = 10000`. A `RateTimer` that cannot keep up does not queue —
it resynchronises and drops the row, so an overloaded 200 Hz stream degrades
into a jittery ~150 Hz one rather than failing loudly.

**`t_ms` is the weak link at this rate.** It comes from `millis()`, so it is
quantised to 1 ms — 20% of a 5 ms period. Differencing consecutive rows gives
`dt` values of 4, 5 or 6 ms for a perfectly regular stream, and integrating
`ax` against that adds noise that is pure measurement artefact. The BNO085
timestamps every report itself, in microseconds (`sh2_SensorValue_t.timestamp`),
and switching `t_ms` to that is a small change worth making if the Pi-side
filter integrates acceleration.

### What the row does not tell you

Three fields that older rows carried are gone: `flags`, `resets` and `bad`.
Dropping them means the Pi can no longer see, from telemetry alone:

- **`flags`** — whether the bot is armed, whether the 400 ms watchdog has
  expired, and whether the last command was scaled down to fit the wheels.
  All three safety layers still work exactly as before; they are just no
  longer reported. A disarmed bot and a bot driving correctly now produce
  identical-looking rows, so "it isn't moving" is no longer diagnosable from
  the log.
- **`resets`** — the count of spontaneous BNO085 resets. A reset makes the
  heading restart from its current yaw without warning, and that is now
  invisible: it looks like a clean, continuous heading. `Imu::resetCount()`
  still counts them, so this is one line of firmware away from being
  reported again if it is ever needed.
- **`bad`** — the count of malformed command lines. A Pi corrupting its own
  output now gets no feedback. `Command::badCount()` still counts them.

The applied-velocity echo (`vx`, `vy`, `wz`) is gone with them, so the
"what I asked for vs. what the chassis delivered" diff is no longer available
either. Saturation in particular is now silent.

## Safety model

Three independent layers, in order of severity:

1. **Boots disarmed.** Power-on and reset both land in a state where no `V` can
   move the bot. Opening a serial monitor resets the Mega via DTR, so this
   happens more often than you would think.
2. **Command watchdog, 400 ms** (`CMD_TIMEOUT_MS`). If no `V` arrives in that
   window the motors go to neutral. The bot **stays armed** — resuming commands
   resumes motion, no re-arm handshake needed — so a brief comms hiccup does not
   require operator intervention. **This is no longer visible in telemetry** —
   see "What the row does not tell you" above.
3. **`S`**, the explicit e-stop. Disarms; only `E` recovers.

At the measured top speed of 0.33 m/s, 400 ms is ~130 mm of open-loop travel
before the watchdog bites.
Size your command rate accordingly — 20 Hz commands means 8 consecutive
missed messages before a stop.

The watchdog is evaluated in the **control task, not the parser**, so a flood of
inbound commands cannot starve it and a silent Pi cannot leave a stale velocity
latched on the motors.

---

# Calibration

There are **no wheel encoders** on this chassis, so velocity is open loop.
Commanded speed is what the wheels are told to do, not what they achieve; it
will sag under load, on carpet, and as the battery drops.

## The top of the range does not exist

The old `WHEEL_MAX_MPS = 0.50 m/s` was a guess, and measurement killed it:

| wheel command | forward | left strafe |
|---|---|---|
| 20% | 0.209 m/s | 0.132 m/s |
| 40% | 0.303 m/s | 0.238 m/s |
| 60% | 0.327 m/s | 0.275 m/s |
| 80% | 0.329 m/s | 0.264 m/s |
| 100% | wheels do not turn correctly | wheels do not turn correctly |

Timed 1 m runs, 2026-10-12, single trials — `motor_cal/RESULTS.md`.

**Max forward is ~0.33 m/s.** 0.5 m/s is not slow, not uncalibrated, not a
tuning problem: it is unreachable, and so is anything above a third of a metre
per second. Any path planner built around half a metre per second needs
rewriting, not retuning.

Three more things the table says:

- **The curve is flat above 60%.** 60% to 80% gains forward 0.002 m/s and
  *loses* a strafe 0.011. Extra deflection up there buys current draw.
- **100% does not work at all**, on either axis. `MOTOR_MAX_DEFLECTION = 0.80`
  in `config.h` makes it unreachable: saturation scales to that ceiling rather
  than to full deflection, so no command can ask for it.
- **It is not a straight line, and it misses the origin.** A line through the
  20% and 40% forward points hits 0.115 m/s at zero command, which is
  impossible — below 20% the curve must fall off a cliff into the servo
  deadband. That is why the conversion is a table and not a constant.

## Strafing is slower, and that is the answer

A strafe runs 21–37% slower than a drive at the same wheel command, worst at
low speed:

| wheel command | forward / strafe |
|---|---|
| 20% | 1.59× |
| 40% | 1.27× |
| 60% | 1.19× |

Mecanum rollers slip sideways; roughly a third of the wheel speed goes into
roller rotation instead of chassis motion. **There is deliberately no
`STRAFE_GAIN`.** A gain would be a request for wheel speed the chassis does not
convert into motion — and since the ratio is not even constant, no single
number fits. The strafe axis gets its own measured table instead, and a request
past the end of it saturates and stays honest.

## The map

`config.h` holds two tables of `{speed, wheel command}` rows from timed runs;
`speedmap.cpp` interpolates them linearly and inverts them for the
applied-velocity echo. Each axis has its own table because the axes genuinely
differ. Both directions of an axis share one, which assumes servo response is
symmetric about neutral — the one part of this that is assumed rather than
measured.

Rows must rise in both columns or the map cannot be inverted;
`static_assert`s in `speedmap.h` enforce it, so pasting in a row from the flat
top of the curve fails the build instead of producing a map that silently
cannot be read backwards. After editing a table, run the host tests:

```bash
./test/run_tests.sh
```

To remeasure, see `motor_cal/README.md`. The rig drives
`Drive::setBodyDeflection()`, which bypasses the map, so it never measures its
own output.

## Rotation is still uncalibrated

`Wz` has never been measured. `rotationDeflection()` feeds the rotation term
through the *forward* curve, on the grounds that for a pure drive the wheel's
contact-patch speed and the body speed are the same number — which makes the
forward table a stand-in for a wheel-speed curve. For a rotation, where every
roller is slipping, it is a proxy and nothing more.

**Rotation calibrates itself for free once the IMU is back**, since the BNO085
reports `yaw_rate_dps` directly:

```
E
V,0,0,0.5      ... read yaw_rate_dps ...
V,0,0,1.0      ... read yaw_rate_dps ...
S
```

Build a third table from that the same way, and give `rotationDeflection()`
its own rows.

# Bring-up

Do this **with the wheels off the ground** the first time. One axis at a time,
checking against `config.h`'s three sign trims:

| step | command | expect | if wrong |
|---|---|---|---|
| 1 | `V,0.25,0,0` | all four wheels drive forward | see "a quarter turn" below |
| 2 | `V,0,0.2,0` | bot strafes **left** | as above |
| 3 | `V,0,0,0.5` | bot rotates **anticlockwise**, and `heading_rad` **increases** | flip `DRIVE_WZ_SIGN` |
| 4 | stop sending | motors neutral within 400 ms — watch the wheels, not the telemetry | check the watchdog |
| 5 | push the bot forward by hand | `ax` goes positive, `ay` stays near 0 | the chip is not mounted square — see below |
| 6 | push the bot to the left by hand | `ay` goes positive, `ax` stays near 0 | as above |

If a *single* wheel runs backwards rather than a whole axis, that is wiring or
a mirrored motor — fix `MOTOR_MIRROR_LEFT` / `MOTOR_MIRROR_RIGHT`, or the
mounting, not the axis trims.

**Step 3 has never been verified on this chassis.** Steps 1 and 2 are now
measured (see below), but the IMU was unplugged when that was done, so nothing
has yet confirmed that `+Wz` turns the bot the way `heading_rad` says. It is
the only direct check that the drive and the IMU agree on which way is
positive, and `DRIVE_WZ_SIGN` is the fix if they disagree.

### If an axis is wrong, it may be a quarter turn, not a sign

`DRIVE_VX_SIGN` and `DRIVE_VY_SIGN` can only flip an axis, and on this chassis
the fault was not a flip. As shipped from the shield's pin order, `+Vx` made
the bot strafe **left** and `+Vy` drove it **backwards**: the body frame was
rotated 90°, which no pair of sign flips can express.

The fix was the pin-to-corner mapping in `config.h` — `PIN_LEFT_FRONT` and
friends are now ordered by *physical corner*, derived from the measured pulse
patterns, with the mixing and the mirrors untouched. If translation ever comes
out rotated again, check that mapping before reaching for a sign trim.

Steps 5 and 6 check the accelerometer axes, which the firmware passes through
untouched. If they come out swapped, negated, or mixed, the chip is rotated
relative to the chassis: either remount it square, or rotate `ax`/`ay` in
`Imu::poll()` — there is deliberately no sign trim for them in `config.h`,
because a rotation is not always expressible as two sign flips.

**Watch for spontaneous BNO085 resets during the first drive test.** The Servo
library's timer ISR can disturb I2C timing once four motors are attached. The
count is no longer streamed, so to see it you have to add `imu.resetCount()`
back into `printTelemetry()` for the test — a heading that silently restarts
mid-drive is otherwise very hard to distinguish from a real turn.

---

# Layout

| file | role |
|---|---|
| `p4p_arduino.ino` | `setup()` / `loop()`, task scheduling, telemetry format |
| `config.h` | every tunable constant — pins, rates, limits, geometry, sign trims, speed-map tables |
| `timing.h` | `RateTimer`, non-blocking fixed-rate timer. One per periodic task instead of `delay()` |
| `imu.h` / `.cpp` | BNO085 wrapper — fused heading, acceleration, gyro yaw rate |
| `drive.h` / `.cpp` | mecanum mixing, saturation, motor output. The only file that knows wheels exist |
| `speedmap.h` / `.cpp` | measured velocity ↔ wheel command, per axis. The only thing that makes a `V` mean m/s |
| `motor_cal/` | the calibration rig that produces those tables — a separate sketch, with its own README |
| `test/` | host-side tests. `./test/run_tests.sh`, no Arduino toolchain needed |
| `command.h` / `.cpp` | serial command parser. Knows nothing about motors |
| `system_setup.h` / `.cpp` | everything `setup()` has to do |

`loop()` runs three things: `imu.poll()` and `cmd.poll()` every pass (both
non-blocking), a 50 Hz control task, and a 200 Hz telemetry task. **No `delay()`
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
stty -F /dev/ttyACM0 500000 raw -echo -hupcl clocal
cat /dev/ttyACM0 &            # watch telemetry
printf 'E\n'            > /dev/ttyACM0
printf 'V,0.25,0,0\n'   > /dev/ttyACM0      # 0.2 would snap up to 0.209
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
