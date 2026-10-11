# p4p_arduino

Low-level firmware for the P4P mecanum bot, running on an **Arduino Mega 2560**.

The Mega owns the hard-real-time work — motor mixing, the measured speed map,
the safety watchdog — and talks to a **Raspberry Pi** over USB serial. Path
planning and the docking filter live on the Pi; this firmware is deliberately
dumb and fast.

**The IMU is no longer here.** The BNO085 moved to the Pi, which owns it
directly over its own I2C bus (`p4p_imu` in the conv-chart repo). This
firmware drives wheels and reports what it drove them at. Nothing more.

## Hardware

| | |
|---|---|
| MCU | Arduino Mega 2560 (`arduino:avr:mega:cpu=atmega2560`) |
| Drive | 4× mecanum wheels on continuous-rotation servos — pins 51 (LF), 46 (LR), 47 (RR), 50 (RF). **Not the shield's pin order** — see `config.h` |
| Link | USB serial to the Pi, **115200 baud, 8N1** |
| Rates | commands and motor update **50 Hz**, telemetry **50 Hz** |

---

# Serial API

One command per line, `\n` terminated (a trailing `\r` is accepted). All
numbers are decimal ASCII. The link is full duplex: telemetry keeps streaming
while commands arrive.

## Coordinate frame

Right-handed, and it is REP-103, so it matches the frame the Pi's IMU node
reports in:

| axis | positive direction |
|---|---|
| `Vx` | **forward**, m/s |
| `Vy` | **left**, m/s |
| `Wz` | **anticlockwise** viewed from above, rad/s |

A positive `Wz` should make the Pi's reported heading increase. **That one is
not verified** — the IMU was unplugged when the drive frame was measured, so
`Vx` and `Vy` are confirmed against a tape measure and `Wz` is not. See
"Calibration".

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
| forward / reverse | 0.138 m/s | **0.275 m/s** |
| left / right strafe | 0.101 m/s | **0.227 m/s** |

The firmware prints these at boot. Three things happen at the edges, and none
of them is an error the Pi can see in telemetry:

- **Too fast** — the command saturates at the top of the table. `V,1.0,0,0` is
  0.275 m/s, not 1.0. Nothing is reported; the bot simply travels slower than
  the Pi believes. The applied-velocity uplink is the only way to see it.
- **Too slow but non-zero** — the command snaps *up* to the slowest measured
  speed. Below that the servos are in their deadband and do not turn at all, so
  `V,0.02,0,0` would otherwise be a silent stall. Moving faster than asked is
  the lesser evil, but it is still wrong, so treat the slowest column as a hard
  floor on any approach speed.
- **Exactly zero** — zero deflection. Hold station means stationary.

A strafe is slower than a drive at the same wheel command and there is no gain
that fixes it, so plan paths accordingly: 0.25 m/s is reachable forwards and
not sideways. See "Calibration" below.

`Wz` is **not** calibrated. It is scaled through the forward curve as a proxy
and has never been measured on this chassis.

### Lines the Mega ignores

Blank lines, and any line starting with `#`, are skipped silently and are not
counted as errors — so a captured telemetry log can be replayed at the bot
without tripping the error counter.

## Telemetry — Mega to Pi

One kind of data line, in the same shape the Pi's commands arrive in: tag
first, comma separated, three decimal places.

```
A,<vx>,<vy>,<wz>
```

Sent at `STREAM_RATE_HZ` (50 Hz), `\r\n` terminated. **Anything starting with
`#` is human-readable status** and the Pi-side parser must skip it; those
appear at boot and nowhere else.

| field | meaning | format |
|---|---|---|
| `vx` | applied forward velocity, m/s | 3 dp |
| `vy` | applied left velocity, m/s | 3 dp |
| `wz` | applied anticlockwise rate, rad/s | 3 dp |

Three decimals because that is what the Pi's `format_velocity()` sends, and
because 0.001 m/s is already finer than an open-loop chassis with no encoders
can deliver.

### Applied, not requested

This is the twist the chassis was actually given: after the speed map, after
saturation scaling, after the watchdog. It is **not** the request echoed back,
and on this chassis the two differ often — the reachable range is narrow, so a
`V` outside it is clamped silently and this row is the only evidence.

The Pi already knows what it asked for. What it cannot know without this row
is what the drive did with it.

### What a zero row does not tell you

`A,0.000,0.000,0.000` means one of four things and does not distinguish them:

- disarmed — no `E` yet, or an `S` since;
- the 400 ms command watchdog expired;
- a deliberate `V,0,0,0`, holding station;
- a request below the deadband that the map could not honour.

The firmware used to stream a `flags` bitfield carrying armed / timed-out /
saturated, plus counters for IMU resets and malformed commands. Those are gone
with the IMU row. All three safety layers still work exactly as before and
`Command::badCount()` still counts, so reporting any of it again is a line of
firmware away — but as it stands, a disarmed bot and a bot holding station
produce identical rows, and "it isn't moving" is not diagnosable from the log
alone. The Pi's bridge tracks the distinction on its own side, which is why
nothing here does.

### Example session

```
# p4p_arduino starting
# speed range, forward 0.138 to 0.275 m/s, strafe 0.101 to 0.227 m/s
# outside that: faster saturates, slower snaps up, zero is zero
# DISARMED -- send E to enable
A,0.000,0.000,0.000                 <- disarmed, holding neutral
                                     >> E
                                     >> V,0.25,0,0
A,0.250,0.000,0.000                 <- asked for 0.25, got 0.25
                                     >> V,1.0,0,0
A,0.275,0.000,0.000                 <- asked for 1.0, clamped to the top row
                                     >> V,0,0.5,0
A,0.000,0.227,0.000                 <- sideways tops out lower still
                                     >> V,0.02,0,0
A,0.138,0.000,0.000                 <- below the deadband: snapped UP
                                     >> S
A,0.000,0.000,0.000                 <- disarmed
```

Every one of those clamps is silent on the downlink. The row is where they
show up.

### The Pi's parser is currently stale

`p4p_serial_bridge/protocol.py` in the conv-chart repo still expects the old
nine-field IMU row:

```
t_ms,heading_rad,yaw_rate_dps,vx,vy,wz,flags,resets,bad
```

It also defaults to `baud = 115200` while the firmware was until now at
500000, so the two have not agreed for a while. The baud matches again as of
this change; `parse_line()` still needs updating to the `A,` row, and
`tools/fake_mega.py` with it. That repo's `protocol.py` names this README as
the authority for the wire format, so this section is it.

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

At the measured top speed of 0.275 m/s, 400 ms is ~110 mm of open-loop travel
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

The old `WHEEL_MAX_MPS = 0.50 m/s` was a guess, and measurement killed it.
Timed 1 m runs, 2026-10-12, single trials — `motor_cal/RESULTS.md`:

| wheel command | forward | left strafe |
|---|---|---|
| 15% | 0.138 m/s | 0.101 m/s |
| 20% | 0.189 m/s | 0.150 m/s |
| 25% | 0.225 m/s | 0.187 m/s |
| 30% | 0.254 m/s | 0.219 m/s |
| 35% | **0.275 m/s** | **0.227 m/s** |
| 100% | wheels do not turn correctly | wheels do not turn correctly |

**Max commandable forward speed is 0.275 m/s.** 0.5 m/s is not slow, not
uncalibrated, not a tuning problem: it is unreachable, and so is anything much
above a quarter of a metre per second. Any path planner built around half a
metre per second needs rewriting, not retuning.

An earlier sweep at 40% and 60% reached 0.303 and 0.327 m/s forward, so there
is a little more speed above the operating band. It is **not** in the map: at
the one command both sweeps shared, they disagreed by 12% for a strafe and 10%
for a drive, which is battery and floor rather than chassis, and splicing two
sessions together would put a step in the curve. Extending the range means one
fresh sweep out to 60% in a single sitting.

Three more things the table says:

- **The curve is still bending at 35%.** Per 5% step, forward gains 0.052,
  0.035, 0.030, then 0.021 m/s; a strafe's last step gains only 0.008. The
  earlier sweep found it flat above 60% — 0.002 m/s gained forward from 60% to
  80%, and a strafe *losing* 0.011. Extra deflection up there buys current
  draw, which is the other reason the map stops where it does.
- **100% does not work at all**, on either axis. `MOTOR_MAX_DEFLECTION = 0.80`
  in `config.h` makes it unreachable: saturation scales to that ceiling rather
  than to full deflection, so no command can ask for it.
- **It is not a straight line, and it misses the origin.** A line through the
  15% and 20% forward points hits 0.0 m/s at about 1.5% command, and the
  measured gains per step keep shrinking above that: the curve is convex at the
  bottom and concave at the top. Below 15% is unmeasured and is where the servo
  deadband lives. That is why the conversion is a table and not a constant.

## Strafing is slower, and that is the answer

A strafe runs 17–26% slower than a drive at the same wheel command, worst at
low speed:

| wheel command | forward / strafe |
|---|---|
| 15% | 1.36× |
| 20% | 1.27× |
| 25% | 1.20× |
| 30% | 1.16× |
| 35% | 1.21× |

Mecanum rollers slip sideways; a fifth to a quarter of the wheel speed goes
into roller rotation instead of chassis motion. **There is deliberately no
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

**Rotation is cheap to calibrate from the Pi**, which now owns the BNO085 and
publishes its yaw rate. Command a few pure rotations, read the steady-state
rate off the IMU topic, and build a third table the same way:

```
V,0,0,0.5      ... read yaw rate ...
V,0,0,1.0      ... read yaw rate ...
V,0,0,2.0      ... read yaw rate ...
```

Then give `rotationDeflection()` its own rows instead of borrowing the forward
curve. The same runs settle `DRIVE_WZ_SIGN`, which has never been checked
either — see "Bring-up".

# Bring-up

Do this **with the wheels off the ground** the first time. One axis at a time,
checking against `config.h`'s three sign trims:

| step | command | expect | if wrong |
|---|---|---|---|
| 1 | `V,0.25,0,0` | all four wheels drive forward | see "a quarter turn" below |
| 2 | `V,0,0.2,0` | bot strafes **left** | as above |
| 3 | `V,0,0,0.5` | bot rotates **anticlockwise**, and the Pi's heading **increases** | flip `DRIVE_WZ_SIGN` |
| 4 | stop sending | motors neutral within 400 ms — watch the wheels, not the telemetry | check the watchdog |
| 5 | `V,0.25,0,0` then `V,1.0,0,0` | the uplink reads `A,0.250,...` then `A,0.275,...` | the speed map is not being applied |

If a *single* wheel runs backwards rather than a whole axis, that is wiring or
a mirrored motor — fix `MOTOR_MIRROR_LEFT` / `MOTOR_MIRROR_RIGHT`, or the
mounting, not the axis trims.

**Step 3 has never been verified on this chassis.** Steps 1 and 2 are measured
(see below), but the IMU was unplugged when that was done, so nothing has yet
confirmed that `+Wz` turns the bot the way the Pi's heading says. It is the
only direct check that the drive and the IMU agree on which way is positive,
and now that the IMU lives on the Pi it has to be done across both: command
the rotation here, read the heading there. `DRIVE_WZ_SIGN` is the fix if they
disagree.

### If an axis is wrong, it may be a quarter turn, not a sign

`DRIVE_VX_SIGN` and `DRIVE_VY_SIGN` can only flip an axis, and on this chassis
the fault was not a flip. As shipped from the shield's pin order, `+Vx` made
the bot strafe **left** and `+Vy` drove it **backwards**: the body frame was
rotated 90°, which no pair of sign flips can express.

The fix was the pin-to-corner mapping in `config.h` — `PIN_LEFT_FRONT` and
friends are now ordered by *physical corner*, derived from the measured pulse
patterns, with the mixing and the mirrors untouched. If translation ever comes
out rotated again, check that mapping before reaching for a sign trim.

Checking the IMU's own axes is now the Pi's job — see `p4p_imu` in the
conv-chart repo. One thing worth carrying over, though: **the Servo library's
timer ISR used to disturb I2C timing once four motors were attached**, causing
spontaneous BNO085 resets. That was on this board's bus. The Pi's I2C is a
different bus with no servo ISR on it, so the problem should not follow the
chip across — but if headings start restarting mid-drive, this is the history.

---

# Layout

| file | role |
|---|---|
| `p4p_arduino.ino` | `setup()` / `loop()`, task scheduling, telemetry format |
| `config.h` | every tunable constant — pins, rates, limits, geometry, sign trims, speed-map tables |
| `timing.h` | `RateTimer`, non-blocking fixed-rate timer. One per periodic task instead of `delay()` |
| `drive.h` / `.cpp` | mecanum mixing, saturation, motor output. The only file that knows wheels exist |
| `speedmap.h` / `.cpp` | measured velocity ↔ wheel command, per axis. The only thing that makes a `V` mean m/s |
| `motor_cal/` | the calibration rig that produces those tables — a separate sketch, with its own README |
| `test/` | host-side tests. `./test/run_tests.sh`, no Arduino toolchain needed |
| `command.h` / `.cpp` | serial command parser. Knows nothing about motors |
| `system_setup.h` / `.cpp` | everything `setup()` has to do |

`loop()` runs three things: `cmd.poll()` every pass (non-blocking), a 50 Hz
control task, and a 50 Hz telemetry task. **No `delay()` in `loop()`** — add
periodic work as another `RateTimer`.

## Dependencies

```bash
arduino-cli lib install "Servo"
```

That is the only one. `Adafruit BNO08x` is no longer needed here — the IMU
moved to the Pi.

## Build and flash

```bash
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
```

## Talking to it by hand

```bash
stty -F /dev/ttyACM0 115200 raw -echo -hupcl clocal
cat /dev/ttyACM0 &            # watch the A, rows
printf 'E\n'            > /dev/ttyACM0
printf 'V,0.25,0,0\n'   > /dev/ttyACM0      # 0.1 would snap up to 0.138
printf 'S\n'            > /dev/ttyACM0
```

The `-hupcl` matters: without it the Mega resets when a reader exits.

Only one process can hold `/dev/ttyACM0` at a time. If a flash fails as busy,
`fuser -v /dev/ttyACM0` names the culprit — usually an open Serial Monitor.

## Conventions

- No `delay()` in `loop()`. Add periodic work as another `RateTimer`.
- Constants go in `config.h`, not inline.
- New peripherals are brought up in `systemSetup()`.
- Mixing stays textbook; hardware quirks live in the pin, mirror and sign
  constants so they stay visible.
- Measured numbers live in the speed-map tables with the date and method
  recorded, not inlined as a fudge factor.
- The uplink mirrors the downlink's format. If one changes, change both, and
  update `protocol.py` on the Pi.
