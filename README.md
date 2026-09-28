# p4p_arduino

Low-level firmware for the P4P bot, running on an **Arduino Mega 2560**.

The Mega owns the hard-real-time work — sensors, motor drive, watchdog — and
talks to a **Raspberry Pi** over USB serial. High-level planning and the
docking filter live on the Pi; this firmware is deliberately dumb and fast.

## Current state

IMU streaming only. Motor drive, the `CMD_VEL` parser and the watchdog are
next — see `loop()` in `p4p_arduino.ino` for where they slot in.

## Hardware

| | |
|---|---|
| MCU | Arduino Mega 2560 (`arduino:avr:mega:cpu=atmega2560`) |
| IMU | BNO085 over I2C, addr `0x4A` — SDA = 20, SCL = 21 |
| Link | USB serial to the Pi, 115200 baud |

## Serial protocol

Lines beginning with `#` are human-readable status and should be ignored by
the Pi-side parser. Everything else is CSV.

Uplink (Mega to Pi), at `STREAM_RATE_HZ` (50 Hz):

```
t_ms,heading_rad,yaw_rate_dps,resets
```

| field | meaning |
|---|---|
| `t_ms` | `millis()` at the last IMU sample |
| `heading_rad` | continuous, unwrapped yaw. Never wraps at ±π — the Pi filter uses differences of it |
| `yaw_rate_dps` | calibrated gyro Z, degrees/s |
| `resets` | count of spontaneous BNO085 resets. Should stay 0 |

Heading comes from the game rotation vector, so it uses no magnetometer: it is
*relative* and drifts slowly. Measured drift is well under 0.1 °/min.

Sign convention: heading increases **anticlockwise** viewed from above.
Flip `IMU_YAW_SIGN` in `config.h` if the chip is mounted upside down.

## Layout

| file | role |
|---|---|
| `p4p_arduino.ino` | `setup()` / `loop()`, stream task |
| `config.h` | every tunable constant — pins, rates, I2C address |
| `timing.h` | `RateTimer`, non-blocking fixed-rate timer. Use one per periodic task instead of `delay()` |
| `imu.h` / `.cpp` | BNO085 wrapper, unwrapped heading |
| `system_setup.h` / `.cpp` | everything `setup()` has to do |

## Dependencies

`Adafruit BNO08x` and its dependency `Adafruit BusIO`.

```bash
arduino-cli lib install "Adafruit BNO08x"
```

## Build and flash

```bash
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
```

Watch the stream (`-hupcl` stops the Mega resetting when you quit):

```bash
stty -F /dev/ttyACM0 115200 raw -echo -hupcl clocal
cat /dev/ttyACM0
```

Only one process can hold `/dev/ttyACM0` at a time. If a flash fails as busy,
`fuser -v /dev/ttyACM0` names the culprit — usually an open Serial Monitor.

## Conventions

- No `delay()` in `loop()`. Add periodic work as another `RateTimer`.
- Constants go in `config.h`, not inline.
- New peripherals are brought up in `systemSetup()`.
