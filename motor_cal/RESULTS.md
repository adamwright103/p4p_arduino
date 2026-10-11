# Motor command -> speed, measured

**Distance timed:** 1.00 m. Speeds are `1.00 / t`.

## Round 1 — 2026-10-12

One trial per setting, hand-timed. Labelled in **chassis** directions, which
are not the labels the sketch used when these were taken — see "Axes" below.

### Forward (chassis +x)

| command % | t (s) | speed (m/s) | notes |
|-----------|-------|-------------|-------|
| 20 | 4.78 | 0.209 | |
| 40 | 3.30 | 0.303 | |
| 60 | 3.06 | 0.327 | |
| 80 | 3.04 | 0.329 | flat vs 60% — 0.7% apart, inside hand-timing error |
| 100 | — | — | wheels did not spin correctly; now blocked by `MOTOR_MAX_DEFLECTION` |

### Left strafe (chassis +y)

| command % | t (s) | speed (m/s) | notes |
|-----------|-------|-------------|-------|
| 20 | 7.60 | 0.132 | |
| 40 | 4.20 | 0.238 | |
| 60 | 3.63 | 0.275 | |
| 80 | 3.79 | 0.264 | **slower than 60%** — past the knee, or noise |
| 100 | — | — | wheels did not spin correctly; now blocked by `MOTOR_MAX_DEFLECTION` |

### Axes: these runs were taken before the pin mapping was fixed

The sketch's `F` and `L` at the time were the firmware's `+Vx` and `+Vy`, and
both turned out to be 90 degrees off the chassis. The fix landed in `config.h`
on 2026-10-12 as a pin-to-corner permutation, so `+Vx` is now really forward
and the rig drives the firmware's own axes:

| round-1 label | pin pulses (46,47,50,51) | bot actually went | now called |
|---|---|---|---|
| `F` (old +Vx) | `(-,-,+,+)` | left (+y) | **`L`** |
| `L` (old +Vy) | `(+,-,-,+)` | backwards (−x) | **`F`**, negated |

So the **left** column above is measured in the direction it is now labelled.
The **forward** column was measured driving *backwards* at the same deflection
magnitude; `F` now drives the opposite way. Servo response is symmetric about
neutral, so forward should match backward — but it is an assumption, not a
measurement. **The 20% `F` run in round 2 confirms or kills it:** expect ~4.78 s.

### What the numbers say

- **Max forward is ~0.33 m/s**, so `WHEEL_MAX_MPS = 0.50` is not merely
  uncalibrated, it is unreachable. 0.5 m/s cannot be mapped to any command.
- **The curve is flat above 60%.** Forward gains 0.002 m/s between 60% and 80%;
  left loses 0.011. Deflection above ~60% buys current draw, not speed.
- **Left runs 21–37% slower than forward** at matched command, worst at low
  speed. That is the `STRAFE_GAIN` ratio, and it is not a constant: 1.59 at
  20%, 1.27 at 40%, 1.19 at 60%. A single gain cannot fit it.
- **Strongly non-linear at the bottom.** A straight line through the 20% and
  40% forward points extrapolates to 0.115 m/s at zero command, which is
  impossible — the real curve must fall off a cliff somewhere below 20%. That
  cliff is the servo deadband, and it is entirely unmeasured.

## Round 2 — 15-35% at 5% steps, 2026-10-12

One trial per setting, hand-timed. **These are the rows in the shipped map**
(`SPEED_MAP_FORWARD` / `SPEED_MAP_STRAFE` in `config.h`).

### Forward (+Vx)

| command % | t (s) | speed (m/s) | gain over previous step |
|-----------|-------|-------------|-------------------------|
| 15 | 7.26 | 0.138 | |
| 20 | 5.28 | 0.189 | +0.052 |
| 25 | 4.45 | 0.225 | +0.035 |
| 30 | 3.93 | 0.254 | +0.030 |
| 35 | 3.63 | 0.275 | +0.021 |

### Left strafe (+Vy)

| command % | t (s) | speed (m/s) | gain over previous step |
|-----------|-------|-------------|-------------------------|
| 15 | 9.87 | 0.101 | |
| 20 | 6.68 | 0.150 | +0.048 |
| 25 | 5.35 | 0.187 | +0.037 |
| 30 | 4.57 | 0.219 | +0.032 |
| 35 | 4.40 | 0.227 | +0.008 |

### Why round 1 is not merged in

The two sessions disagree at the command they share:

| 20% command | round 1 | round 2 | |
|---|---|---|---|
| forward | 4.78 s | 5.28 s | 10% slower |
| left | 7.60 s | 6.68 s | 12% faster |

The left runs are the decisive ones: the pulse pattern for a left strafe is
byte-identical across both rounds, so nothing about the chassis changed and a
12% swing is battery, floor or timing technique. Round 1's forward runs were
also driven *backwards*, under the old pin mapping, so their 10% could be
either that or the same session drift — and there is no way to tell them apart
now.

Either way, splicing round 1's 40% and 60% rows onto round 2's curve would put
a step in it that is measurement, not chassis. The map is round 2 only, and it
therefore tops out at 35%: **0.275 m/s forward, 0.227 m/s sideways.**

### What these numbers say

- **The curve is still bending at the top of the band.** Forward gains shrink
  0.052 → 0.021 m/s per 5% step. It has not plateaued by 35%, but it is well
  on the way.
- **A strafe is 17-26% slower**, worst at the bottom: 1.36x at 15%, 1.16x at
  30%. No single gain fits that, which is why there is no `STRAFE_GAIN`.
- **The strafe knee is at ~30%.** The last step buys 0.008 m/s, a third of
  what forward gets, and barely more than hand-timing noise.
- **Nothing below 15% is measured.** A line through the 15% and 20% forward
  points reaches zero at ~1.5% command, which cannot be right — the deadband
  is somewhere in that gap. The map clamps a slower request *up* to 0.138 m/s
  rather than guessing, so treat that as a hard floor on approach speed.

### Still open

- **Repeats.** Every row is a single run. With ~±0.2 s of hand-timing error,
  that is ~5% on the faster runs, and the strafe's 30→35% step is inside it.
- **Below 15%**, where the deadband edge is.
- **Rotation**, which has never been measured at all. The Pi owns the IMU now
  and publishes yaw rate, so this is the cheap one: command a few pure `Wz`
  values, read the rate, build a third table. It also settles
  `DRIVE_WZ_SIGN`, which has never been checked.
- **40-60% in one sitting**, if the extra 0.05 m/s above the band is ever
  wanted.

## Protocol

1. Mark a **start line** and a **finish line 1 m apart**. Mark a third line
   **~0.3 m before the start line** and put the bot's front edge there.
2. Bot on the floor at that third line, powered, battery charged.
3. `./run.sh <pct> F` — the flash resets the board, so it drives ~1 s later.
4. Time from the bot crossing the **start** line to crossing the **finish**
   line. Not from when the wheels start turning: the run-up keeps the
   acceleration ramp out of the timed metre.
5. Tap **RESET** for trials 2 and 3 at the same setting — no re-flash.
6. Note the battery voltage every few runs. Open-loop speed sags as it drops,
   and if it sags across the sweep the map is measuring the battery, not the
   motors.

Hand timing costs ~±0.2 s of reaction error, which is 6% of a 3.3 s run — so
differences under ~0.2 s between settings are not real. Three trials per
setting is what makes the flat top of the curve distinguishable from noise.

### Things worth writing in `notes`

- **Did not move / stalled.** Expected below 20%. A continuous-rotation servo
  has a deadband around neutral, and that is exactly the non-linearity being
  hunted. "0 m/s at 10%" is a real data point — record it.
- **Veered off the line.** A forward run that curves means the four wheels do
  not match at that command. It also corrupts the time, since the bot
  travelled further than a metre.
- **Crabbed / rotated** on a strafe run. Normal for mecanum, but note which way.

## Battery

| run | pack voltage |
|-----|--------------|
|     |              |
