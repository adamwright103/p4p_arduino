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

## Round 2 — 15-35% at 5% steps

The operating range is now <=40%, so this maps the band properly rather than
extending upwards. `./run.sh list` prints it as a checklist.

Three trials per setting. Round 1 was single runs, which is why its left
60-vs-80 inversion cannot be told from timing noise.

| command % | axis | t1 (s) | t2 (s) | t3 (s) | mean (s) | speed (m/s) | notes |
|-----------|------|--------|--------|--------|----------|-------------|-------|
| 15 | F | | | | | | |
| 20 | F | | | | | | round 1: 4.78 s |
| 25 | F | | | | | | |
| 30 | F | | | | | | |
| 35 | F | | | | | | |
| 15 | L | | | | | | |
| 20 | L | | | | | | round 1: 7.60 s |
| 25 | L | | | | | | |
| 30 | L | | | | | | |
| 35 | L | | | | | | |

Two things to watch for, both of which are findings rather than failures:

- **15% may not move the bot**, or may move it only after a shove. That is the
  deadband edge, and it sets the slowest speed the drive can be asked for.
- **The forward runs are now genuinely forward.** Round 1's forward column was
  measured driving *backwards* under the old pin mapping, so if 20% F does not
  come in near 4.78 s, forward and reverse are not symmetric and round 1's
  forward numbers need discarding rather than merging.

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
