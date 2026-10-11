#!/usr/bin/env bash
# run.sh <pct> <F|L> [-n]
#
# F and L are CHASSIS directions: F drives forward, L strafes left. They are
# not the firmware's Vx/Vy, which sit 90 degrees out -- motor_cal.ino maps
# them. Max is 80%: full deflection does not turn the wheels.
#
# Patches SPEED_PCT and AXIS in motor_cal.ino, compiles, and flashes. The
# upload itself resets the Mega, so the run starts ~1 s after the flash
# finishes -- have the bot on the floor at the start line BEFORE you run this.
#
#   ./run.sh 15 F        15% forward
#   ./run.sh 35 L        35% left strafe
#   ./run.sh 35 L -n     patch and compile only, no flash
#   ./run.sh list        print the round-2 sweep as a checklist
#
# Repeat a trial without re-flashing: tap RESET on the Mega.
set -euo pipefail

FQBN=arduino:avr:mega:cpu=atmega2560
PORT=${PORT:-/dev/ttyACM0}
HERE=$(cd "$(dirname "$0")" && pwd)

# Round 2: 5% steps across the operating band, both axes, 3 trials each.
SWEEP="15 20 25 30 35"

if [[ ${1:-} == list ]]; then
  echo "round-2 sweep -- 3 trials each, tap RESET between trials:"
  for a in F L; do
    [[ $a == F ]] && name=forward || name=left
    for p in $SWEEP; do printf '  ./run.sh %-3s %s   # %s %s%%\n' "$p" "$a" "$name" "$p"; done
  done
  exit 0
fi

pct=${1:-}
axis=${2:-}
usage="usage: $0 <1-80> <F|L> [-n]   # F = forward, L = left strafe"
# 80 is MOTOR_MAX_DEFLECTION; the sketch also static_asserts it, so a bad value
# fails the build rather than quietly driving slower than it reports.
[[ $pct =~ ^[0-9]+$ ]] && (( pct >= 1 && pct <= 80 )) || { echo "$usage" >&2; exit 1; }
[[ $axis == F || $axis == L ]] || { echo "$usage" >&2; exit 1; }

if ! grep -qE "(^| )${pct}( |$)" <<<"$SWEEP"; then
  echo "note: ${pct}% is not part of the round-2 sweep (${SWEEP// /, }) -- continuing"
fi

# Rough expected time for 1 m, from the round-1 20/40% fit. Only a sanity
# check that RUN_MS is long enough -- it is an extrapolation, not a prediction.
awk -v p="$pct" -v a="$axis" 'BEGIN {
  if (a == "F") { m = 0.004690; c = 0.11545 } else { m = 0.005308; c = 0.02578 }
  v = m * p + c
  if (v <= 0.02) print "expect: little or no motion at this command"
  else printf "expect: roughly %.1f s for 1 m (~%.2f m/s), run limit is 15 s\n", 1/v, v
}'

sed -i -E "s/^constexpr uint8_t SPEED_PCT = [0-9]+;/constexpr uint8_t SPEED_PCT = ${pct};/" "$HERE/motor_cal.ino"
sed -i -E "s/^constexpr char AXIS = '[FL]';/constexpr char AXIS = '${axis}';/"             "$HERE/motor_cal.ino"
grep -E "^constexpr (uint8_t SPEED_PCT|char AXIS)" "$HERE/motor_cal.ino"

arduino-cli compile --fqbn "$FQBN" "$HERE"
[[ ${3:-} == -n ]] && { echo "-n: not flashing"; exit 0; }

# Fails with the port busy if a serial monitor is open; fuser names the holder.
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$HERE"
echo "flashed -- driving ${pct}% $([[ $axis == F ]] && echo forward || echo left) in ~1 s"
