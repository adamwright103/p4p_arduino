/*
  motor_cal.ino -- open-loop speed calibration rig

  Drives the chassis in a straight line at one fixed motor command so the
  speed can be timed over a measured distance by hand. Set SPEED_PCT and AXIS
  below, flash, put the bot on the floor, and it drives after a 1 s pause.

  This is a separate sketch, not a mode of the firmware, because the firmware
  cannot do this job: it only moves while a Pi keeps feeding it V commands
  inside the 400 ms watchdog, and it routes every velocity through the speed
  map -- the map these runs produce. The rig needs the opposite: no host, and
  one fixed raw wheel command.

  The firmware sources in this folder are symlinks, so the mixing, the pulse
  widths, the pin mapping and the sign trims being calibrated are byte-for-byte
  the ones that ship. Do not copy them.

  Repeating a run needs no re-flash -- tap RESET on the Mega, or re-open the
  serial port (DTR resets the board). Either way you get the same 1 s pause
  and another identical run.

  Build and flash (from this folder):
      arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
      arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
*/
#include <Arduino.h>
#include "config.h"
#include "drive.h"

// ---------------------------------------------------------------------------
// Set these two, flash, run.
// ---------------------------------------------------------------------------
// Motor command as a percentage of full pulse deflection. Capped at
// MOTOR_MAX_DEFLECTION (80%) -- the motors do not turn reliably at 100%, and
// the measured curve is flat above ~60% anyway, so there is nothing up there
// to calibrate. See RESULTS.md.
constexpr uint8_t SPEED_PCT = 35;

// 'F' = forward (+Vx), 'L' = left (+Vy). Both are real chassis directions:
// the pin-to-corner mapping in config.h was corrected on 2026-10-12, so the
// firmware's own axes now point where they say they do.
constexpr char AXIS = 'L';

// ---------------------------------------------------------------------------
// Run shape -- the defaults are fine for a 1 m timed section.
// ---------------------------------------------------------------------------
// Pause between power-up/reset and the wheels turning. Long enough to let go
// of the bot and pick up the stopwatch.
constexpr uint32_t START_DELAY_MS = 1000;

// Hard stop after this long, so a missed catch doesn't become a wall. The
// slowest measured run so far is 7.6 s for a metre (20% left), hence the
// generous default. Shorten it for the fast runs if the floor is tight.
constexpr uint32_t RUN_MS = 15000;

static_assert(SPEED_PCT > 0, "SPEED_PCT must be at least 1");
static_assert(SPEED_PCT <= (uint8_t)(MOTOR_MAX_DEFLECTION * 100.0f + 0.5f),
              "SPEED_PCT above MOTOR_MAX_DEFLECTION: Drive would silently "
              "scale it down and the run would not measure what it says");
static_assert(AXIS == 'F' || AXIS == 'L', "AXIS must be 'F' or 'L'");

static Drive drive;
static uint32_t start_ms = 0;
static bool running = false;

// ---------------------------------------------------------------------------
// The command
// ---------------------------------------------------------------------------
// setBodyDeflection() takes the wheel command directly, as a fraction of full
// pulse deflection, so a run is exactly SPEED_PCT% by construction. It does
// not go through the speed map, which is deliberate: the map is built from
// these measurements, and a rig that read it would be measuring its own
// output.
//
// Every wheel ends up at the same deflection magnitude on either axis, so the
// two sweeps stay directly comparable and neither saturates:
//
//   F (forward, +Vx)  wheels [+f +f +f +f]   (body sense, pre-mirror)
//   L (left,    +Vy)  wheels [-f +f +f -f]
static void commandAxis() {
  const float frac = SPEED_PCT / 100.0f;
  if (AXIS == 'F') {
    drive.setBodyDeflection(frac, 0.0f, 0.0f);
  } else {
    drive.setBodyDeflection(0.0f, frac, 0.0f);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Motors to neutral before anything else, so the pins are never left
  // floating while the banner prints.
  drive.begin();

  const uint16_t dev_us =
      (uint16_t)((SPEED_PCT / 100.0f) * (MOTOR_US_MAX - MOTOR_US_NEUTRAL));

  Serial.println(F("# motor_cal"));
  Serial.print(F("# axis      "));
  Serial.print(AXIS == 'F' ? F("FORWARD") : F("LEFT"));
  Serial.println(AXIS == 'F' ? F("  (+Vx)") : F("  (+Vy)"));
  Serial.print(F("# command   "));
  Serial.print(SPEED_PCT);
  Serial.println(F("% of full deflection"));
  Serial.print(F("# pulse     "));
  Serial.print(MOTOR_US_NEUTRAL);
  Serial.print(F(" +/- "));
  Serial.print(dev_us);
  Serial.println(F(" us"));
  Serial.print(F("# run limit "));
  Serial.print(RUN_MS);
  Serial.println(F(" ms"));
  Serial.println(F("# send any character to stop early"));
  Serial.println(F("# starting in 1 s -- LED on = driving"));

  // Blink through the pause: visible countdown, and it proves the sketch is
  // alive rather than hung before the wheels ever move.
  const uint32_t t0 = millis();
  while (millis() - t0 < START_DELAY_MS) {
    digitalWrite(LED_BUILTIN, ((millis() - t0) / 100) & 1);
  }

  digitalWrite(LED_BUILTIN, HIGH);
  commandAxis();
  start_ms = millis();
  running = true;

  // A clamp here would mean the wheels are not at SPEED_PCT and the run is
  // measuring something other than what it claims. The static_assert above
  // should make this unreachable; say so out loud if it ever is not.
  if (drive.saturated()) {
    Serial.println(F("# !! SCALED DOWN to MOTOR_MAX_DEFLECTION -- discard"));
  }
  Serial.println(F("# DRIVING"));
}

void loop() {
  if (!running) return;

  // Either kill switch: a byte on the link, or the run limit.
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    halt(F("# STOPPED by serial"));
    return;
  }
  if (millis() - start_ms >= RUN_MS) {
    halt(F("# STOPPED on run limit"));
  }
}

static void halt(const __FlashStringHelper* why) {
  drive.stop();
  digitalWrite(LED_BUILTIN, LOW);
  running = false;
  Serial.println(why);
  Serial.print(F("# drove for "));
  Serial.print(millis() - start_ms);
  Serial.println(F(" ms"));
  Serial.println(F("# tap RESET to run again at the same setting"));
}
