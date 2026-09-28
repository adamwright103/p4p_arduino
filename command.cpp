// command.cpp
#include "command.h"
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

void Command::poll() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\n') {
      if (overflow_) {          // tail of a too-long line: already counted
        overflow_ = false;
      } else {
        buf_[len_] = '\0';
        handleLine(buf_);
      }
      len_ = 0;
      continue;
    }
    if (c == '\r') continue;

    if (len_ >= CMD_BUF_LEN - 1) {   // no room for this byte plus a null
      if (!overflow_) { overflow_ = true; bad_++; }
      continue;
    }
    buf_[len_++] = c;
  }
}

char* Command::trim(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  char* end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t')) end--;
  *end = '\0';
  return s;
}

void Command::handleLine(char* s) {
  s = trim(s);
  if (*s == '\0') return;          // blank line, not an error
  if (*s == '#') return;           // comment, so a log can be replayed at us

  const char tag = toupper(*s);
  char* rest = trim(s + 1);

  if (tag == 'E') {
    if (*rest != '\0') { bad_++; return; }
    // Arming must never start motion. Drop any stored velocity and restart
    // the watchdog clock so the first telemetry line after E reads clean.
    vx_ = vy_ = wz_ = 0.0f;
    last_cmd_ms_ = millis();
    armed_ = true;
    return;
  }

  if (tag == 'S') {
    if (*rest != '\0') { bad_++; return; }
    vx_ = vy_ = wz_ = 0.0f;
    armed_ = false;
    return;
  }

  if (tag == 'V') {
    if (*rest != ',') { bad_++; return; }
    rest++;

    // strtod takes any number of decimal places, so 1 dp and 6 dp both parse
    // natively. It also rejects junk by leaving endptr where it started.
    // On AVR double is 32-bit, so this is a float parse -- no cost.
    float v[3];
    char* p = rest;
    for (uint8_t i = 0; i < 3; i++) {
      char* endp = p;
      v[i] = (float)strtod(p, &endp);
      if (endp == p) { bad_++; return; }     // nothing numeric consumed
      p = trim(endp);
      if (i < 2) {
        if (*p != ',') { bad_++; return; }   // missing separator
        p++;
      }
    }
    if (*p != '\0') { bad_++; return; }      // trailing junk, e.g. a 4th field

    // NaN and inf would propagate straight through the mixing into the PWM
    // clamp as an arbitrary value, so reject them here.
    for (uint8_t i = 0; i < 3; i++) {
      if (isnan(v[i]) || isinf(v[i])) { bad_++; return; }
    }

    vx_ = v[0];
    vy_ = v[1];
    wz_ = v[2];
    last_cmd_ms_ = millis();
    return;
  }

  bad_++;   // unknown tag
}
