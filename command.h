// command.h
// Non-blocking line parser for the Pi link. Knows nothing about motors.
//
// Accepted commands, one per line, terminated by \n (a trailing \r is fine):
//
//     E                      enable  -- arm the motors, velocity reset to zero
//     S                      stop    -- disarm and zero. The software e-stop
//     V,<vx>,<vy>,<wz>       set body velocity, SI units
//
// The bot boots DISARMED: V is parsed and stored but not acted on until an E
// arrives. E deliberately zeroes the stored velocity, so arming can never make
// the bot lurch off on a stale command -- a fresh V is always required.
//
// Malformed lines are discarded and counted; they never change the stored
// command. The command tag lets the parser resynchronise after a garbled line
// instead of acting on half of one.
#pragma once
#include <Arduino.h>
#include "config.h"

class Command {
 public:
  // Drains the RX buffer. Call every pass; never blocks.
  void poll();

  bool armed() const { return armed_; }

  // Milliseconds since the last accepted V. The caller compares this against
  // CMD_TIMEOUT_MS -- expiry is the control task's business, not the parser's,
  // so a flood of commands can never starve the safety check.
  uint32_t ageMs() const { return millis() - last_cmd_ms_; }

  float vx() const { return vx_; }
  float vy() const { return vy_; }
  float wz() const { return wz_; }

  uint16_t badCount() const { return bad_; }

 private:
  void handleLine(char* s);
  static char* trim(char* s);

  char buf_[CMD_BUF_LEN];
  uint8_t len_ = 0;
  bool overflow_ = false;   // line too long: drop it and everything to the \n

  bool armed_ = false;
  float vx_ = 0.0f;
  float vy_ = 0.0f;
  float wz_ = 0.0f;
  uint32_t last_cmd_ms_ = 0;
  uint16_t bad_ = 0;
};
