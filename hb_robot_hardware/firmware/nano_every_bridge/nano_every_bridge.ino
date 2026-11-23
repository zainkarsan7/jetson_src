#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#if !defined(__AVR_ATmega4809__)
#error "Select Arduino Nano Every"
#endif

/*
 * NANO EVERY PINOUT
 *
 * A4 SDA / A5 SCL -> 5V side of I2C level shifter.
 * 3.3V side -> SDA/SCL of BOTH MPU6050 boards.
 * Common signal GND.
 *
 * MPU1 AD0 = GND  : address 0x68
 * MPU2 AD0 = 3.3V : address 0x69
 *
 * Power breakouts according to their rating.
 * XDA, XCL and INT are unused.
 *
 * D4 STEP, D5 DIR -> suitable 5V buffers -> DM542T PUL+/DIR+.
 * PUL-/DIR- -> signal GND for common-cathode wiring.
 * Driver enabled continuously; separate motor supply, e.g. 24V.
 *
 * D7: fully retracted HOME limit switch.
 * Active LOW.

 * Positive = RETRACT toward HOME.
 * Negative = ADVANCE toward workpiece.
 * PREP_MOVING and SEARCH both use negative motion.
 * HOME is the positive terminal limit.
 
 * D13 -> flap servo signal.
 * External servo 5V supply; common signal GND.
 *
 * CALIBRATED CONVENTION:
 *   Positive steps RETRACT toward HOME.
 *   Negative steps ADVANCE toward the workpiece.
 *
 * TCB1: STEP generation.
 * Standard Servo library: TCB2.
 * Standard Arduino timekeeping: TCB3.
 *
 * No direct plasma voltage connection.
 * Torch feedback arrives over USB.
 *
 * Calibration requires the torch to be physically inhibited.
 */

const uint8_t STEP = 4;
const uint8_t DIR = 5;
const uint8_t HOME_PIN = 7;
const uint8_t SERVO_PIN = 13;

// STEP timing at 10kHz; motion supervision at 1kHz.
// STEP HIGH lasts 100us.
const uint16_t HZ = 10000;

enum State : uint8_t {
  UNKNOWN, HOME, PREP_MOVING, READY, SEARCH, ESTABLISH,
  HOLD, TRACK, HOMING, JOG, PAUSED, FAULT
};

const char *names[] = {
  "UNKNOWN", "HOME", "PREP_MOVING", "READY", "SEARCH", "ESTABLISH",
  "HOLD", "TRACK", "HOMING", "JOG", "PAUSED", "FAULT"
};

enum Fault : uint8_t {
  NONE,
  STOPPED,
  LINK_LOST,
  TORCH_INVALID,
  SEARCH_END,
  HOME_MISSED,
  SWITCH_STUCK,
  HOME_LOST,
  BAD_CONFIG,
  ARC_UNSTABLE,
  TRACK_LIMIT
};

// CONFIG v2 fields: all integers.
enum {
  CAL,
  RET,
  DIVE,
  HOME_MAX,
  RET_RATE,
  HOME_RATE,
  SEARCH_RATE,
  SEARCH_MS,
  POS_HIGH,
  HOME_DIR,
  EST_MS,
  TRACK_RATE,
  TRACK_OFFSET,
  LINK_MS,
  JOG_MAX,
  JOG_RATE,
  JOG_WINDOW,
  NCFG
};

long cfg[NCFG] = {0};

bool configured = false;
uint32_t session = 0;
uint32_t lastCommand = 0;
bool lastOk = false;
const char *lastReply = "BOOT";

Servo flap;

volatile State state = UNKNOWN;
volatile uint8_t fault = NONE;
volatile uint8_t permit = 3;

volatile bool qualified = false;
volatile bool switchOn = false;
volatile bool pulse = false;
volatile bool referenced = false;

volatile int8_t correction = 0;
volatile int8_t direction = 1;

volatile int32_t position = 0;
volatile int32_t trackLow = 0;
volatile int32_t trackHigh = 0;

volatile uint32_t ms = 0;
volatile uint32_t permitAt = 0;
volatile uint32_t began = 0;
volatile uint32_t budget = 0;
volatile uint32_t establishAt = 0;
volatile uint32_t remaining = 0;

volatile uint32_t activeId = 0;
volatile uint32_t doneId = 0;

// 0 = none, 1 = completed, 2 = aborted.
volatile uint8_t doneResult = 0;

volatile uint16_t rate = 0;
volatile uint16_t targetRate = 0;
volatile uint16_t phase = 0;

volatile bool trackingAuthorized = false;
volatile bool homeReleaseObserved = false;

uint32_t rxErrors = 0;
uint32_t imuDrops = 0;
uint32_t i2cErrors = 0;

/*
 * Permission modes:
 * 0 = fresh confirmed torch off
 * 1 = torch on, transfer absent
 * 2 = torch on, transfer detected
 * 3 = stale/invalid/faulted feedback
 * 4 = shutdown commanded, output/transfer not yet cleared
 * 5 = ignition commanded, waiting for output
 */

bool moving(State s) {
  return s == PREP_MOVING ||
         s == SEARCH ||
         s == TRACK ||
         s == HOMING ||
         s == JOG;
}

// Call from ISR or with interrupts disabled.
void finish(uint8_t result) {
  rate = 0;
  targetRate = 0;

  if (activeId) {
    doneId = activeId;
    doneResult = result;
    activeId = 0;
  }
}

// Stopping never initiates another motion.
void fail(uint8_t reason, bool pause = false) {
  finish(2);
  correction = 0;
  qualified = false;
  trackingAuthorized = false;
  fault = reason;
  state = pause ? PAUSED : FAULT;
}

// Caller holds ATOMIC_BLOCK.
// First STEP follows direction setup by at least 100us.
void startMove(
    State s,
    int8_t dir,
    uint32_t steps,
    uint16_t speed,
    uint32_t timeout,
    uint32_t id) {

  digitalWrite(
      DIR,
      (dir > 0) == bool(cfg[POS_HIGH]) ? HIGH : LOW);

  direction = dir;
  remaining = steps;
  targetRate = speed;
  rate = 0;
  phase = 0;
  began = ms;
  budget = timeout;
  activeId = id;
  fault = NONE;
  state = s;
}

void superviseAxis() {
  static uint8_t onCount = 0;
  static uint8_t offCount = 0;

  const bool raw = digitalRead(HOME_PIN) == LOW;

  // Debounce both activation and release over 5ms.
  if (raw) {
    offCount = 0;
    if (onCount < 5) ++onCount;
  } else {
    onCount = 0;
    if (offCount < 5) ++offCount;
  }

  if (onCount == 5) switchOn = true;
  if (offCount == 5) switchOn = false;

  if (state == HOME && !switchOn) {
    referenced = false;
    fail(HOME_LOST);
    return;
  }

  if (!configured) return;
  // During negative PREP the switch must release.
  // The release is required before the next positive HOME seek.
  if (state == PREP_MOVING && offCount == 5) {
    homeReleaseObserved = true;
  }

  const bool fresh =
      (uint32_t)(ms - permitAt) <= (uint32_t)cfg[LINK_MS];

  // Calibration movements have local step/time bounds.
  // They do not require a continuously scheduled host heartbeat.
  const bool local =
      cfg[CAL] && (state == JOG || state == HOMING);

  if (moving(state) ||
      state == READY ||
      state == ESTABLISH ||
      state == HOLD) {

    if (!local && !fresh) {
      fail(LINK_LOST, true);
      return;
    }

    if (!cfg[CAL] && permit == 3) {
      fail(TORCH_INVALID, true);
      return;
    }
  }

  if ((state == PREP_MOVING || state == HOMING) &&
      !cfg[CAL] && permit != 0) {
    fail(TORCH_INVALID);
    return;
  }

  if (state == READY) {
    // Transfer may arrive before the SEARCH service.
    if (permit == 2) {
      state = ESTABLISH;
      establishAt = ms;
    } else if (permit == 4 || ms - began > 10000UL) {
      fail(TORCH_INVALID);
    }
    return;
  }

  if (state == SEARCH) {
    // First transfer indication stops descent immediately.
    if (permit == 2) {
      rate = 0;
      state = ESTABLISH;
      establishAt = ms;
      return;
    }

    if (permit != 1) {
      fail(TORCH_INVALID);
      return;
    }
  }

  if (state == ESTABLISH) {
    rate = 0;

    if (permit == 2 && qualified) {
      // trackLow = max(
      //     -(int32_t)cfg[DIVE],
      //     position - (int32_t)cfg[TRACK_OFFSET]);

      // trackHigh = min(
      //     (int32_t)cfg[RET],
      //     position + (int32_t)cfg[TRACK_OFFSET]);
      const int32_t mechanicalLow =
          -((int32_t)cfg[RET] + (int32_t)cfg[DIVE]);

      trackLow = max(
          mechanicalLow,
          position - (int32_t)cfg[TRACK_OFFSET]);

      trackHigh = min(
          (int32_t)0,
          position + (int32_t)cfg[TRACK_OFFSET]);

      finish(1);
      state = HOLD;
    } else if (
        (permit != 1 && permit != 2) ||
        ms - establishAt >= (uint32_t)cfg[EST_MS]) {
      fail(ARC_UNSTABLE);
    }

    return;
  }

  if (state == HOLD || state == TRACK) {
    // Normal torch shutdown stops corrections.
    if (permit == 0 || permit == 4) {
      rate = 0;
      state = HOLD;
      return;
    }

    if (permit != 2 || !qualified) {
      fail(ARC_UNSTABLE);
      return;
    }

    if (!trackingAuthorized ||
    !cfg[TRACK_RATE] ||
    !correction) {
      rate = 0;
      state = HOLD;
      return;
    }

    if ((correction > 0 && position >= trackHigh) ||
        (correction < 0 && position <= trackLow)) {
      fail(TRACK_LIMIT);
      return;
    }

    if (state != TRACK || direction != correction) {
      startMove(
          TRACK, correction, 0, cfg[TRACK_RATE], 0, 0);
      return;
    }
  }

  if (!moving(state)) {
    rate = 0;
    return;
  }

  if (state != TRACK && ms - began > budget) {
    fail(state == HOMING ? HOME_MISSED : SEARCH_END);
    return;
  }

  if (state == HOMING && raw) {
    // Stop at first contact, then qualify stationary contact.
    rate = 0;

    if (onCount == 5) {
    position = 0;
    referenced = true;
    homeReleaseObserved = false;
    trackingAuthorized = false;
    correction = 0;

    finish(1);
    state = HOME;
  }

    return;
  }

  // Acceleration calculation at 1kHz rather than every pulse tick.
  const uint32_t age = ms - began;
  rate = age < 200
      ? (uint32_t)targetRate * age / 200
      : targetRate;
}

ISR(TCB1_INT_vect) {
  TCB1.INTFLAGS = TCB_CAPT_bm;

  // Always complete the previous pulse first.
  if (pulse) {
    digitalWrite(STEP, LOW);
    pulse = false;
  }

  // Always run the supervision clock, even at HOME.
  static uint8_t divider = 0;

  if (++divider == 10) {
    divider = 0;
    ++ms;
    superviseAxis();
  }

  if (!moving(state) ||
      !rate ||
      (state != TRACK && !remaining)) {
    return;
  }

  const bool rawHome =
      digitalRead(HOME_PIN) == LOW;

  // Terminal positive limit. No positive pulse
  // while the HOME switch is active.
  if (direction > 0 && rawHome) {
    if (state == HOMING) {
      rate = 0;
      return;  // Supervisor qualifies the switch.
    }

    fail(HOME_LOST);
    return;
  }

  // A production motion must never move beyond
  // the known home-referenced travel envelope.
  if (!cfg[CAL] && state != HOMING) {
    const int32_t lower =
        -((int32_t)cfg[RET] + (int32_t)cfg[DIVE]);

    if (direction < 0 && position <= lower) {
      fail(state == TRACK ? TRACK_LIMIT : SEARCH_END);
      return;
    }

    if (direction > 0 && position >= 0) {
      fail(HOME_MISSED);
      return;
    }
  }

  if (state == TRACK &&
      ((direction > 0 && position >= trackHigh) ||
       (direction < 0 && position <= trackLow))) {
    fail(TRACK_LIMIT);
    return;
  }

  phase += rate;

  if (phase < HZ) return;

  phase -= HZ;

  digitalWrite(STEP, HIGH);
  pulse = true;
  position += direction;

  if (state == TRACK || --remaining) return;

  if (state == JOG) {
    referenced = false;
    finish(1);
    state = UNKNOWN;

  } else if (state == PREP_MOVING) {
    if (digitalRead(HOME_PIN) == LOW ||
        !homeReleaseObserved) {
      fail(SWITCH_STUCK);
    } else {
      finish(1);
      state = READY;
      began = ms;
    }

  } else if (state == HOMING) {
    // The allowed step budget was consumed.
    // If the switch has not activated, homing failed.
    if (digitalRead(HOME_PIN) == LOW) {
      rate = 0;
      // Supervisor will qualify the contact.
    } else {
      fail(HOME_MISSED);
    }

  } else {
    fail(SEARCH_END);
  }
}

uint8_t checksum(const char *p) {
  uint8_t c = 0;
  while (*p) c ^= *p++;
  return c;
}

// Nonblocking transmit queue.
// Sensor frames may be dropped; room is reserved for control/status.
char tx[512];
uint16_t txHead = 0;
uint16_t txTail = 0;

uint16_t freeTx() {
  return (txTail - txHead - 1) & 511;
}

bool sendFrame(const char *p, bool sensor = false) {
  const uint16_t n = strlen(p) + 5;

  if (freeTx() < n + (sensor ? 256 : 0)) {
    if (sensor) ++imuDrops;
    return false;
  }

  const char hex[] = "0123456789ABCDEF";
  const uint8_t c = checksum(p);

  auto put = [](char b) {
    tx[txHead] = b;
    txHead = (txHead + 1) & 511;
  };

  put('$');
  while (*p) put(*p++);
  put('*');
  put(hex[c >> 4]);
  put(hex[c & 15]);
  put('\n');

  return true;
}

void flushTx() {
  int n = Serial.availableForWrite();

  while (n-- > 0 && txTail != txHead) {
    Serial.write(tx[txTail]);
    txTail = (txTail + 1) & 511;
  }
}

void ack(
    uint32_t s,
    uint32_t id,
    bool ok,
    const char *reply) {

  char b[100];

  snprintf(
      b, sizeof(b),
      "ACK,%lu,%lu,%u,%s",
      (unsigned long)s,
      (unsigned long)id,
      ok ? 1 : 0,
      reply);

  sendFrame(b);
}

bool number(const char *p, long &v) {
  if (!p || !*p) return false;

  errno = 0;
  char *end;
  v = strtol(p, &end, 10);

  return !errno && !*end;
}

bool validConfig(long *v) {
  if (v[CAL] < 0 || v[CAL] > 1 ||
      v[POS_HIGH] < 0 || v[POS_HIGH] > 1 ||
      v[HOME_DIR] != 1) {
    return false;
  }

  for (uint8_t i = RET; i <= HOME_MAX; ++i)
    if (v[i] < 1 || v[i] > 100000) return false;

  for (uint8_t i = RET_RATE; i <= SEARCH_RATE; ++i)
    if (v[i] < 1 || v[i] > 2000) return false;

  return
      (v[CAL] || v[HOME_MAX] >= v[RET] + v[DIVE]) &&
      v[SEARCH_MS] >= 500 &&
      v[SEARCH_MS] <= 30000 &&
      v[EST_MS] >= 1 &&
      v[EST_MS] <= 30000 &&
      v[TRACK_RATE] >= 0 &&
      v[TRACK_RATE] <= 2000 &&
      v[TRACK_OFFSET] >= 0 &&
      v[TRACK_OFFSET] <= 100000 &&
      ((v[TRACK_RATE] == 0) == (v[TRACK_OFFSET] == 0)) &&
      (!v[CAL] || v[TRACK_RATE] == 0) &&
      v[LINK_MS] >= 100 &&
      v[LINK_MS] <= 2000 &&
      v[JOG_MAX] >= 1 &&
      v[JOG_MAX] <= 500 &&
      v[JOG_RATE] >= 1 &&
      v[JOG_RATE] <= 1000 &&
      v[JOG_WINDOW] >= v[JOG_MAX] &&
      v[JOG_WINDOW] <= 100000;
}

void command(char *line) {
  if (*line != '$') return;

  char *star = strrchr(line, '*');

  if (!star || strlen(star + 1) != 2) {
    ++rxErrors;
    return;
  }

  char *end;
  const unsigned long sum = strtoul(star + 1, &end, 16);
  *star = 0;

  if (*end || sum > 255 || checksum(line + 1) != sum) {
    ++rxErrors;
    return;
  }

  char *a[24];
  char *save;
  uint8_t n = 0;

  for (char *p = strtok_r(line + 1, ",", &save);
       p;
       p = strtok_r(nullptr, ",", &save)) {
    if (n == 24) return;
    a[n++] = p;
  }

  long sid;
  long id;

  if (n < 3 || !number(a[1], sid) || sid <= 0)
    return;

  if (!strcmp(a[0], "P") &&
    n == 6 &&
    (uint32_t)sid == session) {

  long m, q, c, tracking;

  if (!number(a[2], m) || m < 0 || m > 5 ||
      !number(a[3], q) || q < 0 || q > 1 ||
      !number(a[4], c) || c < -1 || c > 1 ||
      !number(a[5], tracking) ||
      tracking < 0 || tracking > 1) {
    return;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    permit = m;
    qualified = q;
    trackingAuthorized =
        !cfg[CAL] && tracking == 1 && m == 2 && q == 1 &&
        (state == HOLD || state == TRACK);

    correction =
        trackingAuthorized ? (int8_t)c : 0;

    permitAt = ms;
  }

  return;
}

  if (strcmp(a[0], "C") ||
      n < 4 ||
      !number(a[2], id) ||
      id < 0) {
    return;
  }

  // STOP is the only operation allowed from a replacement session
  // before CONFIG. This permits recovery after a host reconnect.
  if (!strcmp(a[3], "STOP") &&
      n == 4 &&
      (uint32_t)sid != session) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      fail(STOPPED);
    }

    ack(sid, id, true, "STOPPED");
    return;
  }

  if (!strcmp(a[3], "CONFIG") && id == 0) {
    long v[NCFG];
    bool ok = n == 4 + NCFG;

    for (uint8_t i = 0; ok && i < NCFG; ++i)
      ok = number(a[i + 4], v[i]);

    ok = ok && validConfig(v);

    if (!ok) {
      ack(sid, 0, false, "BAD_CONFIG");
      return;
    }

    // Identical retry: ACK only. Do not reset position/motion.
    if ((uint32_t)sid == session && configured) {
      ok = !memcmp(v, cfg, sizeof(cfg));

      ack(
          sid, 0, ok,
          ok ? "CONFIGURED" : "CONFIG_CHANGED");

      return;
    }

    // A new session can take over only after motion is stopped.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ok = !moving(state) &&
           state != READY &&
           state != ESTABLISH &&
           state != HOLD;

      if (ok) {
        memcpy(cfg, v, sizeof(cfg));
        configured = true;
        session = sid;
        lastCommand = 0;

        finish(2);
        state = UNKNOWN;
        fault = NONE;
        referenced = false;
        position = 0;
        permit = 3;
        qualified = false;
        trackingAuthorized = false;
        homeReleaseObserved = false;
        correction = 0;

        activeId = 0;
        doneId = 0;
        doneResult = 0;
      }
    }

    ack(
        sid, 0, ok,
        ok ? "CONFIGURED" : "BUSY_STOP_FIRST");

    return;
  }

  if ((uint32_t)sid != session || !session || !id)
    return;

  // Only one ordinary command is in flight from the host.
  // Retrying its ID returns the original answer.
  if ((uint32_t)id == lastCommand) {
    ack(session, id, lastOk, lastReply);
    return;
  }

  if ((uint32_t)id < lastCommand) {
    ack(session, id, false, "OLD_COMMAND");
    return;
  }

  bool ok = false;
  const char *reply = "STATE_OR_PERMISSION";
  long arg = 0;

  if (!strcmp(a[3], "FLAP") &&
      n == 5 &&
      number(a[4], arg) &&
      arg >= 0 &&
      arg <= 180) {

    flap.write(arg);
    ok = true;
    reply = "FLAP_COMMAND_ACCEPTED";
  } else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      const bool fresh =
          ms - permitAt <= (uint32_t)cfg[LINK_MS];

      const bool off =
          cfg[CAL] || (fresh && permit == 0);

      if (!strcmp(a[3], "STOP") && n == 4) {
        fail(STOPPED);
        ok = true;
        reply = "STOPPED";

      } else if (
          !strcmp(a[3], "RESET_FAULT") &&
          n == 4 &&
          !moving(state)) {

        // Clearing a fault never enables movement or asserts HOME.
        // Therefore this is allowed even with absent TorchStatus.
        finish(2);
        state = UNKNOWN;
        fault = NONE;
        referenced = false;
        correction = 0;
        qualified = false;
        trackingAuthorized = false;
        homeReleaseObserved = false;

        ok = true;
        reply = "RESET_REHOME_REQUIRED";

      } else if (
          configured &&
          !strcmp(a[3], "JOG") &&
          n == 5 &&
          cfg[CAL] &&
          (state == UNKNOWN || state == HOME) &&
          number(a[4], arg) &&
          arg &&
          labs(arg) <= cfg[JOG_MAX] &&
          labs(position + arg) <= cfg[JOG_WINDOW]) {

        referenced = false;

        startMove(
            JOG,
            arg > 0 ? 1 : -1,
            labs(arg),
            cfg[JOG_RATE],
            labs(arg) * 1000UL / cfg[JOG_RATE] + 1500,
            id);

        ok = true;
        reply = "JOG_ACCEPTED";

  } else if (
      configured &&
      !strcmp(a[3], "HOME") &&
      n == 4 &&
      !cfg[CAL] &&
      fresh &&
      permit == 0 &&
      !moving(state) &&
      state != FAULT &&
      state != PAUSED) {

    const bool rawHome =
        digitalRead(HOME_PIN) == LOW;

    if (rawHome && switchOn) {
      // No movement: qualify the physically parked position.
      position = 0;
      referenced = true;
      homeReleaseObserved = false;
      trackingAuthorized = false;
      correction = 0;
      fault = NONE;
      state = HOME;

      doneId = (uint32_t)id;
      doneResult = 1;
      activeId = 0;

      ok = true;
      reply = "HOME_CONFIRMED";

    } else if (
        !rawHome &&
        !switchOn &&
        referenced &&
        homeReleaseObserved &&
        position < 0 &&
        -(int64_t)position <= cfg[HOME_MAX]) {

      // Normal return from a known negative position.
      // Positive direction only; raw switch stops pulses.
      const uint32_t steps =
          (uint32_t)(-(int64_t)position);

      startMove(
          HOMING,
          +1,
          steps,
          cfg[HOME_RATE],
          (uint32_t)(
              (uint64_t)steps * 1000ULL /
              (uint32_t)cfg[HOME_RATE] + 1500ULL),
          (uint32_t)id);

      ok = true;
      reply = "HOME_ACCEPTED";
    }

      } else if (
          configured &&
          !cfg[CAL] &&
          !strcmp(a[3], "PREP") &&
          n == 4 &&
          off &&
          state == HOME &&
          switchOn &&
          referenced) {

        // PREP advances negatively away from terminal HOME.
        homeReleaseObserved = false;
        startMove(
          PREP_MOVING,
          -1,
          cfg[RET],
          cfg[RET_RATE],
          cfg[RET] * 1000UL / cfg[RET_RATE] + 1500,
          id);

        ok = true;
        reply = "PREP_ACCEPTED";

      } else if (
          configured &&
          !cfg[CAL] &&
          !strcmp(a[3], "SEARCH") &&
          n == 4 &&
          fresh) {

        if (state == READY && permit == 1) {
          startMove(
            SEARCH,
            -1,
            cfg[DIVE],
            cfg[SEARCH_RATE],
            cfg[SEARCH_MS],
            id);

          ok = true;
        } else if (
            permit == 2 &&
            (state == READY ||
             state == ESTABLISH ||
             state == HOLD)) {

          if (state == HOLD) {
            doneId = id;
            doneResult = 1;
          } else {
            activeId = id;

            if (state == READY) {
              state = ESTABLISH;
              establishAt = ms;
            }
          }

          ok = true;
        }

        if (ok) reply = "SEARCH_ACCEPTED";
      }
    }
  }

  lastCommand = id;
  lastOk = ok;
  lastReply = reply;

  ack(session, id, ok, reply);
}

/*
 * Bounded ATmega4809 TWI master.
 *
 * Wire is used ONLY for initial pin routing and clock setup.
 * Do not call Wire transactions after setup: its interrupt master
 * is disabled and these transactions use bounded register polling.
 *
 * A failed sensor transaction resets the TWI peripheral and allows
 * the main loop to continue servicing serial communication.
 */
void resetBus() {
  TWI0.MCTRLA = 0;
  TWI0.MCTRLB = 0;
  TWI0.MCTRLA = TWI_ENABLE_bm;
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

bool busWait(uint8_t flag, uint32_t start) {
  while (!(TWI0.MSTATUS & flag)) {
    if ((TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm)) ||
        micros() - start > 2000) {
      return false;
    }
  }

  return !(TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm));
}

bool transaction(
    uint8_t addr,
    uint8_t reg,
    uint8_t *data,
    uint8_t n,
    bool read) {

  const uint32_t start = micros();
  bool ok = true;

  TWI0.MADDR = addr << 1;

  ok = busWait(TWI_WIF_bm, start) &&
       !(TWI0.MSTATUS & TWI_RXACK_bm);

  if (ok) {
    TWI0.MDATA = reg;

    ok = busWait(TWI_WIF_bm, start) &&
         !(TWI0.MSTATUS & TWI_RXACK_bm);
  }

  if (ok && read) {
    TWI0.MADDR = (addr << 1) | 1;

    for (uint8_t i = 0; ok && i < n; ++i) {
      ok = busWait(TWI_RIF_bm, start);

      if (ok) {
        data[i] = TWI0.MDATA;

        TWI0.MCTRLB =
            i + 1 == n
                ? TWI_ACKACT_bm | TWI_MCMD_STOP_gc
                : TWI_MCMD_RECVTRANS_gc;
      }
    }
  } else if (ok) {
    TWI0.MDATA = data[0];

    ok = busWait(TWI_WIF_bm, start) &&
         !(TWI0.MSTATUS & TWI_RXACK_bm);

    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
  }

  if (!ok) {
    ++i2cErrors;
    resetBus();
  }

  return ok;
}

bool initImu(uint8_t address) {
  uint8_t who = 0;

  // WHO_AM_I is 0x68 for BOTH selected bus addresses.
  if (!transaction(address, 0x75, &who, 1, true) ||
      who != 0x68) {
    return false;
  }

  // PLL, filtered 1kHz / 10 = 100Hz, +/-250deg/s, +/-2g.
  const uint8_t regs[] = {
    0x6B, 0x1A, 0x19, 0x1B, 0x1C, 0x38
  };

  uint8_t vals[] = {
    1, 3, 9, 0, 0, 1
  };

  for (uint8_t i = 0; i < 6; ++i) {
    if (!transaction(address, regs[i], &vals[i], 1, false))
      return false;
  }

  return true;
}

int16_t word(const uint8_t *b) {
  return (int16_t)((uint16_t)b[0] << 8 | b[1]);
}

void sample(uint8_t i) {
  static bool ready[2] = {false, false};
  static uint32_t retry[2] = {0, 0};
  static uint32_t seq[2] = {0, 0};

  const uint8_t addr = 0x68 + i;
  const uint32_t now = micros();

  if (!ready[i]) {
    if ((int32_t)(now - retry[i]) >= 0) {
      ready[i] = initImu(addr);
      retry[i] = now + 1000000UL;
    }
    return;
  }

  if (freeTx() < 350) {
    ++imuDrops;
    return;
  }

  uint8_t b[15];

  if (!transaction(addr, 0x3A, b, 15, true)) {
    ready[i] = false;
    retry[i] = now + 1000000UL;
    return;
  }

  if (!(b[0] & 1)) return;

  char out[100];

  snprintf(
      out, sizeof(out),
      "%s,%lu,%lu,%d,%d,%d,%d,%d,%d",
      i ? "M2" : "M",
      (unsigned long)seq[i]++,
      (unsigned long)now,
      word(b + 1),
      word(b + 3),
      word(b + 5),
      word(b + 9),
      word(b + 11),
      word(b + 13));

  sendFrame(out, true);
}

void setup() {
  pinMode(STEP, OUTPUT);
  digitalWrite(STEP, LOW);
  pinMode(DIR, OUTPUT);
  pinMode(HOME_PIN, INPUT_PULLUP);

  Serial.begin(230400);

  flap.attach(SERVO_PIN);
  flap.write(5);  // Your closed angle.

  TCB1.CTRLA = 0;
  TCB1.CTRLB = TCB_CNTMODE_INT_gc;
  TCB1.CCMP = F_CPU / 2 / HZ - 1;
  TCB1.CNT = 0;
  TCB1.INTFLAGS = TCB_CAPT_bm;
  TCB1.INTCTRL = TCB_CAPT_bm;
  TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;

  Wire.begin();
  Wire.setClock(400000);
  resetBus();
}

void loop() {
  static char line[240];
  static uint16_t used = 0;
  static bool overflow = false;
  bool ref, trackEnabled;
  flushTx();

  while (Serial.available()) {
    const char c = Serial.read();

    // Recover framing after a truncated or corrupted frame.
    if (c == '$') {
      used = 0;
      overflow = false;
    }

    if (c == '\n') {
      if (!overflow) {
        line[used] = 0;
        command(line);
      } else {
        ++rxErrors;
      }

      used = 0;
      overflow = false;
    } else if (c != '\r' && !overflow) {
      if (used < sizeof(line) - 1)
        line[used++] = c;
      else
        overflow = true;
    }
  }

  static uint32_t statusAt = 0;
  static uint32_t sampleAt = 0;

  const uint32_t now = micros();

  if (now - statusAt >= 50000UL) {
    statusAt = now;

    State s;
    uint8_t f, r;
    bool sw;
    int32_t p;
    uint32_t age, aid, did;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      s = state;
      f = fault;
      p = position;
      sw = switchOn;
      age = ms - permitAt;
      aid = activeId;
      did = doneId;
      r = doneResult;
      ref = referenced;
      trackEnabled = trackingAuthorized;
    }

    // Completion is retained in every status, so losing a single
    // status frame does not lose the completion result.
    char out[180];

    snprintf(
        out, sizeof(out),
        "H,%lu,%s,%ld,%u,%u,%lu,%lu,%lu,%u,%lu,%lu,%lu,%u,%u",
        (unsigned long)session,
        names[s],
        (long)p,
        sw ? 1 : 0,
        f,
        (unsigned long)age,
        (unsigned long)aid,
        (unsigned long)did,
        r,
        (unsigned long)imuDrops,
        (unsigned long)i2cErrors,
        (unsigned long)rxErrors,
        ref ? 1 : 0,
        trackEnabled ? 1 : 0
      );

    sendFrame(out);
  }

  // Alternate sensors: 100Hz each, avoiding back-to-back bursts.
  if (now - sampleAt >= 5000UL) {
    static uint8_t i = 0;

    sampleAt = now;
    sample(i);
    i ^= 1;
  }

  flushTx();
}