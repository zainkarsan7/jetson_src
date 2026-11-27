#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#if !defined(__AVR_ATmega4809__)
#error "Select Arduino Nano Every"
#endif

/*
 * TORCH HEIGHT NANO EVERY
 *
 * D4 STEP, D5 DIR -> suitable buffers -> DM542T PUL+/DIR+.
 * Common-cathode wiring: PUL-/DIR- -> signal GND.
 * Separate motor supply; do not power the motor driver from servo 5V.
 *
 * D7 -> HOME switch -> GND.
 * INPUT_PULLUP, active LOW, 5ms debounce.
 *
 * Positive steps RETRACT toward terminal HOME.
 * Negative steps ADVANCE toward the workpiece.
 * PREP and SEARCH both advance negatively.
 *
 * TCB1 generates STEP pulses: 10kHz interrupt, 100us pulse HIGH.
 * Supervision runs at 1kHz.
 *
 * No IMU, servo or I2C workload on this Nano.
 * No direct plasma-voltage connection.
 * Torch feedback and voltage corrections arrive over USB.
 *
 * Calibration: physically inhibit the torch.
 * RESET_FAULT never moves or asserts HOME.
 * HOME requires production mode and fresh confirmed torch OFF.
 * Startup may confirm an already debounced HOME switch without any STEP.
 * A fault, reset or motion command cancels that startup-only confirmation.
 * Tracking preserves fractional steps across same-direction deadband holds.
 *
 * SEARCH owns the transition from READY:
 *   permit 1 -> SEARCH motion
 *   permit 2 -> stationary ESTABLISH
 *
 * Repeating the same command ID returns the original ACK.
 * A new SEARCH command ID requires READY.
 */

const uint8_t STEP = 4;
const uint8_t DIR = 5;
const uint8_t HOME_PIN = 7;
const uint16_t HZ = 10000;
// At the production 20 steps/s tracking rate, a 200ms ramp on every short
// correction can prevent all steps. Start TRACK at up to 20 steps/s; higher
// tracking rates still ramp over 200ms. Never exceed the configured rate.
const uint16_t TRACK_START_RATE = 20;

enum State : uint8_t {
  UNKNOWN, HOME, PREP_MOVING, READY, SEARCH, ESTABLISH,
  HOLD, TRACK, HOMING, JOG, PAUSED, FAULT
};

const char *names[] = {
  "UNKNOWN", "HOME", "PREP_MOVING", "READY", "SEARCH", "ESTABLISH",
  "HOLD", "TRACK", "HOMING", "JOG", "PAUSED", "FAULT"
};

enum Fault : uint8_t {
  NONE, STOPPED, LINK_LOST, TORCH_INVALID, SEARCH_END,
  HOME_MISSED, SWITCH_STUCK, HOME_LOST, BAD_CONFIG,
  ARC_UNSTABLE, TRACK_LIMIT
};

// Integer CONFIG fields, retaining protocol v2 order.
enum {
  CAL, RET, DIVE, HOME_MAX, RET_RATE, HOME_RATE, SEARCH_RATE,
  SEARCH_MS, POS_HIGH, HOME_DIR, EST_MS, TRACK_RATE,
  TRACK_OFFSET, LINK_MS, JOG_MAX, JOG_RATE, JOG_WINDOW, NCFG
};

long cfg[NCFG] = {0};
bool configured = false;
uint32_t session = 0;
uint32_t lastCommand = 0;
bool lastOk = false;
const char *lastReply = "BOOT";

volatile State state = UNKNOWN;
volatile uint8_t fault = NONE;
volatile uint8_t permit = 3;

volatile bool qualified = false;
volatile bool switchOn = false;
volatile bool pulse = false;
volatile bool referenced = false;
volatile bool trackingAuthorized = false;
volatile bool homeReleaseObserved = false;
volatile bool startupHomePending = false;
volatile bool trackingProgressValid = false;

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
volatile uint32_t rampBegan = 0;
volatile bool searchResumeAllowed = false;
volatile uint32_t remaining = 0;
volatile uint32_t activeId = 0;
volatile uint32_t doneId = 0;

// Completion: 0 none, 1 completed, 2 aborted.
volatile uint8_t doneResult = 0;
volatile uint16_t rate = 0;
volatile uint16_t targetRate = 0;
volatile uint16_t phase = 0;

uint32_t rxErrors = 0;

// Reserved fields retained for compatibility with the existing bridge.
const uint32_t imuDrops = 0;
const uint32_t i2cErrors = 0;

/*
 * Permission modes:
 * 0 fresh confirmed OFF
 * 1 ON, no transfer
 * 2 ON, transfer
 * 3 stale/invalid/faulted feedback
 * 4 shutdown commanded, output/transfer not yet cleared
 * 5 ignition commanded, waiting for output
 */

bool moving(State s) {
  return s == PREP_MOVING || s == SEARCH || s == TRACK ||
         s == HOMING || s == JOG;
}

// Called from ISR or inside ATOMIC_BLOCK.
void finish(uint8_t result) {
  searchResumeAllowed = false;
  trackingProgressValid = false;
  phase = 0;
  rate = 0;
  targetRate = 0;

  if (activeId) {
    doneId = activeId;
    doneResult = result;
    activeId = 0;
  }
}

void fail(uint8_t reason, bool pause = false) {
  startupHomePending = false;
  // A later containment STOP must not erase the initiating hardware fault.
  if (reason == STOPPED && fault != NONE && (state == FAULT || state == PAUSED))
    reason = fault;
  finish(2);
  correction = 0;
  qualified = false;
  trackingAuthorized = false;
  fault = reason;
  state = pause ? PAUSED : FAULT;
}

void startMove(
    State s, int8_t dir, uint32_t steps, uint16_t speed,
    uint32_t timeout, uint32_t id) {

  startupHomePending = false;
  trackingProgressValid = false;

  digitalWrite(
      DIR,
      (dir > 0) == bool(cfg[POS_HIGH]) ? HIGH : LOW);

  direction = dir;
  remaining = steps;
  targetRate = speed;
  rate = 0;
  phase = 0;
  began = ms;
  rampBegan = ms;
  searchResumeAllowed = (s == SEARCH);
  budget = timeout;
  activeId = id;
  fault = NONE;
  state = s;
}

void superviseAxis() {
  static uint8_t onCount = 0;
  static uint8_t offCount = 0;

  const bool raw = digitalRead(HOME_PIN) == LOW;

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

  if (state == PREP_MOVING && offCount == 5)
    homeReleaseObserved = true;

  const bool fresh =
      (uint32_t)(ms - permitAt) <= (uint32_t)cfg[LINK_MS];

  // This establishes a reference only; it never seeks or emits a STEP. Use
  // both raw contact and the local 5ms debounce, not a cached ROS switch value.
  if (startupHomePending && !cfg[CAL] && state == UNKNOWN &&
      fault == NONE && !referenced && !activeId && !trackingAuthorized &&
      fresh && permit == 0 && raw && switchOn) {
    finish(1);
    position = 0;
    referenced = true;
    qualified = false;
    correction = 0;
    homeReleaseObserved = false;
    startupHomePending = false;
    state = HOME;
    return;
  }

  // Calibration jogs have local time/travel bounds.
  const bool local =
      cfg[CAL] && (state == JOG || state == HOMING);

  if (moving(state) || state == READY ||
      state == ESTABLISH || state == HOLD) {
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
    // SEARCH owns READY -> ESTABLISH.
    // Feedback alone cannot start establishment.
    // Earlier watchdog and invalid-feedback checks remain active.
    rate = 0;

    if (permit == 4 || ms - began > 10000UL)
      fail(TORCH_INVALID);

    return;
  }

  if (state == SEARCH) {
    // Stop descent on the first transfer indication.
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

    // Pauses count toward the ORIGINAL search wall-time budget. No new command,
    // additional travel allowance or new timeout is created when a pulse fades.
    if (searchResumeAllowed && (!remaining || ms - began >= budget)) {
      fail(SEARCH_END);
      return;
    }
    if (permit == 1 && searchResumeAllowed) {
      state = SEARCH;
      phase = 0;
      rampBegan = ms; // ramp up again; leave began/budget/remaining/id intact
      return;
    }

    if (permit == 2 && qualified) {
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
    // Normal torch shutdown stops corrections without homing.
    if (permit == 0 || permit == 4) {
      rate = 0;
      trackingProgressValid = false;
      phase = 0;
      state = HOLD;
      return;
    }

    if (permit != 2 || !qualified) {
      fail(ARC_UNSTABLE);
      return;
    }

    if (!trackingAuthorized || !cfg[TRACK_RATE] || !correction) {
      if (!trackingAuthorized || !cfg[TRACK_RATE]) {
        trackingProgressValid = false;
        phase = 0;
      }
      // A zero correction is an authorized deadband/dwell hold. Keep the
      // fractional step only while tracking stays armed in this cut.
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
      const uint16_t savedPhase =
          trackingProgressValid && direction == correction ? phase : 0;
      startMove(TRACK, correction, 0, cfg[TRACK_RATE], 0, 0);
      phase = savedPhase;
      trackingProgressValid = true;
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
    // Stop on raw contact; establish HOME after stationary debounce.
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

  const uint32_t age = ms - rampBegan;
  const uint16_t initialRate = state == TRACK
      ? (targetRate < TRACK_START_RATE ? targetRate : TRACK_START_RATE)
      : 0;
  rate = age < 200
      ? initialRate + (uint32_t)(targetRate - initialRate) * age / 200
      : targetRate;
}

ISR(TCB1_INT_vect) {
  TCB1.INTFLAGS = TCB_CAPT_bm;

  // Always finish the previous pulse first.
  if (pulse) {
    digitalWrite(STEP, LOW);
    pulse = false;
  }

  static uint8_t divider = 0;
  if (++divider == 10) {
    divider = 0;
    ++ms;
    superviseAxis();
  }

  if (!moving(state) || !rate ||
      (state != TRACK && !remaining)) {
    return;
  }

  const bool rawHome = digitalRead(HOME_PIN) == LOW;

  // No positive pulse into an active terminal switch.
  if (direction > 0 && rawHome) {
    if (state == HOMING) {
      rate = 0;
      return;
    }

    fail(HOME_LOST);
    return;
  }

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
    if (digitalRead(HOME_PIN) == LOW || !homeReleaseObserved) {
      fail(SWITCH_STUCK);
    } else {
      finish(1);
      state = READY;
      began = ms;
    }

  } else if (state == HOMING) {
    if (digitalRead(HOME_PIN) == LOW)
      rate = 0;
    else
      fail(HOME_MISSED);

  } else {
    fail(SEARCH_END);
  }
}

uint8_t checksum(const char *p) {
  uint8_t c = 0;
  while (*p) c ^= *p++;
  return c;
}

char tx[512];
uint16_t txHead = 0;
uint16_t txTail = 0;

uint16_t freeTx() {
  return (txTail - txHead - 1) & 511;
}

bool sendFrame(const char *p) {
  const uint16_t n = strlen(p) + 5;
  if (freeTx() < n) return false;

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

void ack(uint32_t s, uint32_t id, bool ok, const char *reply) {
  // Includes room for detailed SEARCH rejection diagnostics.
  char b[224];

  snprintf(
      b, sizeof(b), "ACK,%lu,%lu,%u,%s",
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
      v[SEARCH_MS] >= 500 && v[SEARCH_MS] <= 30000 &&
      v[EST_MS] >= 1 && v[EST_MS] <= 30000 &&
      v[TRACK_RATE] >= 0 && v[TRACK_RATE] <= 2000 &&
      v[TRACK_OFFSET] >= 0 && v[TRACK_OFFSET] <= 100000 &&
      ((v[TRACK_RATE] == 0) == (v[TRACK_OFFSET] == 0)) &&
      (!v[CAL] || v[TRACK_RATE] == 0) &&
      v[LINK_MS] >= 100 && v[LINK_MS] <= 2000 &&
      v[JOG_MAX] >= 1 && v[JOG_MAX] <= 500 &&
      v[JOG_RATE] >= 1 && v[JOG_RATE] <= 1000 &&
      v[JOG_WINDOW] >= v[JOG_MAX] && v[JOG_WINDOW] <= 100000;
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

  long sid, id;
  if (n < 3 || !number(a[1], sid) || sid <= 0) return;

  if (!strcmp(a[0], "P") && n == 6 && (uint32_t)sid == session) {
    long m, q, c, tracking;

    if (!number(a[2], m) || m < 0 || m > 5 ||
        !number(a[3], q) || q < 0 || q > 1 ||
        !number(a[4], c) || c < -1 || c > 1 ||
        !number(a[5], tracking) || tracking < 0 || tracking > 1) {
      return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      permit = m;
      qualified = q;

      trackingAuthorized =
          !cfg[CAL] && tracking == 1 && m == 2 && q == 1 &&
          (state == HOLD || state == TRACK);

      correction = trackingAuthorized ? (int8_t)c : 0;
      permitAt = ms;
    }

    return;
  }

  if (strcmp(a[0], "C") || n < 4 ||
      !number(a[2], id) || id < 0) {
    return;
  }

  // A replacement host session may STOP before CONFIG.
  if (!strcmp(a[3], "STOP") &&
      n == 4 && (uint32_t)sid != session) {
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

    // Same-session CONFIG retries never reset motion or position.
    if ((uint32_t)sid == session && configured) {
      ok = !memcmp(v, cfg, sizeof(cfg));
      ack(sid, 0, ok, ok ? "CONFIGURED" : "CONFIG_CHANGED");
      return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ok = !moving(state) && state != READY &&
           state != ESTABLISH && state != HOLD;

      if (ok) {
        // Do not turn a reconnect from FAULT/PAUSED, or an explicit reset in
        // UNKNOWN, into automatic fault recovery. Same-session retries return
        // above and cannot rearm startup confirmation.
        const bool allowStartupHome = fault == NONE && !activeId &&
            !trackingAuthorized &&
            ((state == UNKNOWN && lastCommand == 0 &&
              (!configured || startupHomePending)) ||
             (state == HOME && referenced));
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
        startupHomePending = allowStartupHome;
      }
    }

    ack(sid, 0, ok, ok ? "CONFIGURED" : "BUSY_STOP_FIRST");
    return;
  }

  if ((uint32_t)sid != session || !session || !id) return;

  // Retrying an accepted motion command does not start it again.
  // A rejected same-ID command also returns its original rejection.
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

  // Capture rejection evidence with interrupts disabled, then format
  // outside ATOMIC_BLOCK so formatting cannot delay STEP interrupts.
  State searchState = UNKNOWN;
  uint8_t searchPermit = 3;
  uint8_t searchFault = NONE;
  uint32_t searchAge = 0;

  // lastReply retains this pointer for same-ID retries.
  // A local non-static array would leave lastReply dangling.
  static char searchReply[160];

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    const bool fresh =
        ms - permitAt <= (uint32_t)cfg[LINK_MS];

    const bool off = cfg[CAL] || (fresh && permit == 0);

    if (!strcmp(a[3], "STOP") && n == 4) {
      fail(STOPPED);
      ok = true;
      reply = "STOPPED";

    } else if (
        !strcmp(a[3], "RESET_FAULT") &&
        n == 4 && !moving(state)) {

      // No movement; explicit HOME still required.
      startupHomePending = false;
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
        configured && !strcmp(a[3], "JOG") && n == 5 &&
        cfg[CAL] && (state == UNKNOWN || state == HOME) &&
        number(a[4], arg) && arg &&
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
        configured && !strcmp(a[3], "HOME") && n == 4 &&
        !cfg[CAL] && fresh && permit == 0 &&
        !moving(state) && state != FAULT && state != PAUSED) {

      const bool rawHome = digitalRead(HOME_PIN) == LOW;

      startupHomePending = false;
      trackingAuthorized = false;
      correction = 0;
      qualified = false;
      homeReleaseObserved = false;

      if (rawHome && switchOn) {
        // Already on a debounced HOME switch: no STEP is issued.
        finish(1);
        position = 0;
        referenced = true;
        fault = NONE;
        state = HOME;
        activeId = 0;
        doneId = (uint32_t)id;
        doneResult = 1;

        ok = true;
        reply = "HOME_CONFIRMED";

      } else {
        // Seek independently of the open-loop position estimate.
        const uint32_t steps = (uint32_t)cfg[HOME_MAX];
        const uint32_t timeout =
            (uint32_t)(
                (uint64_t)steps * 1000ULL /
                (uint32_t)cfg[HOME_RATE] + 1500ULL);

        startMove(
            HOMING,
            +1,
            steps,
            (uint16_t)cfg[HOME_RATE],
            timeout,
            (uint32_t)id);

        ok = true;
        reply = "HOME_SEEK_ACCEPTED";
      }

    } else if (
        configured && !cfg[CAL] &&
        !strcmp(a[3], "PREP") && n == 4 &&
        off && state == HOME && switchOn && referenced) {

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

    } else if (!strcmp(a[3], "SEARCH") && n == 4) {
      // Capture the exact state/permission used for this decision.
      searchState = state;
      searchPermit = permit;
      searchFault = fault;
      searchAge = ms - permitAt;

      if (!configured) {
        reply = "SEARCH_NOT_CONFIGURED";

      } else if (cfg[CAL]) {
        reply = "SEARCH_CALIBRATION_MODE";

      } else if (!fresh) {
        reply = "SEARCH_LINK_STALE";

      } else if (fault != NONE || state == FAULT || state == PAUSED) {
        reply = "SEARCH_FAULT_LATCHED";

      } else if (state != READY) {
        reply = "SEARCH_WRONG_STATE";

      } else if (!referenced) {
        reply = "SEARCH_NOT_REFERENCED";

      } else if (permit == 1) {
        // Output ON, transfer absent: begin bounded search motion.
        startMove(
            SEARCH,
            -1,
            cfg[DIVE],
            cfg[SEARCH_RATE],
            cfg[SEARCH_MS],
            id);

        ok = true;
        reply = "SEARCH_ACCEPTED";

      } else if (permit == 2) {
        // Initialize the same search envelope, but hold stationary immediately.
        // If this unqualified indication falls, only the unused budget resumes.
        // ATOMIC_BLOCK prevents any STEP between startMove and ESTABLISH.
        startMove(SEARCH, -1, cfg[DIVE], cfg[SEARCH_RATE], cfg[SEARCH_MS], id);
        trackingAuthorized = false;
        correction = 0;
        establishAt = ms;
        state = ESTABLISH;

        ok = true;
        reply = "SEARCH_ACCEPTED";

      } else {
        reply = "SEARCH_WRONG_PERMISSION";
      }
    }
  } // End ATOMIC_BLOCK. Formatting and ACK remain outside it.

  if (!ok && !strcmp(a[3], "SEARCH") && n == 4) {
    snprintf(
        searchReply,
        sizeof(searchReply),
        "%s|state=%s|permit=%u|fault=%u|age_ms=%lu",
        reply,
        names[searchState],
        (unsigned int)searchPermit,
        (unsigned int)searchFault,
        (unsigned long)searchAge);

    reply = searchReply;
  }

  lastCommand = id;
  lastOk = ok;
  lastReply = reply;

  ack(session, id, ok, reply);
}

void setup() {
  pinMode(STEP, OUTPUT);
  digitalWrite(STEP, LOW);
  pinMode(DIR, OUTPUT);
  pinMode(HOME_PIN, INPUT_PULLUP);

  Serial.begin(230400);

  TCB1.CTRLA = 0;
  TCB1.CTRLB = TCB_CNTMODE_INT_gc;
  TCB1.CCMP = F_CPU / 2 / HZ - 1;
  TCB1.CNT = 0;
  TCB1.INTFLAGS = TCB_CAPT_bm;
  TCB1.INTCTRL = TCB_CAPT_bm;
  TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
}

void loop() {
  static char line[240];
  static uint16_t used = 0;
  static bool overflow = false;

  flushTx();

  while (Serial.available()) {
    const char c = Serial.read();

    // Resynchronize after a truncated/corrupted frame.
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
  const uint32_t now = micros();

  if (now - statusAt >= 50000UL) {
    statusAt = now;

    State s;
    uint8_t f, r;
    bool sw, ref, trackEnabled;
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

    // Completion remains in every subsequent status frame.
    char out[180];

    snprintf(
        out,
        sizeof(out),
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
        trackEnabled ? 1 : 0);

    sendFrame(out);
  }

  flushTx();
}
