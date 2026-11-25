#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#if !defined(__AVR_ATmega4809__)
#error "Select Arduino Nano Every"
#endif

/*
 * PLATFORM NANO EVERY
 *
 * D13 -> camera flap servo signal.
 * Servo uses external 5V power, with common signal GND.
 * Boot position: 5 degrees, your CLOSED position.
 *
 * A4/SDA and A5/SCL -> 5V side of an I2C level shifter.
 * Level shifter 3.3V side -> both MPU6050 SDA/SCL connections.
 *
 * MPU1 AD0 -> GND:  address 0x68.
 * MPU2 AD0 -> 3.3V: address 0x69.
 * Power each breakout according to its rating.
 * XDA, XCL and INT are unused.
 *
 * D4/D5/D7 unused on this board.
 * Stepper and HOME switch belong to the HEIGHT Nano.
 *
 * USB: 230400 baud.
 * Frames: $payload*XX\n, XX is payload XOR.
 *
 * Host: FLAP,id,angle
 * Nano: ACK,id,angle
 *
 * M/M2: seq,micros,ax,ay,az,gx,gy,gz
 * Raw ranges: +/-2g and +/-250 degrees/s.
 *
 * S: version,uptime_ms,servo_angle,imu1_ready,imu2_ready,
 *    dropped_sensor_frames,i2c_errors
 *
 * ACK/status report commanded servo position, not measured position.
 */

constexpr uint8_t SERVO_PIN = 13;
constexpr uint8_t BOOT_CLOSED_DEG = 5;

Servo flap;
uint8_t flapAngle = BOOT_CLOSED_DEG;

bool imuReady[2] = {false, false};
uint32_t imuRetry[2] = {0, 0};
uint32_t imuSeq[2] = {0, 0};
uint32_t imuDrops = 0;
uint32_t i2cErrors = 0;

char tx[512];
uint16_t txHead = 0;
uint16_t txTail = 0;

uint8_t checksum(const char *p) {
  uint8_t value = 0;
  while (*p) value ^= (uint8_t)*p++;
  return value;
}

uint16_t txFree() {
  return (txTail - txHead - 1) & 511;
}

bool sendFrame(const char *p, bool sensor = false) {
  const uint16_t needed = strlen(p) + 5;

  // Reserve room for flap ACKs and status.
  if (txFree() < needed + (sensor ? 192 : 0)) {
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
  int available = Serial.availableForWrite();
  while (available-- > 0 && txTail != txHead) {
    Serial.write(tx[txTail]);
    txTail = (txTail + 1) & 511;
  }
}

bool number(const char *p, long &value) {
  if (!p || !*p) return false;
  errno = 0;
  char *end;
  value = strtol(p, &end, 10);
  return !errno && !*end;
}

void command(char *line) {
  if (*line != '$') return;

  char *star = strrchr(line, '*');
  if (!star || strlen(star + 1) != 2) return;

  char *end;
  unsigned long expected = strtoul(star + 1, &end, 16);
  *star = 0;

  if (*end || expected > 255 ||
      checksum(line + 1) != expected) {
    return;
  }

  char *save;
  char *op = strtok_r(line + 1, ",", &save);
  char *idText = strtok_r(nullptr, ",", &save);
  char *angleText = strtok_r(nullptr, ",", &save);

  if (!op || strcmp(op, "FLAP") ||
      strtok_r(nullptr, ",", &save)) {
    return;
  }

  long id, angle;
  if (!number(idText, id) || id <= 0 ||
      !number(angleText, angle) ||
      angle < 0 || angle > 180) {
    return;
  }

  // Repeated commands simply set the same angle.
  flapAngle = (uint8_t)angle;
  flap.write(flapAngle);

  char reply[48];
  snprintf(reply, sizeof(reply), "ACK,%ld,%ld", id, angle);
  sendFrame(reply);
}

/*
 * Bounded ATmega4809 I2C transactions.
 * Wire is used ONLY for pin routing and clock initialization.
 * Do not use Wire transactions after resetBus().
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
    uint8_t address,
    uint8_t reg,
    uint8_t *data,
    uint8_t count,
    bool read) {

  uint32_t start = micros();
  TWI0.MADDR = address << 1;

  bool ok = busWait(TWI_WIF_bm, start) &&
            !(TWI0.MSTATUS & TWI_RXACK_bm);

  if (ok) {
    TWI0.MDATA = reg;
    ok = busWait(TWI_WIF_bm, start) &&
         !(TWI0.MSTATUS & TWI_RXACK_bm);
  }

  if (ok && read) {
    TWI0.MADDR = (address << 1) | 1;

    for (uint8_t i = 0; ok && i < count; ++i) {
      ok = busWait(TWI_RIF_bm, start);
      if (ok) {
        data[i] = TWI0.MDATA;
        TWI0.MCTRLB =
            i + 1 == count
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
  uint8_t identity = 0;

  // WHO_AM_I is 0x68 for both selected I2C addresses.
  if (!transaction(address, 0x75, &identity, 1, true) ||
      identity != 0x68) {
    return false;
  }

  const uint8_t regs[] = {
    0x6B, 0x1A, 0x19, 0x1B, 0x1C, 0x38
  };
  uint8_t values[] = {
    1, 3, 9, 0, 0, 1
  };  // 100Hz, +/-2g, +/-250 degrees/s.

  for (uint8_t i = 0; i < 6; ++i) {
    if (!transaction(address, regs[i], &values[i], 1, false))
      return false;
  }
  return true;
}

int16_t signedWord(const uint8_t *b) {
  return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

void sampleImu(uint8_t i) {
  uint32_t now = micros();
  uint8_t address = 0x68 + i;

  if (!imuReady[i]) {
    if ((int32_t)(now - imuRetry[i]) >= 0) {
      imuReady[i] = initImu(address);
      imuRetry[i] = now + 1000000UL;
    }
    return;
  }

  uint8_t b[15];
  if (!transaction(address, 0x3A, b, sizeof(b), true)) {
    imuReady[i] = false;
    imuRetry[i] = now + 1000000UL;
    return;
  }

  if (!(b[0] & 1)) return;

  char payload[100];
  snprintf(
      payload, sizeof(payload),
      "%s,%lu,%lu,%d,%d,%d,%d,%d,%d",
      i ? "M2" : "M",
      (unsigned long)imuSeq[i]++,
      (unsigned long)now,
      signedWord(b + 1),
      signedWord(b + 3),
      signedWord(b + 5),
      signedWord(b + 9),
      signedWord(b + 11),
      signedWord(b + 13));

  sendFrame(payload, true);
}

void setup() {
  Serial.begin(230400);
  flap.attach(SERVO_PIN);
  flap.write(BOOT_CLOSED_DEG);

  Wire.begin();
  Wire.setClock(400000);
  resetBus();
}

void loop() {
  static char line[96];
  static uint8_t used = 0;
  static bool overflow = false;

  flushTx();

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '$') {
      used = 0;
      overflow = false;
    }

    if (c == '\n') {
      if (!overflow) {
        line[used] = 0;
        command(line);
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

  static uint32_t sampleAt = 0;
  static uint32_t statusAt = 0;
  uint32_t now = micros();

  if (now - statusAt >= 250000UL) {
    statusAt = now;
    char payload[100];

    snprintf(
        payload, sizeof(payload),
        "S,1,%lu,%u,%u,%u,%lu,%lu",
        (unsigned long)millis(),
        flapAngle,
        imuReady[0] ? 1 : 0,
        imuReady[1] ? 1 : 0,
        (unsigned long)imuDrops,
        (unsigned long)i2cErrors);

    sendFrame(payload);
  }

  // Alternate sensors, targeting 100Hz per sensor.
  if (now - sampleAt >= 5000UL) {
    static uint8_t i = 0;
    sampleAt = now;
    sampleImu(i);
    i ^= 1;
  }

  flushTx();
}