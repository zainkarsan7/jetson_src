#include <Wire.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// Nano Every: SDA=A4, SCL=A5, servo signal=D9.
// GY-521 with AD0 low.
static const uint8_t MPU = 0x68;
static const uint8_t SERVO_PIN = 9;
static const uint32_t SERIAL_BAUD = 230400;
static const uint32_t PERIOD_US = 5000; // 200 Hz target
bool imuReady = false;
Servo flap;
uint32_t nextSample = 0;
uint32_t seq = 0;
char line[80];
uint8_t pos = 0;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool readRegs(uint8_t reg, uint8_t *out, uint8_t n) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) out[i] = Wire.read();
  return true;
}
uint8_t checksum(const char *p) {
  uint8_t c = 0;
  while (*p) c ^= (uint8_t)*p++;
  return c;
}
void sendFrame(const char *payload) {
  // A full six-axis frame can exceed the UART ring buffer. Serial.print drains it.
  Serial.print('$'); Serial.print(payload); Serial.print('*');
  const char hex[] = "0123456789ABCDEF";
  uint8_t c = checksum(payload);
  Serial.print(hex[c >> 4]); Serial.print(hex[c & 15]); Serial.print('\n');
}
void command(char *s) {
  if (s[0] != '$') return;
  char *star = strrchr(s, '*');
  if (!star || strlen(star + 1) != 2) return;
  uint8_t expected = (uint8_t)strtoul(star + 1, nullptr, 16);
  *star = 0;
  char *payload = s + 1;
  if (checksum(payload) != expected) return;
  if (strncmp(payload, "FLAP,", 5) != 0) return;
  char *comma = strchr(payload + 5, ',');
  if (!comma) return;
  *comma = 0;
  char *end1 = nullptr; char *end2 = nullptr;
  unsigned long id = strtoul(payload + 5, &end1, 10);
  long angle = strtol(comma + 1, &end2, 10);
  if (*end1 || *end2 || angle < 0 || angle > 180) return;
  flap.write((int)angle);
  char ack[45];
  snprintf(ack, sizeof(ack), "ACK,%lu,%ld", id, angle);
  sendFrame(ack);
}
int16_t signedWord(const uint8_t *bytes) {
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000);
  flap.attach(SERVO_PIN);
  flap.write(0); // Default OPEN. Verify mechanical direction before installation.
  delay(50);
  uint8_t id;
  if (!readRegs(0x75, &id, 1) || id != 0x68) {
    sendFrame("ERROR,MPU6050_ID");
    return;
  }
  // PLL clock, 1 kHz filtered gyro / (1+4), +/-250 deg/s, +/-2 g.
  if (!writeReg(0x6B, 0x01) || !writeReg(0x1A, 0x03) ||
      !writeReg(0x19, 4) || !writeReg(0x1B, 0) || !writeReg(0x1C, 0) ||
      !writeReg(0x38, 1)) {
    sendFrame("ERROR,MPU6050_CONFIG");
    return;
  }
  imuReady = true;
  nextSample = micros() + PERIOD_US;
}
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line[pos] = 0; command(line); pos = 0;
    } else if (c != '\r') {
      if (pos < sizeof(line) - 1) line[pos++] = c;
      else pos = 0;
    }
  }
  if (!imuReady) return;
  uint32_t now = micros();
  if ((int32_t)(now - nextSample) >= 0) {
    nextSample += PERIOD_US;
    if ((int32_t)(now - nextSample) >= 0) nextSample = now + PERIOD_US;
    // One burst from INT_STATUS through GYRO_ZOUT_L. Skip temperature bytes 7-8.
    uint8_t b[15];
    if (!readRegs(0x3A, b, sizeof(b)) || !(b[0] & 1)) return;
    char payload[80];
    snprintf(payload, sizeof(payload), "M,%lu,%lu,%d,%d,%d,%d,%d,%d",
             (unsigned long)seq++, (unsigned long)now, signedWord(b+1), signedWord(b+3),
             signedWord(b+5), signedWord(b+9), signedWord(b+11), signedWord(b+13));
    sendFrame(payload);
  }
}
