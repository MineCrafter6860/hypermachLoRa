// ============================================================
// E22-900T30D  —  TRANSMITTER (Pico / arduino-pico)
// TEKNOFEST HYİ Protocol — 78-byte binary packet
//
// UART0 : GP0=TX  GP1=RX   (Serial1) — E22 LoRa
// M0    : GP2  (output)
// M1    : GP3  (output)
// AUX   : GP15 (input)  → onboard LED (GP25)
// I2C0  : GP4=SDA  GP5=SCL
//   MPU6050 @ 0x68  (direct register writes — clone safe)
//   BME280  @ 0x76  (Adafruit BME280 library)
// ============================================================

#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ---- Pin definitions ---------------------------------------
#define PIN_M0        2
#define PIN_M1        3
#define PIN_AUX       15
#define LED_PIN       25

// ---- I2C addresses -----------------------------------------
#define MPU_ADDR      0x68
#define BME_ADDR      0x76
#define SEA_LEVEL_HPA 1013.25f

// ---- HYİ protocol ------------------------------------------
#define TEAM_ID       0x00   // TODO: replace with assigned team ID
#define PACKET_SIZE   78

// ---- MPU6050 registers -------------------------------------
#define MPU_PWR_MGMT_1   0x6B
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG_REG   0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_WHO_AM_I     0x75

// ---- Scale factors -----------------------------------------
// Accel: keep in g for HYİ packet; also compute m/s² for debug
#define ACCEL_SCALE_G   (1.0f / 16384.0f)        // ±2g → g
#define ACCEL_SCALE_MS2 (9.80665f / 16384.0f)    // ±2g → m/s²
#define GYRO_SCALE      (1.0f / 131.0f)           // ±250°/s → °/s

// ---- Struct declared at top so all functions can see it ----
struct ImuData {
  float ax_g,  ay_g,  az_g;   // acceleration in g  (for packet)
  float ax_ms2,ay_ms2,az_ms2; // acceleration in m/s² (for debug)
  float gx, gy, gz;            // gyroscope in °/s
};

// ---- Float→bytes union (IEEE 754 little-endian) ------------
typedef union {
  float    value;
  uint8_t  bytes[4];
} FLOAT32_CONV;

// ---- Objects -----------------------------------------------
Adafruit_BME280 bme;

// ---- Global state ------------------------------------------
uint8_t  packet_counter      = 0;
float    pad_altitude_ref    = 0.0f;

// ============================================================
//  HYİ packet helpers
// ============================================================
void insertFloat(uint8_t* pkt, uint8_t idx, float val) {
  FLOAT32_CONV c;
  c.value = val;
  pkt[idx]     = c.bytes[0];
  pkt[idx + 1] = c.bytes[1];
  pkt[idx + 2] = c.bytes[2];
  pkt[idx + 3] = c.bytes[3];
}

uint8_t calcChecksum(uint8_t* pkt) {
  uint32_t sum = 0;
  for (int i = 4; i < 75; i++) sum += pkt[i];  // bytes[4..74] (0-indexed)
  return (uint8_t)(sum % 256);
}

// ============================================================
//  MPU6050 — direct register access (clone-safe)
// ============================================================
void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

int16_t readInt16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

ImuData mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

  int16_t axRaw = readInt16();
  int16_t ayRaw = readInt16();
  int16_t azRaw = readInt16();
  readInt16();  // MPU internal temp — discard
  int16_t gxRaw = readInt16();
  int16_t gyRaw = readInt16();
  int16_t gzRaw = readInt16();

  ImuData d;
  d.ax_g   = axRaw * ACCEL_SCALE_G;
  d.ay_g   = ayRaw * ACCEL_SCALE_G;
  d.az_g   = azRaw * ACCEL_SCALE_G;
  d.ax_ms2 = axRaw * ACCEL_SCALE_MS2;
  d.ay_ms2 = ayRaw * ACCEL_SCALE_MS2;
  d.az_ms2 = azRaw * ACCEL_SCALE_MS2;
  d.gx     = gxRaw * GYRO_SCALE;
  d.gy     = gyRaw * GYRO_SCALE;
  d.gz     = gzRaw * GYRO_SCALE;
  return d;
}

bool mpuInit() {
  uint8_t who = mpuReadReg(MPU_WHO_AM_I);
  Serial.print("[MPU] WHO_AM_I = 0x");
  Serial.println(who, HEX);
  if (who != 0x68 && who != 0x70 && who != 0x72)
    Serial.println("[MPU] Tanimsiz WHO_AM_I — devam ediliyor");

  mpuWriteReg(MPU_PWR_MGMT_1,   0x00);
  delay(100);
  mpuWriteReg(MPU_SMPLRT_DIV,   0x07);
  mpuWriteReg(MPU_CONFIG_REG,   0x00);
  mpuWriteReg(MPU_GYRO_CONFIG,  0x00); // ±250 °/s
  mpuWriteReg(MPU_ACCEL_CONFIG, 0x00); // ±2 g

  uint8_t pwr = mpuReadReg(MPU_PWR_MGMT_1);
  Serial.print("[MPU] PWR_MGMT_1 readback = 0x");
  Serial.println(pwr, HEX);
  return (pwr == 0x00);
}

// ============================================================
//  BME280 altitude helpers
// ============================================================
float bmeRawAltitude() {
  return bme.readAltitude(SEA_LEVEL_HPA);
}

void calibratePadAltitude() {
  Serial.println("[BME] Kalibrasyon basliyor (10 okuma)...");
  float sum = 0.0f;
  for (int i = 0; i < 10; i++) {
    sum += bmeRawAltitude();
    delay(50);
  }
  pad_altitude_ref = sum / 10.0f;
  Serial.print("[BME] Pad referans yuksekligi: ");
  Serial.print(pad_altitude_ref, 2);
  Serial.println(" m");
}

float getRelativeAltitude() {
  return bmeRawAltitude() - pad_altitude_ref;
}

// ============================================================
//  E22 helpers
// ============================================================
void waitAUX() {
  unsigned long t = millis();
  while (digitalRead(PIN_AUX) == LOW) {
    if (millis() - t > 3000) break;
  }
  delay(2);
}

void setMode(bool m1, bool m0) {
  digitalWrite(PIN_M1, m1 ? HIGH : LOW);
  digitalWrite(PIN_M0, m0 ? HIGH : LOW);
  delay(20);
  waitAUX();
}

bool sendConfig(const byte* cmd, int len) {
  Serial1.write(cmd, len);
  delay(50);
  byte resp[16];
  int n = 0;
  unsigned long t = millis();
  while (n < len && millis() - t < 500) {
    if (Serial1.available()) resp[n++] = Serial1.read();
  }
  if (n < len) return false;
  return (resp[0] == 0xC1);
}

void configureModule() {
  Serial.println("[CFG] Entering config mode...");
  setMode(true, false);
  delay(100);
  while (Serial1.available()) Serial1.read();

  byte cmd1[] = {0xC0, 0x00, 0x04, 0x00, 0x00, 0x00, 0x62};
  Serial.println(sendConfig(cmd1, sizeof(cmd1))
                 ? "[CFG] Address/REG0 OK" : "[CFG] Address/REG0 FAIL");

  byte cmd2[] = {0xC0, 0x04, 0x01, 0x00};
  Serial.println(sendConfig(cmd2, sizeof(cmd2))
                 ? "[CFG] REG1 OK" : "[CFG] REG1 FAIL");

  byte cmd3[] = {0xC0, 0x05, 0x01, 0x07};
  Serial.println(sendConfig(cmd3, sizeof(cmd3))
                 ? "[CFG] Channel OK" : "[CFG] Channel FAIL");

  byte cmd4[] = {0xC0, 0x06, 0x01, 0x4B};
  Serial.println(sendConfig(cmd4, sizeof(cmd4))
                 ? "[CFG] REG3 OK" : "[CFG] REG3 FAIL");

  Serial.println("[CFG] Done. Switching to Normal mode.");
  setMode(false, false);
  delay(100);
}

// ============================================================
//  Setup / Loop
// ============================================================
void setup() {
  pinMode(PIN_M0,  OUTPUT);
  pinMode(PIN_M1,  OUTPUT);
  pinMode(PIN_AUX, INPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(9600);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  unsigned long t = millis();
  while (!Serial && millis() - t < 3000);

  Serial.println("=== E22-900T30D TX / HYI PROTOKOLU ===");

  if (mpuInit())
    Serial.println("[MPU] Init OK");
  else
    Serial.println("[MPU] Init sorunlu — devam ediliyor");

  if (bme.begin(BME_ADDR, &Wire)) {
    Serial.println("[BME] Init OK");
  } else {
    Serial.println("[BME] BULUNAMADI — baglantıyi kontrol et!");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(200); }
  }

  calibratePadAltitude();
  configureModule();

  Serial.println("[TX] Hazir. HYI paketi gonderiliyor...");
}

void loop() {
  digitalWrite(LED_PIN, digitalRead(PIN_AUX));

  // ---- Read sensors ----------------------------------------
  ImuData imu       = mpuRead();
  float altitude    = getRelativeAltitude();  // pad-referenced, metres
  float temperature = bme.readTemperature();
  float humidity    = bme.readHumidity();
  float pressure    = bme.readPressure() / 100.0f;

  // ---- Tilt angle from vertical ----------------------------
  // angle = acos(az / |a|), clamped to 0–180°
  float aMag  = sqrt(imu.ax_g * imu.ax_g +
                     imu.ay_g * imu.ay_g +
                     imu.az_g * imu.az_g);
  float angle = (aMag > 0.0f)
                ? degrees(acos(constrain(imu.az_g / aMag, -1.0f, 1.0f)))
                : 0.0f;

  // ---- Build 78-byte HYİ packet ----------------------------
  uint8_t packet[PACKET_SIZE];
  memset(packet, 0x00, PACKET_SIZE);

  // Header (bytes 0–3, 1-indexed 1–4)
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0x54;
  packet[3] = 0x52;

  // Team ID & counter (bytes 4–5, 1-indexed 5–6)
  packet[4] = TEAM_ID;
  packet[5] = packet_counter;

  // BME280 altitude pad-referenced (bytes 6–9, 1-indexed 7–10)
  insertFloat(packet, 6, altitude);

  // GPS fields (bytes 10–45, 1-indexed 11–46) — all 0x00 via memset

  // MPU6050 gyroscope °/s (bytes 46–57, 1-indexed 47–58)
  insertFloat(packet, 46, imu.gx);
  insertFloat(packet, 50, imu.gy);
  insertFloat(packet, 54, imu.gz);

  // MPU6050 accelerometer in g (bytes 58–69, 1-indexed 59–70)
  insertFloat(packet, 58, imu.ax_g);
  insertFloat(packet, 62, imu.ay_g);
  insertFloat(packet, 66, imu.az_g);

  // Tilt angle 0–180° (bytes 70–73, 1-indexed 71–74)
  insertFloat(packet, 70, angle);

  // Status byte — parachute placeholder (byte 74, 1-indexed 75)
  packet[74] = 0x00;

  // CheckSum (byte 75, 1-indexed 76) — always last before footer
  packet[75] = calcChecksum(packet);

  // Footer (bytes 76–77, 1-indexed 77–78)
  packet[76] = 0x0D;
  packet[77] = 0x0A;

  // ---- Transmit over LoRa ----------------------------------
  waitAUX();
  Serial1.write((byte)0x00); // Target ADDH
  Serial1.write((byte)0x02); // Target ADDL
  Serial1.write((byte)0x07); // Target channel
  Serial1.write(packet, PACKET_SIZE);
  waitAUX();

  // ---- Debug print (human-readable) ------------------------
  Serial.print("[TX] #"); Serial.print(packet_counter);
  Serial.print(" | Alt:");   Serial.print(altitude,   2);
  Serial.print("m Ang:");    Serial.print(angle,       1);
  Serial.print("deg T:");    Serial.print(temperature, 1);
  Serial.print("C P:");      Serial.print(pressure,    1);
  Serial.print("hPa H:");    Serial.print(humidity,    1);
  Serial.print("% | ax:");   Serial.print(imu.ax_ms2,  2);
  Serial.print(" ay:");      Serial.print(imu.ay_ms2,  2);
  Serial.print(" az:");      Serial.print(imu.az_ms2,  2);
  Serial.print("m/s2 | gx:"); Serial.print(imu.gx,    2);
  Serial.print(" gy:");       Serial.print(imu.gy,     2);
  Serial.print(" gz:");       Serial.print(imu.gz,     2);
  Serial.print("dps | CS:0x");
  Serial.println(packet[75], HEX);

  packet_counter++;  // uint8_t wraps 255→0 automatically

  delay(100);  // HYI minimum inter-packet interval
}
