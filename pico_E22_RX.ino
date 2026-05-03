// ============================================================
// E22-900T30D  —  RECEIVER / GROUND STATION (Pico / arduino-pico)
// TEKNOFEST HYİ Protocol — 78-byte binary packet
//
// UART0 : GP0=TX  GP1=RX   (Serial1, 9600 8N1)  — E22 LoRa in
// UART1 : GP8=TX  GP9=RX   (Serial2, 19200 8N1) — HYİ output
// M0    : GP2  (output)
// M1    : GP3  (output)
// AUX   : GP15 (input)  → onboard LED (GP25)
// ============================================================

#include <math.h>

// ---- Pin definitions ---------------------------------------
#define PIN_M0    2
#define PIN_M1    3
#define PIN_AUX   15
#define LED_PIN   25

// ---- HYİ protocol ------------------------------------------
#define PACKET_SIZE 78

// ---- Float→bytes union (for decoding debug prints) ---------
typedef union {
  float   value;
  uint8_t bytes[4];
} FLOAT32_CONV;

float bytesToFloat(uint8_t* b) {
  FLOAT32_CONV c;
  c.bytes[0] = b[0];
  c.bytes[1] = b[1];
  c.bytes[2] = b[2];
  c.bytes[3] = b[3];
  return c.value;
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

  // Address 0x0002, channel 7, 9600 8N1, 2.4 kbps air rate
  byte cmd1[] = {0xC0, 0x00, 0x04, 0x00, 0x02, 0x00, 0x62};
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
//  HYİ packet validation
// ============================================================
uint8_t calcChecksum(uint8_t* pkt) {
  uint32_t sum = 0;
  for (int i = 4; i < 75; i++) sum += pkt[i];
  return (uint8_t)(sum % 256);
}

bool validatePacket(uint8_t* pkt) {
  // Header
  if (pkt[0] != 0xFF || pkt[1] != 0xFF ||
      pkt[2] != 0x54 || pkt[3] != 0x52) return false;
  // Footer
  if (pkt[76] != 0x0D || pkt[77] != 0x0A) return false;
  // Checksum
  if (pkt[75] != calcChecksum(pkt)) return false;
  return true;
}

void printPacketDebug(uint8_t* pkt) {
  Serial.print("[RX] #");    Serial.print(pkt[5]);
  Serial.print(" TeamID:0x"); Serial.print(pkt[4], HEX);

  float alt   = bytesToFloat(&pkt[6]);
  float gx    = bytesToFloat(&pkt[46]);
  float gy    = bytesToFloat(&pkt[50]);
  float gz    = bytesToFloat(&pkt[54]);
  float ax    = bytesToFloat(&pkt[58]);
  float ay    = bytesToFloat(&pkt[62]);
  float az    = bytesToFloat(&pkt[66]);
  float angle = bytesToFloat(&pkt[70]);

  Serial.print(" | Alt:");   Serial.print(alt,   2); Serial.print("m");
  Serial.print(" Ang:");     Serial.print(angle,  1); Serial.print("deg");
  Serial.print(" | ax:");    Serial.print(ax,     3);
  Serial.print(" ay:");      Serial.print(ay,     3);
  Serial.print(" az:");      Serial.print(az,     3); Serial.print("g");
  Serial.print(" | gx:");    Serial.print(gx,     2);
  Serial.print(" gy:");      Serial.print(gy,     2);
  Serial.print(" gz:");      Serial.print(gz,     2); Serial.print("dps");
  Serial.print(" | CS:0x");  Serial.println(pkt[75], HEX);
}

// ============================================================
//  Setup / Loop
// ============================================================
uint8_t rx_buffer[PACKET_SIZE];
uint8_t rx_index = 0;

void setup() {
  pinMode(PIN_M0,  OUTPUT);
  pinMode(PIN_M1,  OUTPUT);
  pinMode(PIN_AUX, INPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);         // USB CDC — debug monitor
  Serial1.begin(9600);          // E22 LoRa (GP0/GP1)

  Serial2.setTX(8);             // HYİ output
  Serial2.setRX(9);
  Serial2.begin(19200);         // 19200 8N1 as required by HYİ

  unsigned long t = millis();
  while (!Serial && millis() - t < 3000);

  Serial.println("=== E22-900T30D RX / HYI PROTOKOLU ===");
  Serial.println("[RX] Serial2 (GP8/GP9) -> HYI @ 19200 baud");

  configureModule();

  Serial.println("[RX] Dinlemede...");
}

void loop() {
  // AUX → onboard LED
  digitalWrite(LED_PIN, digitalRead(PIN_AUX));

  // ---- Byte-by-byte accumulation ---------------------------
  // Collect bytes until idle for 100 ms, then process buffer.
  // This handles the slow drip from the E22 at 2.4 kbps air rate.
  if (Serial1.available()) {
    unsigned long lastByte = millis();
    while (millis() - lastByte < 100) {
      while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();

        // Re-sync: if buffer is empty, only start on 0xFF (header[0])
        if (rx_index == 0 && b != 0xFF) continue;

        rx_buffer[rx_index++] = b;
        lastByte = millis();

        if (rx_index == PACKET_SIZE) {
          rx_index = 0;  // reset for next packet

          if (validatePacket(rx_buffer)) {
            // Forward full 78 bytes to HYİ software
            Serial2.write(rx_buffer, PACKET_SIZE);
            printPacketDebug(rx_buffer);
          } else {
            Serial.println("[RX] Gecersiz paket — header/footer/checksum hatasi");
          }
          break;
        }
      }
    }
  }
}
