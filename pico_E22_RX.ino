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

// ============================================================
// HUMAN-READABLE CONFIGURATION
// ============================================================
struct LoRaSettings {
  uint16_t address;     // Module Address (0 to 65535)
  uint8_t  netID;       // Network ID (0 to 255)
  uint8_t  channel;     // Frequency Channel (0 to 83)
  uint32_t baudRate;    // UART Baud: 9600, 19200, 115200, etc.
  uint8_t  airDataRate; // Air kbps: 2 (for 2.4k), 9 (for 9.6k), etc.
  uint8_t  txPower;     // Power dBm: 30, 27, 24, 21
  bool     fixedMode;   // true = Fixed Mode, false = Transparent
};

// Edit these values to quickly change your setup:
LoRaSettings myConfig = {
  .address     = 0x0002,   // Set to 0x0002 for the Receiver
  .netID       = 0x00,     // Network ID
  .channel     = 18,        // <--- CHANGE CHANNEL HERE
  .baudRate    = 115200,   // Match your Serial1.begin() speed
  .airDataRate = 2,        // 2.4kbps (standard for range)[cite: 1, 2]
  .txPower     = 30,       // Maximum power (30dBm)
  .fixedMode   = true      // Required for targeted transmission[cite: 1]
};

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
  Serial.println("[CFG] Applying Human-Readable Settings...");
  setMode(true, false); // Switch to Configuration Mode (M1=1, M0=0)
  delay(100);
  while (Serial1.available()) Serial1.read();

  // 1. Calculate Register Bytes
  uint8_t addh = (myConfig.address >> 8) & 0xFF;
  uint8_t addl = myConfig.address & 0xFF;

  // FIX BUG 1: Proper Baud Rate & Air Data Rate Mapping
  uint8_t reg0_baud;
  switch (myConfig.baudRate) {
    case 1200:   reg0_baud = 0x00; break;
    case 2400:   reg0_baud = 0x01; break;
    case 4800:   reg0_baud = 0x02; break;
    case 9600:   reg0_baud = 0x03; break;
    case 19200:  reg0_baud = 0x04; break;
    case 38400:  reg0_baud = 0x05; break;
    case 57600:  reg0_baud = 0x06; break;
    case 115200: reg0_baud = 0x07; break;
    default:     reg0_baud = 0x03; // Default 9600
  }

  uint8_t reg0_air;
  switch (myConfig.airDataRate) {
    case 0:  reg0_air = 0x00; break; // 0.3k
    case 1:  reg0_air = 0x01; break; // 1.2k
    case 2:  reg0_air = 0x02; break; // 2.4k
    case 4:  reg0_air = 0x03; break; // 4.8k
    case 9:  reg0_air = 0x04; break; // 9.6k
    case 19: reg0_air = 0x05; break; // 19.2k
    case 38: reg0_air = 0x06; break; // 38.4k
    case 62: reg0_air = 0x07; break; // 62.5k
    default: reg0_air = 0x02; // Default 2.4k
  }
  uint8_t reg0 = (reg0_baud << 5) | (0x00 << 3) | reg0_air; // 8N1 parity

  // Map Power (bits 1-0)
  uint8_t reg1_pwr = (myConfig.txPower == 30) ? 0x00 : 
                     (myConfig.txPower == 27) ? 0x01 :
                     (myConfig.txPower == 24) ? 0x02 : 0x03;
  uint8_t reg1 = (0x00 << 6) | reg1_pwr; // 240-byte packet size

  // Map Fixed Mode (bit 6)
  uint8_t reg3 = (myConfig.fixedMode ? 0x01 : 0x00) << 6 | 0x0B; // Default RSSI off, LBT off

  // 2. Send Commands
  byte cmd1[] = {0xC0, 0x00, 0x04, addh, addl, myConfig.netID, reg0};
  sendConfig(cmd1, sizeof(cmd1));

  byte cmd2[] = {0xC0, 0x04, 0x01, reg1};
  sendConfig(cmd2, sizeof(cmd2));

  byte cmd3[] = {0xC0, 0x05, 0x01, myConfig.channel};
  sendConfig(cmd3, sizeof(cmd3));

  byte cmd4[] = {0xC0, 0x06, 0x01, reg3};
  sendConfig(cmd4, sizeof(cmd4));

  setMode(false, false); // Return to Normal Mode (M1=0, M0=0)
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
  Serial1.begin(myConfig.baudRate); // Re-initialize Pico's UART to match the E22's new baud
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
