
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

namespace {

// ============================================================
// Pipkin REV.A pin mapping
// XIAO nRF54L15
// ============================================================

// SX1281 radio
constexpr uint8_t RADIO_NSS_PIN   = PIN_D4;
constexpr uint8_t RADIO_DIO1_PIN  = PIN_D5;
constexpr uint8_t RADIO_RESET_PIN = PIN_D2;
constexpr uint8_t RADIO_BUSY_PIN  = PIN_D3;

// External RF switch
constexpr uint8_t RADIO_RX_EN_PIN = PIN_D0;
constexpr uint8_t RADIO_TX_EN_PIN = PIN_D1;

// Hardware SPI
constexpr uint8_t RADIO_SCK_PIN  = PIN_SPI_SCK;
constexpr uint8_t RADIO_MISO_PIN = PIN_SPI_MISO;
constexpr uint8_t RADIO_MOSI_PIN = PIN_SPI_MOSI;

// J4 external I2C bus, using Wire1
// SDA = D12, SCL = D11
constexpr uint8_t I2C_SDA_PIN = PIN_D12;
constexpr uint8_t I2C_SCL_PIN = PIN_D11;

// ============================================================
// SX1281 command definitions
// ============================================================

constexpr uint8_t CMD_GET_STATUS      = 0xC0;
constexpr uint8_t CMD_SET_STANDBY     = 0x80;
constexpr uint8_t CMD_SET_PACKET_TYPE = 0x8A;
constexpr uint8_t CMD_GET_PACKET_TYPE = 0x03;

constexpr uint8_t STDBY_RC = 0x00;
constexpr uint8_t PACKET_TYPE_LORA = 0x01;

// ============================================================
// Configuration
// ============================================================

constexpr uint32_t RADIO_BUSY_TIMEOUT_MS = 250;
constexpr uint32_t RADIO_POLL_INTERVAL_MS = 2000;
constexpr uint32_t I2C_SCAN_INTERVAL_MS = 10000;
constexpr uint32_t LED_BLINK_INTERVAL_MS = 500;

// Enable to hold NSS LOW/HIGH for multimeter testing.
// Each state lasts five seconds.
constexpr bool RUN_NSS_HOLD_TEST = false;

const SPISettings radioSpiSettings(
    1000000,
    MSBFIRST,
    SPI_MODE0
);

bool statusLedOn = false;
uint32_t lastLedToggle = 0;
uint32_t lastRadioPoll = 0;
uint32_t lastI2cScan = 0;

// ============================================================
// Utility functions
// ============================================================

void printHex(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

const char* busyState() {
  return digitalRead(RADIO_BUSY_PIN) == HIGH
             ? "HIGH"
             : "LOW";
}

bool waitForRadioReady(
    uint32_t timeoutMs = RADIO_BUSY_TIMEOUT_MS
) {
  const uint32_t started = millis();

  while (digitalRead(RADIO_BUSY_PIN) == HIGH) {
    if (millis() - started >= timeoutMs) {
      return false;
    }
    delay(1);
  }

  return true;
}

// ============================================================
// SX1281 status decoding
// ============================================================

void printRadioStatus(uint8_t status) {
  Serial.print("[SX1281] Status: 0x");
  printHex(status);
  Serial.println();

  const uint8_t mode = (status >> 5) & 0x07;
  const uint8_t commandStatus = (status >> 2) & 0x07;

  Serial.print("[SX1281] Circuit mode: ");

  switch (mode) {
    case 2:
      Serial.println("STDBY_RC");
      break;
    case 3:
      Serial.println("STDBY_XOSC");
      break;
    case 4:
      Serial.println("FS");
      break;
    case 5:
      Serial.println("RX");
      break;
    case 6:
      Serial.println("TX");
      break;
    default:
      Serial.println("RESERVED / INVALID");
      break;
  }

  Serial.print("[SX1281] Command status: ");

  switch (commandStatus) {
    case 1:
      Serial.println("Success");
      break;
    case 2:
      Serial.println("Data available");
      break;
    case 3:
      Serial.println("Command timeout");
      break;
    case 4:
      Serial.println("Processing error");
      break;
    case 5:
      Serial.println("Execution failure");
      break;
    case 6:
      Serial.println("TX completed");
      break;
    default:
      Serial.println("RESERVED / INVALID");
      break;
  }

  if (status == 0x00 || status == 0xFF) {
    Serial.println(
        "[SX1281] WARNING: Suspicious SPI response!"
    );
  }
}

// ============================================================
// NSS diagnostic
// ============================================================

void testNssPin() {
  Serial.println();
  Serial.println("[TEST] NSS LOW for 5 seconds");

  digitalWrite(RADIO_NSS_PIN, LOW);
  delay(5000);

  Serial.println("[TEST] NSS HIGH for 5 seconds");

  digitalWrite(RADIO_NSS_PIN, HIGH);
  delay(5000);
}

// ============================================================
// SX1281 hardware reset
// ============================================================

bool resetRadio() {
  Serial.println();
  Serial.println("[SX1281] Hardware reset...");

  digitalWrite(RADIO_NSS_PIN, HIGH);

  digitalWrite(RADIO_RESET_PIN, HIGH);
  delay(1);

  digitalWrite(RADIO_RESET_PIN, LOW);
  delay(2);

  digitalWrite(RADIO_RESET_PIN, HIGH);
  delay(10);

  Serial.print("[SX1281] BUSY after reset: ");
  Serial.println(busyState());

  if (!waitForRadioReady(500)) {
    Serial.println(
        "[SX1281] FAIL: BUSY stuck HIGH after reset"
    );
    return false;
  }

  Serial.println(
      "[SX1281] Reset complete; BUSY LOW"
  );

  return true;
}

// ============================================================
// SX1281 SPI write command
// ============================================================

bool writeRadioCommand(
    const char* name,
    uint8_t opcode,
    uint8_t parameter
) {
  Serial.println();
  Serial.print("[SX1281] Sending ");
  Serial.println(name);

  if (!waitForRadioReady()) {
    Serial.println(
        "[SX1281] FAIL: BUSY HIGH before command"
    );
    return false;
  }

  SPI.beginTransaction(radioSpiSettings);

  digitalWrite(RADIO_NSS_PIN, LOW);

  const uint8_t rx0 = SPI.transfer(opcode);
  const uint8_t rx1 = SPI.transfer(parameter);

  digitalWrite(RADIO_NSS_PIN, HIGH);

  SPI.endTransaction();

  Serial.print("[SX1281] MOSI: ");
  printHex(opcode);
  Serial.print(" ");
  printHex(parameter);
  Serial.println();

  Serial.print("[SX1281] MISO: ");
  printHex(rx0);
  Serial.print(" ");
  printHex(rx1);
  Serial.println();

  if (!waitForRadioReady()) {
    Serial.print("[SX1281] FAIL: ");
    Serial.print(name);
    Serial.println(" BUSY timeout");
    return false;
  }

  Serial.print("[SX1281] ");
  Serial.print(name);
  Serial.println(" completed; BUSY LOW");

  return true;
}

// ============================================================
// SX1281 GetStatus
// ============================================================

bool readRadioStatus(uint8_t &status) {
  status = 0xFF;

  if (!waitForRadioReady()) {
    Serial.println(
        "[SX1281] GetStatus: BUSY HIGH before SPI"
    );
    return false;
  }

  SPI.beginTransaction(radioSpiSettings);

  digitalWrite(RADIO_NSS_PIN, LOW);

  // GetStatus is a single-byte SPI transaction.
  status = SPI.transfer(CMD_GET_STATUS);

  digitalWrite(RADIO_NSS_PIN, HIGH);

  SPI.endTransaction();

  Serial.print("[SX1281] GetStatus MISO: 0x");
  printHex(status);
  Serial.println();

  if (!waitForRadioReady()) {
    Serial.println(
        "[SX1281] GetStatus: BUSY timeout after SPI"
    );
    return false;
  }

  return true;
}

// ============================================================
// SX1281 GetPacketType
// ============================================================

bool readRadioPacketType(
    uint8_t &status,
    uint8_t &packetType
) {
  status = 0xFF;
  packetType = 0xFF;

  if (!waitForRadioReady()) {
    Serial.println(
        "[SX1281] GetPacketType: BUSY HIGH before SPI"
    );
    return false;
  }

  SPI.beginTransaction(radioSpiSettings);

  digitalWrite(RADIO_NSS_PIN, LOW);

  const uint8_t rx0 =
      SPI.transfer(CMD_GET_PACKET_TYPE);

  status = SPI.transfer(0x00);
  packetType = SPI.transfer(0x00);

  digitalWrite(RADIO_NSS_PIN, HIGH);

  SPI.endTransaction();

  Serial.print("[SX1281] GetPacketType MISO: ");
  printHex(rx0);
  Serial.print(" ");
  printHex(status);
  Serial.print(" ");
  printHex(packetType);
  Serial.println();

  if (!waitForRadioReady()) {
    Serial.println(
        "[SX1281] GetPacketType: BUSY timeout after SPI"
    );
    return false;
  }

  return true;
}

// ============================================================
// Complete SX1281 diagnostic
// ============================================================

void testRadio() {
  Serial.println();
  Serial.println("========================================");
  Serial.println(" SX1281 DIAGNOSTIC TEST");
  Serial.println("========================================");

  // Step 1: Reset
  if (!resetRadio()) {
    Serial.println("[SX1281] TEST FAILED: Reset");
    return;
  }

  // Step 2: First command - enter standby
  if (!writeRadioCommand(
          "SetStandby(STDBY_RC)",
          CMD_SET_STANDBY,
          STDBY_RC
      )) {
    Serial.println(
        "[SX1281] TEST FAILED: SetStandby"
    );
    return;
  }

  // Step 3: Read status
  uint8_t status = 0xFF;

  if (!readRadioStatus(status)) {
    Serial.println(
        "[SX1281] TEST FAILED: GetStatus"
    );
    return;
  }

  printRadioStatus(status);

  // Step 4: Set packet type to LoRa
  if (!writeRadioCommand(
          "SetPacketType(LoRa)",
          CMD_SET_PACKET_TYPE,
          PACKET_TYPE_LORA
      )) {
    Serial.println(
        "[SX1281] TEST FAILED: SetPacketType"
    );
    return;
  }

  // Step 5: Verify packet type
  uint8_t packetType = 0xFF;

  if (!readRadioPacketType(status, packetType)) {
    Serial.println(
        "[SX1281] TEST FAILED: GetPacketType"
    );
    return;
  }

  Serial.print("[SX1281] Packet type: 0x");
  printHex(packetType);
  Serial.println();

  if (packetType == PACKET_TYPE_LORA) {
    Serial.println();
    Serial.println(
        "[SX1281] PASS: LoRa packet type verified!"
    );
  } else {
    Serial.println();
    Serial.println(
        "[SX1281] FAIL: Packet type mismatch"
    );
  }

  Serial.println("========================================");
}

// ============================================================
// Periodic SX1281 polling
// ============================================================

void pollRadio() {
  Serial.println();
  Serial.println("[SX1281] Periodic poll");

  Serial.print("[SX1281] BUSY: ");
  Serial.println(busyState());

  if (!waitForRadioReady()) {
    Serial.println(
        "[SX1281] Radio still BUSY; skipping SPI"
    );
    return;
  }

  uint8_t status = 0xFF;
  uint8_t packetType = 0xFF;

  if (readRadioStatus(status)) {
    printRadioStatus(status);
  } else {
    return;
  }

  if (readRadioPacketType(status, packetType)) {
    Serial.print("[SX1281] Packet type: 0x");
    printHex(packetType);

    if (packetType == PACKET_TYPE_LORA) {
      Serial.println(" PASS");
    } else {
      Serial.println(" FAIL");
    }
  }
}

// ============================================================
// External I2C scanner
//
// IMPORTANT:
// Use Wire1, not Wire.
//
// Wire  = D4/D5 (conflicts with radio)
// Wire1 = D12/D11 (J4)
// ============================================================

void scanI2cBus() {
  Serial.println();
  Serial.println(
      "[I2C] Scanning addresses 0x08-0x77 on J4..."
  );

  uint8_t found = 0;

  for (uint8_t address = 0x08;
       address <= 0x77;
       ++address) {

    Wire1.beginTransmission(address);

    const uint8_t error =
        Wire1.endTransmission();

    if (error == 0) {
      Serial.print("[I2C] ACK at 0x");
      printHex(address);
      Serial.println();

      ++found;
    }
  }

  if (found == 0) {
    Serial.println(
        "[I2C] No devices acknowledged."
    );
  } else {
    Serial.print("[I2C] Found ");
    Serial.print(found);
    Serial.println(" device(s).");
  }
}

// ============================================================
// Hardware initialization
// ============================================================

void initializeHardware() {
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Disable external RF switch
  // This diagnostic never transmits.
  pinMode(RADIO_RX_EN_PIN, OUTPUT);
  pinMode(RADIO_TX_EN_PIN, OUTPUT);

  digitalWrite(RADIO_RX_EN_PIN, LOW);
  digitalWrite(RADIO_TX_EN_PIN, LOW);

  // Radio GPIO
  pinMode(RADIO_NSS_PIN, OUTPUT);
  digitalWrite(RADIO_NSS_PIN, HIGH);

  pinMode(RADIO_RESET_PIN, OUTPUT);
  digitalWrite(RADIO_RESET_PIN, HIGH);

  pinMode(RADIO_BUSY_PIN, INPUT);
  pinMode(RADIO_DIO1_PIN, INPUT);

  // SPI
  SPI.setPins(
      RADIO_SCK_PIN,
      RADIO_MISO_PIN,
      RADIO_MOSI_PIN,
      -1
  );

  SPI.begin();

  // Preserve manual NSS control
  pinMode(RADIO_NSS_PIN, OUTPUT);
  digitalWrite(RADIO_NSS_PIN, HIGH);

  // ==========================================================
  // I2C FIX
  //
  // Wire1 is the second hardware I2C instance.
  // It uses D12 (SDA) and D11 (SCL).
  //
  // Do not initialize Wire here!
  // ==========================================================

  Wire1.begin();
  Wire1.setClock(100000);

  Serial.println("[SYSTEM] SPI initialized.");
  Serial.println(
      "[SYSTEM] Wire1 initialized on D12/D11."
  );
}

} // namespace

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" Pipkin REV.A Hardware Diagnostics");
  Serial.println(" SX1281 + External I2C");
  Serial.println("========================================");

  Serial.println(
      "I2C: Wire1, SDA=D12, SCL=D11"
  );
  Serial.println(
      "SX1281: NSS=D4, RESET=D2, BUSY=D3, DIO1=D5"
  );
  Serial.println(
      "SPI: 1 MHz, MODE0"
  );
  Serial.println(
      "RF transmission disabled."
  );

  initializeHardware();

  // Optional NSS multimeter test
  if (RUN_NSS_HOLD_TEST) {
    testNssPin();
  }

  // Check I2C first.
  // This is intentional: it verifies the radio
  // still works after Wire1 initialization.
  scanI2cBus();

  // Then verify radio operation.
  testRadio();

  lastLedToggle = millis();
  lastRadioPoll = millis();
  lastI2cScan = millis();

  Serial.println();
  Serial.println(
      "[SYSTEM] Type 'r' to reset and retest radio."
  );
  Serial.println(
      "[SYSTEM] Type 'i' to scan I2C."
  );
  Serial.println(
      "[SYSTEM] Type 'a' to test both."
  );
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  const uint32_t now = millis();

  // LED heartbeat
  if (now - lastLedToggle >= LED_BLINK_INTERVAL_MS) {
    lastLedToggle = now;
    statusLedOn = !statusLedOn;

    digitalWrite(
        LED_BUILTIN,
        statusLedOn ? LOW : HIGH
    );
  }

  // Serial commands
  while (Serial.available()) {
    const char command = Serial.read();

    switch (command) {
      case 'r':
      case 'R':
        testRadio();
        lastRadioPoll = millis();
        break;

      case 'i':
      case 'I':
        scanI2cBus();
        lastI2cScan = millis();
        break;

      case 'a':
      case 'A':
        scanI2cBus();
        testRadio();
        lastRadioPoll = millis();
        lastI2cScan = millis();
        break;

      default:
        break;
    }
  }

  // Periodic radio status
  if (now - lastRadioPoll >= RADIO_POLL_INTERVAL_MS) {
    lastRadioPoll = now;
    pollRadio();
  }

  // Periodic J4 scan
  if (now - lastI2cScan >= I2C_SCAN_INTERVAL_MS) {
    lastI2cScan = now;
    scanI2cBus();
  }

  delay(10);
}
