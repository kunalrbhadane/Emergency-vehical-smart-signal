/*
  ESP32 Traffic Light Controller + NFC Emergency + Servo
  -------------------------------------------------------
  Features:
    - Normal traffic light cycle with 4 directions
    - Emergency override via NFC (green light for 30s)
    - Servo control: closes when emergency active, opens otherwise
    - Updated LED mapping for NeoPixel LEDs
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PN532.h>
#include <ESP32Servo.h>

/* ================= CONFIGURATION ================= */
#define NUM_DIRECTIONS 4
#define LEDS_PER_DIRECTION 2
#define LED_COUNT 8

#define LED_PIN 5
#define I2C_SDA 21
#define I2C_SCL 22
#define PN532_IRQ_PIN 4
#define PN532_RESET_PIN 2

#define NORMAL_GREEN_MS 20000UL
#define NORMAL_YELLOW_MS 5000UL

#define EMERGENCY_GREEN_MS 30000UL
#define EMERGENCY_OTHER_YELLOW_MS 3000UL

#define UNKNOWN_UID_PRINT_DEBOUNCE_MS 1000UL
#define PN532_READ_TIMEOUT 250UL

/* Servo config */
#define SERVO_PIN 15
Servo gateServo;
#define SERVO_OPEN_ANGLE 90
#define SERVO_CLOSED_ANGLE 0

/* Calculate normal red duration for each direction */
const unsigned long NORMAL_RED_MS = NUM_DIRECTIONS * (NORMAL_GREEN_MS + NORMAL_YELLOW_MS) - (NORMAL_GREEN_MS + NORMAL_YELLOW_MS);

/* ================= STATIC CHECK ==================== */
#if ((NUM_DIRECTIONS * LEDS_PER_DIRECTION) != LED_COUNT)
  #error "LED_COUNT must equal NUM_DIRECTIONS * LEDS_PER_DIRECTION"
#endif

/* ================= GLOBALS ======================== */
static Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
static Adafruit_PN532 nfc = Adafruit_PN532(PN532_IRQ_PIN, PN532_RESET_PIN);

static uint32_t COLOR_RED;
static uint32_t COLOR_YELLOW;
static uint32_t COLOR_GREEN;
static uint32_t COLOR_OFF = 0;

/* New direction-to-pixel mapping */
static const uint8_t directionPixelMap[NUM_DIRECTIONS][LEDS_PER_DIRECTION] = {
  {1, 4}, // direction 0
  {3, 6}, // direction 1
  {0, 5}, // direction 2
  {2, 7}  // direction 3
};

/* NFC UID mapping structure */
typedef struct {
  const char* uidHex;
  uint8_t directionIndex;
} UidMap;

static UidMap nfcUidToDirectionMap[] = {
  { "B9477C05", 0 },
  { "04B134C2D50123", 1 },
  { "04C1A2B3D41234", 2 },
  { "0499AA11BB2233", 3 }
};
static const uint8_t NFCTAG_MAP_COUNT = sizeof(nfcUidToDirectionMap) / sizeof(nfcUidToDirectionMap[0]);

/* State machines */
typedef enum { NORM_GREEN, NORM_YELLOW } NormalState;
static NormalState normalState = NORM_GREEN;
static uint8_t normalCurrentDirection = 0;
static unsigned long normalStateStartMillis = 0UL;

static bool emergencyActive = false;
static String currentEmergencyUid = String("");
static int emergencyDirection = -1;
static unsigned long emergencyStartMillis = 0UL;

static unsigned long lastUnknownPrintMillis = 0UL;

/* ============== FUNCTION DECLARATIONS ============== */
static String uidBytesToHexString(uint8_t* uid, uint8_t uidLength);
static int findDirectionForUid(const String uidHex);
static void setDirectionColor(uint8_t dir, uint32_t color);
static void setAllDirectionsTo(uint32_t color);
static void updateStrip(void);
static void beginEmergency(const String uidHex, int dirIndex);
static void endEmergencyAndResume(void);
static void handleNormalStateMachine(void);
static void handleEmergencyStateMachine(void);
static void showInitialState(void);

/* ==================== SETUP ======================== */
void setup(void) {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== ESP32 Traffic Light Controller + Servo ===");
  Serial.print("Computed approximate RED duration per direction: ");
  Serial.print(NORMAL_RED_MS); Serial.print(" ms ("); Serial.print(NORMAL_RED_MS/1000); Serial.println(" s)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.print("I2C started on SDA="); Serial.print(I2C_SDA); Serial.print(" SCL="); Serial.println(I2C_SCL);

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("Didn't find PN532 board; check wiring!");
  } else {
    Serial.print("Found PN532; firmware version: 0x"); Serial.println(versiondata, HEX);
    nfc.SAMConfig();
  }

  strip.begin();
  strip.show();
  COLOR_RED = strip.Color(255,0,0);
  COLOR_YELLOW = strip.Color(180,180,0);
  COLOR_GREEN = strip.Color(0,200,0);

  gateServo.attach(SERVO_PIN);
  gateServo.write(SERVO_OPEN_ANGLE);
  Serial.println("Servo initialized and set to OPEN");

  showInitialState();

  normalState = NORM_GREEN;
  normalCurrentDirection = 0;
  normalStateStartMillis = millis();
  Serial.println("Starting normal cycle.");
}

/* ===================== LOOP ======================== */
void loop(void) {
  uint8_t uid[7];
  uint8_t uidLength = 0;

  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, PN532_READ_TIMEOUT);
  if (success && uidLength > 0) {
    String uidHex = uidBytesToHexString(uid, uidLength);
    Serial.print("NFC tag detected: "); Serial.println(uidHex);

    int dir = findDirectionForUid(uidHex);
    if (dir >= 0 && dir < NUM_DIRECTIONS) {
      if (emergencyActive && uidHex == currentEmergencyUid) {
        Serial.println("Duplicate emergency tag during active emergency - ignored.");
      } else {
        Serial.print("Emergency tag mapped to direction "); Serial.println(dir);
        beginEmergency(uidHex, dir);
      }
    } else {
      unsigned long now = millis();
      if (now - lastUnknownPrintMillis > UNKNOWN_UID_PRINT_DEBOUNCE_MS) {
        Serial.print("Unknown tag UID: "); Serial.println(uidHex);
        lastUnknownPrintMillis = now;
      }
    }
  }

  if (emergencyActive) handleEmergencyStateMachine();
  else handleNormalStateMachine();

  delay(10);
}

/* ============= STATE MACHINE LOGIC ================= */
static void handleNormalStateMachine(void) {
  unsigned long now = millis();
  unsigned long elapsed = now - normalStateStartMillis;

  if (normalState == NORM_GREEN) {
    if (elapsed == 0) {
      Serial.print("Normal: GREEN for direction "); Serial.println(normalCurrentDirection);
      for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) {
        if (d == normalCurrentDirection) setDirectionColor(d, COLOR_GREEN);
        else setDirectionColor(d, COLOR_RED);
      }
      updateStrip();
    }
    if (elapsed >= NORMAL_GREEN_MS) {
      normalState = NORM_YELLOW;
      normalStateStartMillis = now;
      Serial.print("Normal: GREEN->YELLOW for direction "); Serial.println(normalCurrentDirection);
      setDirectionColor(normalCurrentDirection, COLOR_YELLOW);
      updateStrip();
    }
  } else if (normalState == NORM_YELLOW) {
    if (elapsed >= NORMAL_YELLOW_MS) {
      normalCurrentDirection = (normalCurrentDirection + 1) % NUM_DIRECTIONS;
      normalState = NORM_GREEN;
      normalStateStartMillis = now;
      Serial.print("Normal: Next GREEN direction "); Serial.println(normalCurrentDirection);
      for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) {
        if (d == normalCurrentDirection) setDirectionColor(d, COLOR_GREEN);
        else setDirectionColor(d, COLOR_RED);
      }
      updateStrip();
    }
  }
}

static void handleEmergencyStateMachine(void) {
  unsigned long now = millis();
  unsigned long elapsed = now - emergencyStartMillis;

  if (elapsed <= EMERGENCY_GREEN_MS) {
    gateServo.write(SERVO_CLOSED_ANGLE); // close servo during emergency

    for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) {
      if ((int)d == emergencyDirection) setDirectionColor(d, COLOR_GREEN);
      else {
        if (elapsed < EMERGENCY_OTHER_YELLOW_MS) setDirectionColor(d, COLOR_YELLOW);
        else setDirectionColor(d, COLOR_RED);
      }
    }
    updateStrip();
  } else {
    gateServo.write(SERVO_OPEN_ANGLE); // open servo after emergency

    Serial.print("Emergency for direction "); Serial.print(emergencyDirection);
    Serial.print(" (UID "); Serial.print(currentEmergencyUid); Serial.println(") completed. Resuming normal cycle.");
    endEmergencyAndResume();
  }
}

/* ============= EMERGENCY CONTROL =================== */
static void beginEmergency(const String uidHex, int dirIndex) {
  emergencyActive = true;
  currentEmergencyUid = uidHex;
  emergencyDirection = dirIndex;
  emergencyStartMillis = millis();

  Serial.print("BEGIN EMERGENCY: direction "); Serial.print(emergencyDirection);
  Serial.print(" uid="); Serial.println(currentEmergencyUid);

  for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) {
    if ((int)d == emergencyDirection) setDirectionColor(d, COLOR_GREEN);
    else setDirectionColor(d, COLOR_YELLOW);
  }
  updateStrip();
}

static void endEmergencyAndResume(void) {
  emergencyActive = false;
  currentEmergencyUid = String("");
  int nextDir = (emergencyDirection + 1) % NUM_DIRECTIONS;
  emergencyDirection = -1;

  normalCurrentDirection = (uint8_t)nextDir;
  normalState = NORM_GREEN;
  normalStateStartMillis = millis();

  Serial.print("Resuming normal cycle at direction "); Serial.println(normalCurrentDirection);

  for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) {
    if (d == normalCurrentDirection) setDirectionColor(d, COLOR_GREEN);
    else setDirectionColor(d, COLOR_RED);
  }
  updateStrip();
}

/* ============= LED & UTILITIES ===================== */
static void setDirectionColor(uint8_t dir, uint32_t color) {
  for (uint8_t i = 0; i < LEDS_PER_DIRECTION; ++i) {
    uint8_t idx = directionPixelMap[dir][i];
    if (idx < LED_COUNT) strip.setPixelColor(idx, color);
  }
}

static void setAllDirectionsTo(uint32_t color) {
  for (uint16_t i = 0; i < LED_COUNT; ++i) strip.setPixelColor(i, color);
}

static void updateStrip(void) {
  strip.show();
}

static String uidBytesToHexString(uint8_t* uid, uint8_t uidLength) {
  String s = "";
  for (uint8_t i = 0; i < uidLength; ++i) {
    uint8_t b = uid[i];
    if (b < 16) s += "0";
    s += String(b, HEX);
  }
  s.toUpperCase();
  return s;
}

static int findDirectionForUid(const String uidHex) {
  for (uint8_t i = 0; i < NFCTAG_MAP_COUNT; ++i) {
    if (uidHex.equalsIgnoreCase(nfcUidToDirectionMap[i].uidHex)) return nfcUidToDirectionMap[i].directionIndex;
  }
  return -1;
}

static void showInitialState(void) {
  setAllDirectionsTo(COLOR_OFF);
  for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) setDirectionColor(d, COLOR_RED);
  updateStrip();
  delay(200);
  for (int t = 0; t < 2; ++t) {
    setAllDirectionsTo(COLOR_OFF); updateStrip(); delay(120);
    for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) setDirectionColor(d, COLOR_YELLOW);
    updateStrip(); delay(120);
  }
  for (uint8_t d = 0; d < NUM_DIRECTIONS; ++d) setDirectionColor(d, COLOR_RED);
  updateStrip();
}

/* End of file */