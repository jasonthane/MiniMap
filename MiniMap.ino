// SPDX-FileCopyrightText: 2022 Limor Fried for Adafruit Industries
// SPDX-License-Identifier: MIT

#include <Arduino.h>
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TestBed.h"
#include <Adafruit_BME280.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GPS.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

// Convert 24-bit RGB (0xRRGGBB) to 16-bit RGB565 at compile time
#define RGBHEX(hex) ((((hex) >> 8) & 0xF800) | (((hex) >> 5) & 0x07E0) | (((hex) >> 3) & 0x001F))

// ---------------- Hardware / Libraries ----------------

// GPS on hardware serial
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

// Set to true to echo raw GPS chars to Serial (very noisy)
#define GPSECHO true

Adafruit_BME280 bme;        // I2C
bool bmefound = false;
extern Adafruit_TestBed TB;

Adafruit_MAX17048 lipo;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);

// ---------------- UI State ----------------
bool isImperial = true;     // true = feet + mph, false = meters + m/s
bool isAltitude = true;
const uint16_t inactiveStatusColor = RGBHEX(0x535353);

#define METERS_TO_FEET 3.28084f
#define KNOTS_TO_MPS   0.514444f    // Exact: 1 knot = 1852 m / 3600 s
#define KNOTS_TO_MPH   1.15077945f  // 1 knot = 1.15077945 statute miles per hour

const char* unitMeters = " m";
const char* unitFeet   = " ft";
const char* unitMSec  = " m/s";
const char* unitMPH    = " mph";    // Imperial speed unit

// ---------------- GNSS state (simplified!) ----------------
static float lastGoodAlt_m   = 0.0f;
static float lastGoodSpd_mps = 0.0f;
static uint32_t lastGoodFixTime = 0;  // millis() when we last had a valid fix
static bool everHadFix = false;      // true once we get first fix

// Debug print (1 Hz)
static uint32_t lastDbgMs = 0;

// ---------------- Splash Screen ----------------

// 5x7 bitmap font for "MiniMap" - each byte is a column, LSB at top
const uint8_t PROGMEM letter_M[] = {0x7F, 0x02, 0x04, 0x02, 0x7F};  // M
const uint8_t PROGMEM letter_i[] = {0x00, 0x44, 0x7D, 0x40, 0x00};  // i
const uint8_t PROGMEM letter_n[] = {0x7C, 0x04, 0x04, 0x04, 0x78};  // n
const uint8_t PROGMEM letter_a[] = {0x20, 0x54, 0x54, 0x54, 0x78};  // a
const uint8_t PROGMEM letter_p[] = {0x7C, 0x14, 0x14, 0x14, 0x08};  // p

const uint8_t* const letters[] = {letter_M, letter_i, letter_n, letter_i, letter_M, letter_a, letter_p};
const int numLetters = 7;

void drawBlockLetter(const uint8_t* bitmap, int x, int y, int scale, uint16_t color) {
  for (int col = 0; col < 5; col++) {
    uint8_t colData = pgm_read_byte(&bitmap[col]);
    for (int row = 0; row < 7; row++) {
      if (colData & (1 << row)) {
        canvas.fillRect(x + col * scale, y + row * scale, scale - 1, scale - 1, color);
      }
    }
  }
}

// Check if a screen pixel is part of the MiniMap text
bool isPartOfLogo(int px, int py, int textX, int textY, int scale, int letterWidth, int gap) {
  // Check each letter
  int lx = textX;
  for (int i = 0; i < numLetters; i++) {
    // Is pixel within this letter's bounding box?
    if (px >= lx && px < lx + 5 * scale && py >= textY && py < textY + 7 * scale) {
      int col = (px - lx) / scale;
      int row = (py - textY) / scale;
      uint8_t colData = pgm_read_byte(&letters[i][col]);
      if (colData & (1 << row)) {
        return true;
      }
    }
    lx += letterWidth + gap;
  }
  return false;
}

// Get the color for a logo pixel based on which letter it's in
uint16_t getLogoColor(int px, int textX, int letterWidth, int gap, uint16_t* colors) {
  int lx = textX;
  for (int i = 0; i < numLetters; i++) {
    if (px >= lx && px < lx + 5 * letterWidth / 5) {
      return colors[i];
    }
    lx += letterWidth + gap;
  }
  return colors[0];
}

void showSplash() {
  const int blockSize = 6;
  const int gridW = 240 / blockSize;  // 40 columns
  const int gridH = 135 / blockSize;  // 22 rows
  
  // MiniMap text positioning (matching block grid)
  const int scale = blockSize;
  const int letterWidth = 5 * scale;
  const int gap = 3;
  const int totalWidth = numLetters * letterWidth + (numLetters - 1) * gap;
  const int totalHeight = 7 * scale;
  const int textX = (240 - totalWidth) / 2;
  const int textY = (135 - totalHeight) / 2;
  
  // Bright logo colors
  uint16_t logoColors[] = {
    RGBHEX(0x00DDFF), RGBHEX(0x00FFCC), RGBHEX(0x00FF88),
    RGBHEX(0x88FF00), RGBHEX(0xCCFF00), RGBHEX(0xFFCC00), RGBHEX(0xFF8800)
  };
  
  // Desaturated/darker background colors (same gradient, ~40% brightness)
  uint16_t bgColors[] = {
    RGBHEX(0x004455), RGBHEX(0x005544), RGBHEX(0x005533),
    RGBHEX(0x335500), RGBHEX(0x445500), RGBHEX(0x553300), RGBHEX(0x552200)
  };
  
  int maxDiag = gridW + gridH;
  
  // Phase 1: Diagonal sweep in - 0.25 seconds
  // Step by 8 diagonals per frame for speed
  for (int wave = 0; wave <= maxDiag + 10; wave += 8) {
    canvas.fillScreen(ST77XX_BLACK);
    
    for (int gy = 0; gy < gridH; gy++) {
      for (int gx = 0; gx < gridW; gx++) {
        int diag = gx + gy;
        if (diag <= wave) {
          int px = gx * blockSize;
          int py = gy * blockSize;
          
          int cx = px + blockSize/2;  // Center of block
          int cy = py + blockSize/2;
          bool isLogo = isPartOfLogo(cx, cy, textX, textY, scale, letterWidth, gap);
          
          // Background color based on x position (gradient across screen)
          int colorIdx = (gx * 7) / gridW;
          uint16_t color = isLogo ? getLogoColor(cx, textX, letterWidth, gap, logoColors) : bgColors[colorIdx];
          
          canvas.fillRect(px, py, blockSize - 1, blockSize - 1, color);
        }
      }
    }
    
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    delay(25);
  }
  
  // Phase 2: Background blocks fade out - 0.5 seconds
  for (int fade = 0; fade < 10; fade++) {
    canvas.fillScreen(ST77XX_BLACK);
    
    for (int gy = 0; gy < gridH; gy++) {
      for (int gx = 0; gx < gridW; gx++) {
        int px = gx * blockSize;
        int py = gy * blockSize;
        int cx = px + blockSize/2;
        int cy = py + blockSize/2;
        
        bool isLogo = isPartOfLogo(cx, cy, textX, textY, scale, letterWidth, gap);
        
        if (isLogo) {
          uint16_t color = getLogoColor(cx, textX, letterWidth, gap, logoColors);
          canvas.fillRect(px, py, blockSize - 1, blockSize - 1, color);
        } else if (random(10) > fade) {
          // Fade: randomly skip more blocks each frame
          int colorIdx = (gx * 7) / gridW;
          canvas.fillRect(px, py, blockSize - 1, blockSize - 1, bgColors[colorIdx]);
        }
      }
    }
    
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    delay(50);
  }
  
  // Brief final hold with just logo
  canvas.fillScreen(ST77XX_BLACK);
  for (int gy = 0; gy < gridH; gy++) {
    for (int gx = 0; gx < gridW; gx++) {
      int px = gx * blockSize;
      int py = gy * blockSize;
      int cx = px + blockSize/2;
      int cy = py + blockSize/2;
      if (isPartOfLogo(cx, cy, textX, textY, scale, letterWidth, gap)) {
        canvas.fillRect(px, py, blockSize - 1, blockSize - 1, getLogoColor(cx, textX, letterWidth, gap, logoColors));
      }
    }
  }
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  delay(500);
}

// ---------------- Misc ----------------

void setup() {
  Serial.begin(115200);
  delay(100);

  // GPS: start at 9600 (module default), configure sentences + rate, then bump to 115200
  GPSSerial.begin(9600);     // or begin(9600, SERIAL_8N1, RX, TX) if needed
  GPS.begin(9600);

  delay(100);

  GPS.sendCommand("$PMTK353,1,0,0,0,0*2A");  // GPS-only mode (talker ID = GP) - testing library

  // 5 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

  // Increase NMEA baud rate (PMTK251). Checksum for 115200 is 0x1F.
  GPS.sendCommand("$PMTK251,115200*1F");
  delay(200);

#if defined(ARDUINO_ARCH_ESP32)
  GPSSerial.updateBaudRate(115200);   // works on most ESP32 core versions
#else
  GPSSerial.begin(115200);
#endif

  // TestBed + display
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1;
  TB.begin();
  TB.setColor(WHITE);

  display.init(135, 240); // ST7789 240x135
  display.setRotation(3);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  canvas.setFont(&FreeSans12pt7b);
  canvas.setTextColor(ST77XX_WHITE);

  showSplash();
 
  // Battery gauge
  if (!lipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.print(F("Found MAX17048 with Chip ID: 0x"));
  Serial.println(lipo.getChipID(), HEX);

  // Optional BME280 probe (kept as-is)
  if (TB.scanI2CBus(0x77)) {
    Serial.println("BME280 address");
    unsigned status = bme.begin();
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    } else {
      Serial.println("BME280 found OK");
      bmefound = true;
    }
  }

  // Buttons
  pinMode(0, INPUT_PULLDOWN);
  pinMode(1, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);
}

static inline void serviceGPS() {
  while (GPSSerial.available()) {
    char c = GPS.read();
    if (GPSECHO && c) Serial.print(c);
    
    // Check for complete sentence after each character
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
    }
  }
}

static inline void updateGNSSFilters() {
  if (GPS.fix) {
    // We have a current valid fix

    if (isfinite(GPS.altitude)) {
      lastGoodAlt_m = GPS.altitude;
    }
    if (isfinite(GPS.speed)) {
      lastGoodSpd_mps = GPS.speed * KNOTS_TO_MPS;
    }
    lastGoodFixTime = millis();
    everHadFix = true;
  }
}

static inline void debugPrint1Hz() {
  uint32_t now = millis();
  if (now - lastDbgMs < 1000) return;
  lastDbgMs = now;

  Serial.print(F("fix=")); Serial.print(GPS.fix);
  Serial.print(F(" sats=")); Serial.print(GPS.satellites);
  Serial.print(F(" alt_m=")); Serial.print(GPS.altitude, 1);
  Serial.print(F(" spd_mps=")); Serial.print(GPS.speed * KNOTS_TO_MPS, 2);
  if (everHadFix) {
    uint32_t ago = (now - lastGoodFixTime)/1000;
    Serial.print(F(" last_fix=")); Serial.print(ago); Serial.print(F("s ago"));
  }
  Serial.println();
}

// Button state tracking for edge detection
bool lastBtn1 = false;
bool lastBtn2 = false;

uint32_t lastDisplayUpdate = 0;
const uint32_t DISPLAY_UPDATE_MS = 200;

void loop() {
  uint32_t now = millis();

  // Buttons with edge detection (trigger once per press)
  bool btn1 = digitalRead(1);
  bool btn2 = digitalRead(2);
  
  if (btn2 && !lastBtn2) {
    isAltitude = !isAltitude;
  }
  if (btn1 && !lastBtn1) {
    isImperial = !isImperial;
  }
  
  lastBtn1 = btn1;
  lastBtn2 = btn2;

  serviceGPS();
  updateGNSSFilters();

  // UI render (rate-limited)
  if (now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    canvas.fillScreen(ST77XX_BLACK);

    // Determine current state
    bool hasCurrentFix = GPS.fix && 
                        ((isAltitude && isfinite(GPS.altitude)) || 
                         (!isAltitude && isfinite(GPS.speed)));

    // Title
    canvas.setCursor(0, 17);
    canvas.setFont(&FreeSans12pt7b);
    canvas.setTextColor(hasCurrentFix ? ST77XX_WHITE : inactiveStatusColor);
    canvas.println(isAltitude ? "Altitude\n" : "Speed\n");

    uint16_t valueColor;
    uint16_t statusColor;
    const char* statusText;
    float valueToShow = 0.0f;
    bool showValue = true;

    if (hasCurrentFix) {
      // Current live data
      valueColor = ST77XX_GREEN;
      statusColor = ST77XX_BLUE;
      static char satBuf[32];
      snprintf(satBuf, sizeof(satBuf), "%d satellites connected", GPS.satellites);
      statusText = satBuf;

      if (isAltitude) {
        valueToShow = GPS.altitude;
      } else {
        valueToShow = GPS.speed * KNOTS_TO_MPS;
      }

    } else if (everHadFix) {
      // Stale: show last known value in grey + time ago
      valueColor = inactiveStatusColor;
      statusColor = inactiveStatusColor;

      uint32_t secondsAgo = (now - lastGoodFixTime) / 1000;
      static char timeBuf[32];
      if (secondsAgo < 60) {
        snprintf(timeBuf, sizeof(timeBuf), "(%lu seconds ago)", secondsAgo);
      } else if (secondsAgo < 3600) {
        snprintf(timeBuf, sizeof(timeBuf), "(%lu minutes ago)", secondsAgo / 60);
      } else {
        snprintf(timeBuf, sizeof(timeBuf), "(%lu hours ago)", secondsAgo / 3600);
      }
      statusText = timeBuf;

      if (isAltitude) {
        valueToShow = lastGoodAlt_m;
      } else {
        valueToShow = lastGoodSpd_mps;
      }

    } else {
      // Never had a fix
      valueColor = inactiveStatusColor;
      statusColor = inactiveStatusColor;
      static char acquireBuf[40];
      snprintf(acquireBuf, sizeof(acquireBuf), "Looking for satellites (%d found)", GPS.satellites);
      statusText = acquireBuf;
      showValue = false;
    }

    // Draw large value (or ---), centered
    canvas.setFont(&FreeSansBold24pt7b);
    canvas.setTextColor(valueColor);

    bool goodValue = showValue && isfinite(valueToShow);
    
    // Build value string
    static char valueBuf[20];
    const char* unit = isAltitude ? (isImperial ? unitFeet : unitMeters)
                                  : (isImperial ? unitMPH : unitMSec);

    if (goodValue) {
      float displayVal = isAltitude 
        ? (isImperial ? valueToShow * METERS_TO_FEET : valueToShow)
        : (isImperial ? valueToShow * 2.23694f : valueToShow);
      snprintf(valueBuf, sizeof(valueBuf), "%.0f%s", displayVal, unit);
    } else {
      snprintf(valueBuf, sizeof(valueBuf), "---%s", unit);
    }

    // Center horizontally
    int16_t x1, y1;
    uint16_t w, h;
    canvas.getTextBounds(valueBuf, 0, 0, &x1, &y1, &w, &h);
    canvas.setCursor((240 - w) / 2, canvas.getCursorY());
    canvas.println(valueBuf);

    // Status line (centered)
    canvas.setFont(&FreeSans9pt7b);
    canvas.setTextColor(statusColor);
    canvas.getTextBounds(statusText, 0, 0, &x1, &y1, &w, &h);
    canvas.setCursor((240 - w) / 2, canvas.getCursorY());
    canvas.println(statusText);

    // Push to screen
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    lastDisplayUpdate = now;
  }

  debugPrint1Hz();
}
