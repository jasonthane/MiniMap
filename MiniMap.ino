// SPDX-FileCopyrightText: 2022 Limor Fried for Adafruit Industries
// SPDX-License-Identifier: MIT

// ============================================================================
// Includes
// ============================================================================

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

// ============================================================================
// Configuration
// ============================================================================

// GPS serial port
#define GPSSerial Serial1
#define GPSECHO false  // Echo raw GPS to Serial (noisy)

// Unit conversions
#define METERS_TO_FEET 3.28084f
#define KNOTS_TO_MPS   0.514444f   // 1 knot = 1852m / 3600s
#define KNOTS_TO_MPH   1.15077945f
#define MPS_TO_MPH     2.23694f

// Display timing
const uint32_t DISPLAY_UPDATE_MS = 200;

// GPS base accuracy (MTK3333 conservative estimates)
// These get multiplied by DOP values to estimate max error
#define GPS_BASE_VERT_ERROR_M    5.0f   // Base vertical accuracy (meters)
#define GPS_BASE_SPEED_ERROR_MPS 0.1f   // Base speed accuracy (m/s, Doppler-derived)

// RGB565 color conversion macro
#define RGBHEX(hex) ((((hex) >> 8) & 0xF800) | (((hex) >> 5) & 0x07E0) | (((hex) >> 3) & 0x001F))

// UI colors
const uint16_t colorGray   = RGBHEX(0x535353);
const uint16_t colorGreen  = ST77XX_GREEN;
const uint16_t colorYellow = RGBHEX(0xFFFF00);
const uint16_t colorRed    = RGBHEX(0xFF0000);

// Accuracy thresholds for color coding
#define ALT_ERROR_RED_M     50.0f   // Red if altitude error > 50m
#define ALT_ERROR_YELLOW_M  10.0f   // Yellow if altitude error > 10m
#define SPD_ERROR_RED_MPH   10.0f   // Red if speed error > 10 mph
#define SPD_ERROR_YELLOW_MPH 5.0f   // Yellow if speed error > 5 mph

// Speed display threshold (display 0 if below this)
#define SPEED_ZERO_THRESHOLD_MPH 3.0f
#define SPEED_ZERO_THRESHOLD_MPS 1.0f  // Metric equivalent (~2.2 mph)

// Unit labels
const char* unitMeters = " m";
const char* unitFeet   = " ft";
const char* unitMSec   = " m/s";
const char* unitMPH    = " mph";

// ============================================================================
// Hardware Objects
// ============================================================================

Adafruit_GPS GPS(&GPSSerial);
Adafruit_BME280 bme;
Adafruit_MAX17048 lipo;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);
extern Adafruit_TestBed TB;

// ============================================================================
// State Variables
// ============================================================================

// UI state
bool isImperial = true;   // true = feet + mph, false = meters + m/s
bool isAltitude = true;   // true = altitude mode, false = speed mode

// GPS tracking
static float lastGoodAlt_m   = 0.0f;
static float lastGoodSpd_mps = 0.0f;
static uint32_t lastGoodFixTime = 0;
static bool everHadFix = false;

// Estimated max errors (calculated from DOP × base accuracy)
static float altitudeError_m  = 0.0f;  // Only valid in 3D mode
static float speedError_mps   = 0.0f;  // Valid in 2D or 3D mode

// Button edge detection
static bool lastBtn1 = false;
static bool lastBtn2 = false;

// Timing
static uint32_t lastDisplayUpdate = 0;
static uint32_t lastDbgMs = 0;

// Sensor status
static bool bmefound = false;

// ============================================================================
// Splash Screen
// ============================================================================

// 5x7 bitmap font for "MiniMap" - each byte is a column, LSB at top
const uint8_t PROGMEM letter_M[] = {0x7F, 0x02, 0x04, 0x02, 0x7F};
const uint8_t PROGMEM letter_i[] = {0x00, 0x44, 0x7D, 0x40, 0x00};
const uint8_t PROGMEM letter_n[] = {0x7C, 0x04, 0x04, 0x04, 0x78};
const uint8_t PROGMEM letter_a[] = {0x20, 0x54, 0x54, 0x54, 0x78};
const uint8_t PROGMEM letter_p[] = {0x7C, 0x14, 0x14, 0x14, 0x08};

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
  int lx = textX;
  for (int i = 0; i < numLetters; i++) {
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

  // Phase 1: Diagonal sweep in (~0.25 seconds)
  for (int wave = 0; wave <= maxDiag + 10; wave += 8) {
    canvas.fillScreen(ST77XX_BLACK);

    for (int gy = 0; gy < gridH; gy++) {
      for (int gx = 0; gx < gridW; gx++) {
        int diag = gx + gy;
        if (diag <= wave) {
          int px = gx * blockSize;
          int py = gy * blockSize;
          int cx = px + blockSize / 2;
          int cy = py + blockSize / 2;
          bool isLogo = isPartOfLogo(cx, cy, textX, textY, scale, letterWidth, gap);

          int colorIdx = (gx * 7) / gridW;
          uint16_t color = isLogo ? getLogoColor(cx, textX, letterWidth, gap, logoColors) : bgColors[colorIdx];

          canvas.fillRect(px, py, blockSize - 1, blockSize - 1, color);
        }
      }
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    delay(25);
  }

  // Phase 2: Background blocks fade out (~0.5 seconds)
  for (int fade = 0; fade < 10; fade++) {
    canvas.fillScreen(ST77XX_BLACK);

    for (int gy = 0; gy < gridH; gy++) {
      for (int gx = 0; gx < gridW; gx++) {
        int px = gx * blockSize;
        int py = gy * blockSize;
        int cx = px + blockSize / 2;
        int cy = py + blockSize / 2;

        bool isLogo = isPartOfLogo(cx, cy, textX, textY, scale, letterWidth, gap);

        if (isLogo) {
          uint16_t color = getLogoColor(cx, textX, letterWidth, gap, logoColors);
          canvas.fillRect(px, py, blockSize - 1, blockSize - 1, color);
        } else if (random(10) > fade) {
          int colorIdx = (gx * 7) / gridW;
          canvas.fillRect(px, py, blockSize - 1, blockSize - 1, bgColors[colorIdx]);
        }
      }
    }

    display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
    delay(50);
  }

  // Phase 3: Hold logo only
  canvas.fillScreen(ST77XX_BLACK);
  for (int gy = 0; gy < gridH; gy++) {
    for (int gx = 0; gx < gridW; gx++) {
      int px = gx * blockSize;
      int py = gy * blockSize;
      int cx = px + blockSize / 2;
      int cy = py + blockSize / 2;
      if (isPartOfLogo(cx, cy, textX, textY, scale, letterWidth, gap)) {
        canvas.fillRect(px, py, blockSize - 1, blockSize - 1, getLogoColor(cx, textX, letterWidth, gap, logoColors));
      }
    }
  }
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  delay(500);
}

// ============================================================================
// GPS Functions
// ============================================================================

static inline void serviceGPS() {
  while (GPSSerial.available()) {
    char c = GPS.read();
    if (GPSECHO && c) Serial.print(c);

    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
    }
  }
}

static inline void updateGNSSFilters() {
  if (GPS.fix) {
    // Only trust altitude when we have a real 3D solution
    if (GPS.fixquality_3d == 3 && isfinite(GPS.altitude)) {
      lastGoodAlt_m = GPS.altitude;
      // VDOP is only meaningful in 3D mode
      altitudeError_m = GPS_BASE_VERT_ERROR_M * GPS.VDOP;
    }
    if (isfinite(GPS.speed)) {
      lastGoodSpd_mps = GPS.speed * KNOTS_TO_MPS;
      // Speed error based on HDOP (works in 2D and 3D)
      speedError_mps = GPS_BASE_SPEED_ERROR_MPS * GPS.HDOP;
    }
    lastGoodFixTime = millis();
    everHadFix = true;
  }
}

// ============================================================================
// Debug
// ============================================================================

static inline void debugPrint1Hz() {
  uint32_t now = millis();
  if (now - lastDbgMs < 1000) return;
  lastDbgMs = now;

  Serial.print(F("fix="));      Serial.print(GPS.fix);
  Serial.print(F(" mode="));    Serial.print(GPS.fixquality_3d);
  Serial.print(F(" sats="));    Serial.print(GPS.satellites);
  Serial.print(F(" alt_m="));   Serial.print(GPS.altitude, 1);
  Serial.print(F(" spd_mps=")); Serial.print(GPS.speed * KNOTS_TO_MPS, 2);
  Serial.print(F(" HDOP="));    Serial.print(GPS.HDOP, 1);
  Serial.print(F(" VDOP="));    Serial.print(GPS.VDOP, 1);
  Serial.print(F(" PDOP="));    Serial.print(GPS.PDOP, 1);
  Serial.print(F(" alt_err_m="));  Serial.print(altitudeError_m, 1);
  Serial.print(F(" spd_err_mps=")); Serial.print(speedError_mps, 2);

  if (everHadFix) {
    uint32_t ago = (now - lastGoodFixTime) / 1000;
    Serial.print(F(" last_fix=")); Serial.print(ago); Serial.print(F("s ago"));
  }
  Serial.println();
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  // ---- GPS initialization ----
  GPSSerial.begin(9600);
  GPS.begin(9600);
  delay(100);

  // Enable GGA + RMC + GSA sentences (for VDOP/PDOP)
  GPS.sendCommand("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");

  // 2 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);

  // Switch to 115200 baud
  GPS.sendCommand("$PMTK251,115200*1F");
  delay(200);

#if defined(ARDUINO_ARCH_ESP32)
  GPSSerial.updateBaudRate(115200);
#else
  GPSSerial.begin(115200);
#endif

  // ---- Display initialization ----
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1;
  TB.begin();
  TB.setColor(WHITE);

  display.init(135, 240);
  display.setRotation(3);

  // Backlight at 50% via PWM
  ledcAttach(TFT_BACKLITE, 5000, 8);
  ledcWrite(TFT_BACKLITE, 127);

  canvas.setFont(&FreeSans12pt7b);
  canvas.setTextColor(ST77XX_WHITE);

  showSplash();
  TB.setColor(0);  // Turn off NeoPixel to save power

  // ---- Battery gauge ----
  if (!lipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.print(F("Found MAX17048 with Chip ID: 0x"));
  Serial.println(lipo.getChipID(), HEX);

  // ---- Optional BME280 ----
  if (TB.scanI2CBus(0x77)) {
    Serial.println("BME280 address");
    unsigned status = bme.begin();
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    } else {
      Serial.println("BME280 found OK");
      bmefound = true;
    }
  }

  // ---- Buttons ----
  pinMode(0, INPUT_PULLDOWN);
  pinMode(1, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  uint32_t now = millis();

  // ---- Button handling (edge detection) ----
  bool btn1 = digitalRead(1);
  bool btn2 = digitalRead(2);

  if (btn2 && !lastBtn2) isAltitude = !isAltitude;
  if (btn1 && !lastBtn1) isImperial = !isImperial;

  lastBtn1 = btn1;
  lastBtn2 = btn2;

  // ---- GPS processing ----
  serviceGPS();
  updateGNSSFilters();

  // ---- UI rendering (rate-limited) ----
  if (now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    canvas.fillScreen(ST77XX_BLACK);

    // Determine fix state
    bool has3DFix = GPS.fix && GPS.fixquality_3d == 3;
    bool hasCurrentFix = GPS.fix &&
                         ((isAltitude && isfinite(GPS.altitude)) ||
                          (!isAltitude && isfinite(GPS.speed)));

    // Title
    canvas.setCursor(0, 17);
    canvas.setFont(&FreeSans12pt7b);
    canvas.setTextColor(hasCurrentFix ? ST77XX_WHITE : colorGray);
    canvas.println(isAltitude ? "Altitude\n" : "Speed\n");

    uint16_t valueColor;
    uint16_t statusColor;
    const char* statusText;
    float valueToShow = 0.0f;
    bool showValue = true;

    if (hasCurrentFix && (!isAltitude || has3DFix)) {
      // Live data with valid fix
      
      // Determine color based on accuracy thresholds
      uint16_t accuracyColor;
      if (isAltitude) {
        // Color based on altitude error (in meters)
        if (altitudeError_m > ALT_ERROR_RED_M) {
          accuracyColor = colorRed;
        } else if (altitudeError_m > ALT_ERROR_YELLOW_M) {
          accuracyColor = colorYellow;
        } else {
          accuracyColor = colorGreen;
        }
      } else {
        // Color based on speed error (in mph)
        float speedError_mph = speedError_mps * MPS_TO_MPH;
        if (speedError_mph > SPD_ERROR_RED_MPH) {
          accuracyColor = colorRed;
        } else if (speedError_mph > SPD_ERROR_YELLOW_MPH) {
          accuracyColor = colorYellow;
        } else {
          accuracyColor = colorGreen;
        }
      }
      valueColor = accuracyColor;
      statusColor = accuracyColor;

      // Get value to show (speed shows 0 if below threshold)
      bool speedBelowThreshold = false;
      if (isAltitude) {
        valueToShow = GPS.altitude;
      } else {
        float speed_mps = GPS.speed * KNOTS_TO_MPS;
        speedBelowThreshold = (speed_mps < SPEED_ZERO_THRESHOLD_MPS);
        valueToShow = speedBelowThreshold ? 0.0f : speed_mps;
      }

      // Format error in user's units
      static char errorStr[16];
      if (isAltitude) {
        float err = isImperial ? altitudeError_m * METERS_TO_FEET : altitudeError_m;
        snprintf(errorStr, sizeof(errorStr), "%.0f %s", err, isImperial ? "ft" : "m");
      } else {
        // If speed is below threshold (showing 0), use threshold as the error
        if (speedBelowThreshold) {
          snprintf(errorStr, sizeof(errorStr), "%s", isImperial ? "3 mph" : "1 m/s");
        } else {
          float err = isImperial ? speedError_mps * MPS_TO_MPH : speedError_mps;
          if (err < 1.0f) {
            snprintf(errorStr, sizeof(errorStr), "%.1f %s", err, isImperial ? "mph" : "m/s");
          } else {
            snprintf(errorStr, sizeof(errorStr), "%.0f %s", err, isImperial ? "mph" : "m/s");
          }
        }
      }

      // Battery status: charging (USB) or percentage
      static char statusBuf[48];
      float chargeRate = lipo.chargeRate();
      if (chargeRate > 0.5f) {
        // Charging = USB power
        snprintf(statusBuf, sizeof(statusBuf), "%d sats  +/-%s  USB power",
                 GPS.satellites, errorStr);
      } else {
        // On battery
        int battPct = (int)lipo.cellPercent();
        if (battPct > 100) battPct = 100;
        snprintf(statusBuf, sizeof(statusBuf), "%d sats  +/-%s  %d%% battery",
                 GPS.satellites, errorStr, battPct);
      }
      statusText = statusBuf;

    } else if (everHadFix) {
      // Stale data - show last known value
      valueColor = colorGray;
      statusColor = colorGray;

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

      valueToShow = isAltitude ? lastGoodAlt_m : lastGoodSpd_mps;

    } else {
      // Never had a fix
      valueColor = colorGray;
      statusColor = colorGray;

      static char acquireBuf[40];
      snprintf(acquireBuf, sizeof(acquireBuf), "Looking for satellites (%d found)", GPS.satellites);
      statusText = acquireBuf;
      showValue = false;
    }

    // Draw large value (centered)
    canvas.setFont(&FreeSansBold24pt7b);
    canvas.setTextColor(valueColor);

    bool goodValue = showValue && isfinite(valueToShow);

    static char valueBuf[20];
    const char* unit = isAltitude ? (isImperial ? unitFeet : unitMeters)
                                  : (isImperial ? unitMPH : unitMSec);

    if (goodValue) {
      float displayVal = isAltitude
        ? (isImperial ? valueToShow * METERS_TO_FEET : valueToShow)
        : (isImperial ? valueToShow * MPS_TO_MPH : valueToShow);
      snprintf(valueBuf, sizeof(valueBuf), "%.0f%s", displayVal, unit);
    } else {
      snprintf(valueBuf, sizeof(valueBuf), "---%s", unit);
    }

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
