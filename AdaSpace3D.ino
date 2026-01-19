/*
 * AdaSpace3D - Unified Firmware (Golden Release)
 * * Features: Dual LED Drive, Reactive Lighting, Auto-Hardware Detect
 * * Safety:   Includes Sensor Watchdog to auto-reset frozen I2C lines
 */
#undef USB_PRODUCT
#define USB_PRODUCT "SpaceMouse Pro Wireless"

#undef BOARD_MANUFACTURER
#define BOARD_MANUFACTURER "3Dconnexion"

#include "src/TLx493D/TLx493D_inc.hpp"
#include <Adafruit_NeoPixel.h>          
// It's included in RP2040 core by default
#include <Adafruit_TinyUSB.h>     
#include <math.h>
#include "UserConfig.h"

// --- CONSTANTS ---
#ifdef PIN_NEOPIXEL
#undef PIN_NEOPIXEL
#endif
#define PIN_NEOPIXEL 16

#define PIN_SIMPLE 25
#define HANG_THRESHOLD 50           // consecutive identical readings before reset
#define PREVENTIVE_RESET_INTERVAL 0 // Set to 300000 (5 mins) if you want periodic resets

// --- LED SETUP ---
Adafruit_NeoPixel strip(NUM_ADDRESSABLE_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// --- HARDWARE GLOBALS ---
const uint8_t physPins[] = {BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN};
const uint8_t physToHID[] = {13, 14, 15, 16};
bool currentButtonState[] = {false, false, false, false};
bool prevButtonState[] = {false, false, false, false};

// --- DATA STRUCTURES ---
struct MagCalibration
{
  double x_neutral = 0.0, y_neutral = 0.0, z_neutral = 0.0;
  bool calibrated = false;
} magCal;

struct SensorWatchdog
{
  double last_x = 0.0, last_y = 0.0, last_z = 0.0;
  int sameValueCount = 0;
  unsigned long lastResetTime = 0;
  unsigned long lastPreventiveReset = 0;
} watchdog;

using namespace ifx::tlx493d;

TLx493D_A1B6 magCable(Wire1, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 magSolder(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 *activeSensor = nullptr;

// HID Report Descriptor
static const uint8_t spaceMouse_hid_report_desc[] = {
    0x05, 0x01, 0x09, 0x08, 0xA1, 0x01, 0xA1, 0x00, 0x85, 0x01, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x36, 0x00, 0x80, 0x46, 0xFF, 0x7F,
    0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02, 0xC0,
    0xA1, 0x00, 0x85, 0x02, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x36, 0x00, 0x80, 0x46, 0xFF, 0x7F,
    0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02, 0xC0,
    0xA1, 0x00, 0x85, 0x03, 0x05, 0x09, 0x19, 0x01, 0x29, 0x20, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x20, 0x81, 0x02, 0xC0,
    0xC0};

Adafruit_USBD_HID usb_hid;
#define CALIBRATION_SAMPLES 50

// --- LED LOGIC ---

void updateHardwareLeds(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t c = strip.Color(r, g, b);
  strip.fill(c);
  strip.show();

  int brightness = (r * 77 + g * 150 + b * 29) >> 8;
  brightness = (brightness * LED_BRIGHTNESS) / 255;
  analogWrite(PIN_SIMPLE, brightness);
}

void blinkError()
{
  while (1)
  {
    updateHardwareLeds(255, 0, 0);
    delay(500);
    updateHardwareLeds(0, 0, 0);
    delay(500);
  }
}

void handleLeds(double totalMove)
{
  if (LED_MODE == 0)
  { // STATIC
    // Rate-limit: static color doesn't need updating every cycle
    static unsigned long lastStaticUpdate = 0;
    if (millis() - lastStaticUpdate < 500)
      return; // Only refresh every 500ms
    lastStaticUpdate = millis();
    updateHardwareLeds(LED_COLOR_R, LED_COLOR_G, LED_COLOR_B);
  }
  else if (LED_MODE == 1)
  { // BREATHING
    float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
    uint8_t r = (LED_COLOR_R * (int)val) / 255;
    uint8_t g = (LED_COLOR_G * (int)val) / 255;
    uint8_t b = (LED_COLOR_B * (int)val) / 255;
    updateHardwareLeds(r, g, b);
  }
  else if (LED_MODE == 2)
  { // REACTIVE
    // Rate-limit LED updates to prevent strip.show() from blocking HID reports
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate < 50)
      return; // Only update LEDs every 50ms
    lastLedUpdate = millis();

    int minScale = 50;
    int maxScale = 255;
    int currentScale = minScale;

    if (totalMove > CONFIG_DEADZONE)
    {
      int addedIntensity = (int)(totalMove * 30.0);
      currentScale = constrain(minScale + addedIntensity, minScale, maxScale);
    }

    uint8_t r = (LED_COLOR_R * currentScale) / 255;
    uint8_t g = (LED_COLOR_G * currentScale) / 255;
    uint8_t b = (LED_COLOR_B * currentScale) / 255;
    updateHardwareLeds(r, g, b);
  }
}

void resetMagnetometer()
{
  // Briefly flash Red to indicate reset
  updateHardwareLeds(255, 0, 0);

  activeSensor->end();
  delay(50);

  // Restart the correct Wire interface
  if (activeSensor == &magCable)
  {
    Wire1.end();
    delay(50);
    Wire1.begin();
  }
  else
  {
    Wire.end();
    delay(50);
    Wire.begin();
  }
  delay(50);

  if (activeSensor->begin())
  {
    watchdog.sameValueCount = 0;
    watchdog.lastResetTime = millis();
    // Return to normal color
    updateHardwareLeds(LED_COLOR_R, LED_COLOR_G, LED_COLOR_B);
  }
}

void setup() {

#if DEBUG_MODE
  delay(2000); // Wait for serial monitor
  Serial.begin(115200);
  while (!Serial) ;
  Serial.println("AdaSpace3D - Debug Mode Enabled");
#endif
  pinMode(PIN_SIMPLE, OUTPUT);
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  updateHardwareLeds(255, 0, 0); // Boot Red

  for (uint8_t i = 0; i < 4; i++)
    pinMode(physPins[i], INPUT_PULLUP);

  Serial.println("AdaSpace3D - Setup 1");

  TinyUSBDevice.setID(USB_VID, USB_PID);
  usb_hid.setReportDescriptor(spaceMouse_hid_report_desc, sizeof(spaceMouse_hid_report_desc));
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  Serial.println("AdaSpace3D - Setup 2");

  while (!TinyUSBDevice.mounted())
    delay(100);

  Serial.println("AdaSpace3D - Setup 3");

  pinMode(MAG_POWER_PIN, OUTPUT);
  digitalWrite(MAG_POWER_PIN, HIGH);
  delay(10);

  Wire1.setSCL(I2C_SCL);
  Wire1.setSDA(I2C_SDA);
  Wire1.begin();


  Wire1.setClock(400000);
  if (magCable.begin()) {
    activeSensor = &magCable;
    updateHardwareLeds(0, 255, 0);
    delay(500); // Green for Cable
  }
  else {
    Wire.begin();
    Wire.setClock(400000);
    if (magSolder.begin()) {
      activeSensor = &magSolder;
      updateHardwareLeds(0, 255, 255);
      delay(500); // Cyan for Solder
    }
    else {
      //blinkError();
      activeSensor = nullptr;
      Serial.println("No magnetometer detected!");
    }
  }

  Serial.println("AdaSpace3D - Setup 4");

  watchdog.lastPreventiveReset = millis();
  if (activeSensor)
    calibrateMagnetometer();

  Serial.println("AdaSpace3D - Setup Complete");
}

void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    currentButtonState[i] = (digitalRead(physPins[i]) == LOW);
    if (currentButtonState[i] != prevButtonState[i]) {
      setButtonStateHID(physToHID[i], currentButtonState[i]);
      prevButtonState[i] = currentButtonState[i];
    }
  }


  if (activeSensor) {
    readAndSendMagnetometerDataTrig();
  }
  delay(2);
}




void calibrateMagnetometer() {
  double sumX = 0, sumY = 0, sumZ = 0;
  int valid = 0;

  updateHardwareLeds(0, 0, 0);
  delay(100);
  updateHardwareLeds(255, 255, 255);

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    double x, y, z;
    if (activeSensor->getMagneticField(&x, &y, &z)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      valid++;
    }
    delay(10);
  }

  if (valid > 0) {
    magCal.x_neutral = sumX / valid;
    magCal.y_neutral = sumY / valid;
    magCal.z_neutral = sumZ / valid;
    magCal.calibrated = true;
    updateHardwareLeds(0, 0, 0);
    delay(200);
  }
  else {
    for (int i = 0; i < 5; i++) {
      updateHardwareLeds(255, 0, 0);
      delay(50);
      updateHardwareLeds(0, 0, 0);
      delay(50);
    }
    calibrateMagnetometer();
  }
}

void readAndSendMagnetometerData() {
  double x, y, z;

  // Preventive Reset Check
  if (PREVENTIVE_RESET_INTERVAL > 0 && millis() - watchdog.lastPreventiveReset > PREVENTIVE_RESET_INTERVAL)
  {
    resetMagnetometer();
    watchdog.lastPreventiveReset = millis();
    return;
  }

  if (activeSensor->getMagneticField(&x, &y, &z))
  {
    if (isfinite(x) && isfinite(y) && isfinite(z))
    {

      double rawX = x;
      double rawY = y;
      double rawZ = z;

      // --- WATCHDOG LOGIC ---
      // If values are IDENTICAL to last read, sensor might be frozen
      if (x == watchdog.last_x && y == watchdog.last_y && z == watchdog.last_z)
      {
        watchdog.sameValueCount++;
        if (watchdog.sameValueCount >= HANG_THRESHOLD)
        {
          resetMagnetometer();
          return;
        }
      }
      else
      {
        watchdog.sameValueCount = 0;
        watchdog.last_x = x;
        watchdog.last_y = y;
        watchdog.last_z = z;
      }

      // Calibration
      if (magCal.calibrated)
      {
        x -= magCal.x_neutral;
        y -= magCal.y_neutral;
        z -= magCal.z_neutral;
      }

      double totalMove = abs(x) + abs(y) + abs(z);
      handleLeds(totalMove);

      if (abs(x) < CONFIG_DEADZONE)
        x = 0.0;
      if (abs(y) < CONFIG_DEADZONE)
        y = 0.0;
      if (abs(z) < CONFIG_ZOOM_DEADZONE)
        z = 0.0;

      int16_t tx = (int16_t)(-x * CONFIG_TRANS_SCALE);
      int16_t ty = (int16_t)(-y * CONFIG_TRANS_SCALE);
      int16_t tz = (int16_t)(z * CONFIG_ZOOM_SCALE);
      int16_t rx = (int16_t)(y * CONFIG_ROT_SCALE);
      int16_t ry = (int16_t)(x * CONFIG_ROT_SCALE);

      #if DEBUG_MODE
      static unsigned long lastDebugPrint = 0;
      unsigned long now = millis();
      if (now - lastDebugPrint > 20)
      {
        lastDebugPrint = now;
        Serial.print("RAW:");
        Serial.print(rawX);
        Serial.print(',');
        Serial.print(rawY);
        Serial.print(',');
        Serial.print(rawZ);
        Serial.print(" | CAL:");
        Serial.print(x);
        Serial.print(',');
        Serial.print(y);
        Serial.print(',');
        Serial.print(z);
        Serial.print(" | TX:");
        Serial.print(tx);
        Serial.print(',');
        Serial.print(ty);
        Serial.print(',');
        Serial.print(tz);
        Serial.print(" | RX:");
        Serial.print(rx);
        Serial.print(',');
        Serial.print(ry);
        Serial.print(',');
        Serial.println(0);
      }
      #endif

      send_tx_rx_reports(tx, ty, tz, rx, ry, 0);
    }
  }
  else
  {
    // If we get an error reading, report 0
    send_tx_rx_reports(0, 0, 0, 0, 0, 0);
  }
}

void readAndSendMagnetometerDataTrig() {
  delay(500);
  double x, y, z;

  if (PREVENTIVE_RESET_INTERVAL > 0 && millis() - watchdog.lastPreventiveReset > PREVENTIVE_RESET_INTERVAL)
  {
    resetMagnetometer();
    watchdog.lastPreventiveReset = millis();
    return;
  }

  if (activeSensor->getMagneticField(&x, &y, &z))
  {
    if (isfinite(x) && isfinite(y) && isfinite(z))
    {
      double rawX = x;
      double rawY = y;
      double rawZ = z;

      if (x == watchdog.last_x && y == watchdog.last_y && z == watchdog.last_z)
      {
        watchdog.sameValueCount++;
        if (watchdog.sameValueCount >= HANG_THRESHOLD)
        {
          resetMagnetometer();
          return;
        }
      }
      else
      {
        watchdog.sameValueCount = 0;
        watchdog.last_x = x;
        watchdog.last_y = y;
        watchdog.last_z = z;
      }

      double yaw = 0.0, baseYaw = 0.0;
      double pitch = 0.0, basePitch = 0.0;

      if (magCal.calibrated)
      {
        double rxy = sqrt(rawX * rawX + rawY * rawY);
        double r0xy = sqrt(magCal.x_neutral * magCal.x_neutral + magCal.y_neutral * magCal.y_neutral);
        if (rxy < 1e-6) rxy = 1e-6;
        if (r0xy < 1e-6) r0xy = 1e-6;

        yaw = atan2(rawY, rawX);
        baseYaw = atan2(magCal.y_neutral, magCal.x_neutral);
        pitch = atan2(rawZ, rxy);
        basePitch = atan2(magCal.z_neutral, r0xy);
      }

      if (magCal.calibrated)
      {
        x -= magCal.x_neutral;
        y -= magCal.y_neutral;
        z -= magCal.z_neutral;
      }

      double totalMove = abs(x) + abs(y) + abs(z);
      handleLeds(totalMove);

      if (abs(x) < CONFIG_DEADZONE)
        x = 0.0;
      if (abs(y) < CONFIG_DEADZONE)
        y = 0.0;
      if (abs(z) < CONFIG_ZOOM_DEADZONE)
        z = 0.0;

      int16_t tx = (int16_t)(-x * CONFIG_TRANS_SCALE);
      int16_t ty = (int16_t)(-y * CONFIG_TRANS_SCALE);
      int16_t tz = (int16_t)(z * CONFIG_ZOOM_SCALE);

      double deltaYaw = yaw - baseYaw;
      double deltaPitch = pitch - basePitch;

      const double PI2 = 2.0 * PI;
      if (deltaYaw > PI) deltaYaw -= PI2;
      else if (deltaYaw < -PI) deltaYaw += PI2;
      if (deltaPitch > PI) deltaPitch -= PI2;
      else if (deltaPitch < -PI) deltaPitch += PI2;

      const double ANGLE_DEADZONE = 0.01;
      if (fabs(deltaYaw) < ANGLE_DEADZONE) deltaYaw = 0.0;
      if (fabs(deltaPitch) < ANGLE_DEADZONE) deltaPitch = 0.0;

      const double ROT_ANGLE_SCALE = 800.0;
      int16_t rx = (int16_t)(deltaPitch * ROT_ANGLE_SCALE);
      int16_t ry = (int16_t)(deltaYaw * ROT_ANGLE_SCALE);

      #if DEBUG_MODE
      static unsigned long lastDebugPrint = 0;
      unsigned long now = millis();
      if (now - lastDebugPrint > 20)
      {
        lastDebugPrint = now;
        Serial.print("RAW:");
        Serial.print(rawX);
        Serial.print(',');
        Serial.print(rawY);
        Serial.print(',');
        Serial.print(rawZ);
        Serial.print(" | CAL:");
        Serial.print(x);
        Serial.print(',');
        Serial.print(y);
        Serial.print(',');
        Serial.print(z);
        Serial.print(" | ANG:");
        Serial.print(deltaYaw);
        Serial.print(',');
        Serial.print(deltaPitch);
        Serial.print(" | TX:");
        Serial.print(tx);
        Serial.print(',');
        Serial.print(ty);
        Serial.print(',');
        Serial.print(tz);
        Serial.print(" | RX:");
        Serial.print(rx);
        Serial.print(',');
        Serial.print(ry);
        Serial.print(',');
        Serial.println(0);
      }
      #endif

      send_tx_rx_reports(tx, ty, tz, rx, ry, 0);
    }
  }
  else
  {
    send_tx_rx_reports(0, 0, 0, 0, 0, 0);
  }
}

void setButtonStateHID(uint8_t hidButton, bool pressed)
{
  #ifdef DEBUG_MODE
  if (DEBUG_MODE) {
    Serial.print("BUTTON ");
    Serial.print(hidButton);
    Serial.print(" : ");
    Serial.println(pressed ? "PRESSED" : "RELEASED");
    return;
  }
  #endif

  if (!TinyUSBDevice.mounted())
    return;
  uint8_t report[4] = {0, 0, 0, 0};
  if (pressed && hidButton <= 32)
  {
    report[(hidButton - 1) / 8] = (1 << ((hidButton - 1) % 8));
  }
  if (usb_hid.ready())
    usb_hid.sendReport(3, report, 4);
}

void send_tx_rx_reports(int16_t tx, int16_t ty, int16_t tz, int16_t rx, int16_t ry, int16_t rz)
{
  #ifdef DEBUG_MODE
  if (DEBUG_MODE) {
    return;
  }
  #endif

  if (!TinyUSBDevice.mounted() || !usb_hid.ready())
    return;

  uint8_t tx_report[6] = {(uint8_t)tx, (uint8_t)(tx >> 8), (uint8_t)ty, (uint8_t)(ty >> 8), (uint8_t)tz, (uint8_t)(tz >> 8)};
  usb_hid.sendReport(1, tx_report, 6);

  delayMicroseconds(500);

  uint8_t rx_report[6] = {(uint8_t)rx, (uint8_t)(rx >> 8), (uint8_t)ry, (uint8_t)(ry >> 8), (uint8_t)rz, (uint8_t)(rz >> 8)};
  usb_hid.sendReport(2, rx_report, 6);
}
