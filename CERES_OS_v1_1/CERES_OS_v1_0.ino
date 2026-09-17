// CERES_OS made by StarlingTech (https://youtube.com/@starling_tech?si=prGFbckiPpZF3i4D)
// Firmware version: V.1.1 (keep in sync with Sys_ver below)
// CERES_OS is a small operating system for smartwatches built around esp32c3
// It is the first iteration of this system and also our first opensource project, 
// so feel free to edit and if You see some issues or bug, please let us know.
// This program was created with help of AI. It is still developed, so some functions are not ready yet.
// Circuit is available in Electonics folder.
// boards manager: esp32 version: 1.0

//LIBRARIES
#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Fonts/Org_01.h>
#include <esp_sleep.h>
#include "rom/gpio.h"
#include "driver/gpio.h"
#include <FS.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// RTC 
RTC_DATA_ATTR unsigned long totalSleepTimeSeconds = 0;
RTC_DATA_ATTR time_t sleepStartTime = 0; 
RTC_DATA_ATTR char lastSyncTimeStr[30] = "Never";

// Configuration of WI-FI / NTP
Preferences wifiPrefs;
WiFiMulti wifiMulti;
#define MAX_WIFI_NETWORKS 3   
const char* AP_CONFIG_NAME = "CERES-Setup";   
const char* ntpServer = "pool.ntp.org";
const char* TZ_INFO   = "CET-1CEST,M3.5.0,M10.5.0/3";
const unsigned long WIFI_RETRY_INTERVAL = 15000; 
unsigned long lastWifiRetry = 0;
bool wifiCredentialsExist = false;
float currentTemp = 0.0;
int currentHum = 0;
unsigned long lastWeatherUpdate = 0;
const unsigned long WEATHER_UPDATE_INTERVAL = 30000;
const char* Sys_ver = "CERES.OS V.1.0";

// =================== ACCELEROMETER (ADXL345, shared I2C SDA=6 SCL=7) ===================
// Wiring: VCC->3V3, GND->GND, SDA->GPIO6, SCL->GPIO7 (same bus as OLED 0x3C).
// Mode: pedometer ONLY. No tilt control, no effect on sleep.
#define ACCEL_INT_PIN     -1   //  -1 = INT not connected
#define STEP_GOAL_DEFAULT 6000
#define STEP_GOAL_MIN     1000
#define STEP_GOAL_MAX     30000
#define STEP_GOAL_STEP    500
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
bool accelAvailable = false;
float accelX = 0, accelY = 0, accelZ = 1.0;
RTC_DATA_ATTR int stepCount = 0;
RTC_DATA_ATTR int stepGoal = STEP_GOAL_DEFAULT;
RTC_DATA_ATTR int bestScore = 0; // STARSHOOTER high score
RTC_DATA_ATTR bool goalDone = false;      // goal celebration already played (reset at midnight)
RTC_DATA_ATTR int lastStepDay = -1;       // tm_yday of last counting
RTC_DATA_ATTR int stepHist[7] = {0,0,0,0,0,0,0}; // step history, [0]=yesterday
unsigned long lastAccelRead = 0;
const unsigned long ACCEL_READ_MS = 100;
// step detector state
unsigned long lastStepTime = 0;
bool stepWasHigh = false;

// Screen settings
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define I2C_ADDRESS  0x3C
#define OLED_RESET     -1
#define I2C_SDA         6
#define I2C_SCL         7
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pinout for esp32c3
#define BUTTON_PIN      2  // PWR / long-press = deep sleep / short = Enter Alarm
#define BUTTON_NEXT_PIN 4  // next screen / +1 value
#define BUTTON_PREV_PIN 3  // previous screen / -1 value
#define BATTERY_PIN     1
#define RGB_LED_PIN     8
#define BUZZER_PIN      10 

Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800); // if your device didn't have additional LED, it is not important

// Card system (SYSTEM always last)
#define CARD_COUNT 6
enum Card {
  CARD_HOME = 0,
  CARD_WATCH,
  CARD_WEATHER,
  CARD_MOTION,
  CARD_GAME,
  CARD_SYSTEM
};
int currentCard = CARD_HOME;

// Motion sub-screens (short PWR switches, like watchState). UI style unchanged.
// OVERVIEW=steps+goal, STEPS=distance/kcal, EDIT_GOAL=goal edit, HIST=7d history.
enum MotionState {
  MOTION_OVERVIEW = 0,
  MOTION_STEPS,
  MOTION_EDIT_GOAL,
  MOTION_HIST
};
MotionState motionState = MOTION_OVERVIEW;

// TIMING / STATE
bool isWifiConnected     = false;
bool isPressed           = false;
unsigned long lastActivityTime  = 0;
unsigned long buttonPressedTime = 0;
const unsigned long INACTIVITY_TIMEOUT = 30000;

// debounce NEXT/PREV
unsigned long lastNextPress = 0;
unsigned long lastPrevPress = 0;
const unsigned long DEBOUNCE_MS = 250;

// NEXT+PREV = Wifi config
unsigned long comboPressStartTime = 0;
bool comboPressActive = false;
const unsigned long COMBO_HOLD_MS = 2500;

unsigned long lastDisplayUpdate = 0; 
unsigned long lastBatteryUpdate = 0;
int currentBatteryPct = 0;

// --- ALARM, TIMER, STOPWATCH VARIABLES ---
enum WatchState { 
  WATCH_NORMAL = 0, 
  WATCH_EDIT_ALARM_H, 
  WATCH_EDIT_ALARM_M, 
  WATCH_EDIT_COUNTDOWN_MIN, 
  WATCH_EDIT_COUNTDOWN_SEC, 
  WATCH_EDIT_STOPWATCH 
};
WatchState watchState = WATCH_NORMAL;

// Alarm
int alarmHour = 0, alarmMinute = 0;
bool alarmActive = false;

// Countdown (Set Time)
int cdEditM = 0, cdEditS = 0;
bool countdownActive = false;
unsigned long countdownTargetTime = 0;

// Stopwatch (Timer)
bool stopwatchActive = false;
unsigned long stopwatchStartTime = 0;
unsigned long stopwatchElapsed = 0;

// Buzzer State
bool isBuzzing = false;
unsigned long buzzStartTime = 0;

// UI_Components
static const unsigned char PROGMEM image_battery_empty_bits[] = {
  0x00,0x00,0x00,0x0f,0xff,0xfe,0x10,0x00,0x01,0x10,0x00,0x01,
  0x70,0x00,0x01,0x80,0x00,0x01,0x80,0x00,0x01,0x80,0x00,0x01,
  0x80,0x00,0x01,0x80,0x00,0x01,0x70,0x00,0x01,0x10,0x00,0x01,
  0x10,0x00,0x01,0x0f,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x00
};
static const unsigned char PROGMEM image_bluetooth_bits[] = {
  0x01,0x00,0x02,0x80,0x02,0x40,0x22,0x20,0x12,0x20,0x0a,0x40,
  0x06,0x80,0x03,0x00,0x06,0x80,0x0a,0x40,0x12,0x20,0x22,0x20,
  0x02,0x40,0x02,0x80,0x01,0x00,0x00,0x00
};
static const unsigned char PROGMEM image_menu_home_bits[] = {
  0x01,0x00,0x12,0x80,0x1c,0x40,0x19,0x20,0x12,0x90,0x24,0x48,
  0x48,0x24,0x90,0x12,0x66,0x0c,0x26,0x08,0x20,0xe8,0x20,0xa8,
  0x20,0xe8,0x20,0xa8,0x3f,0xf8,0x00,0x00
};
static const unsigned char PROGMEM image_menu_settings_sliders_two_bits[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x44,0x00,0xc7,0xfc,0x44,0x00,0x38,
  0x00,0x00,0x70,0x00,0x88,0xff,0x8c,0x00,0x88,0x00,0x70,0x00,0x00,0x00,0x00,
  0x00,0x00
};
static const unsigned char PROGMEM image_paint_2_bits[] = {
  0xff,0xff,0xff,0xf8,0x80,0x00,0x0f,0xf0,0x80,0x00,0x3f,0xe0,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
  0xa0,0x00,0x00,0x00,0xa0,0x00,0x00,0x00,0xe0,0x00,0x00,0x00,
  0xe0,0x00,0x00,0x00,0xe0,0x00,0x00,0x00,0xe0,0x00,0x00,0x00,
  0xe0,0x00,0x00,0x00,0xe0,0x00,0x00,0x00,0xc0,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00
};
static const unsigned char PROGMEM image_paint_2_copy_1_bits[] = {
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x38,
  0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x38,
  0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x28,
  0x00,0x00,0x00,0x28,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x08,
  0x00,0x00,0x00,0x08,0x3f,0xe0,0x00,0x08,0x7f,0x80,0x00,0x08,
  0xff,0xff,0xff,0xf8
};
static const unsigned char PROGMEM image_Pin_pointer_bits[] = {
  0x00,0x3e,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x3e,0x00,0x00,
  0x00,0x3e,0x00,0x00,0x00,0x3e,0x00,0x00,0x07,0xff,0xf0,0x00,
  0x07,0xff,0xf0,0x00,0x07,0xff,0xf0,0x00,0x07,0xff,0xf0,0x00,
  0x07,0xff,0xf0,0x00,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,
  0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80
};
static const unsigned char PROGMEM image_Pin_pointer_copy_1_bits[] = {
  0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,
  0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0x07,0xff,0xf0,0x00,
  0x07,0xff,0xf0,0x00,0x07,0xff,0xf0,0x00,0x07,0xff,0xf0,0x00,
  0x07,0xff,0xf0,0x00,0x00,0x3e,0x00,0x00,0x00,0x3e,0x00,0x00,
  0x00,0x3e,0x00,0x00,0x00,0x3e,0x00,0x00,0x00,0x3e,0x00,0x00
};
static const unsigned char PROGMEM image_weather_cloud_sunny_bits[] = {
  0x00,0x20,0x00,0x02,0x02,0x00,0x00,0x70,0x00,0x01,0x8c,0x00,
  0x09,0x04,0x80,0x02,0x02,0x00,0x02,0x02,0x00,0x07,0x82,0x00,
  0x08,0x44,0x80,0x10,0x2c,0x00,0x30,0x30,0x00,0x60,0x1e,0x00,
  0x80,0x03,0x00,0x80,0x01,0x00,0x80,0x01,0x00,0x7f,0xfe,0x00
};
static const unsigned char PROGMEM image_wifi_bits[] = {
  0x01,0xf0,0x00,0x06,0x0c,0x00,0x18,0x03,0x00,0x21,0xf0,0x80,
  0x46,0x0c,0x40,0x88,0x02,0x20,0x10,0xe1,0x00,0x23,0x18,0x80,
  0x04,0x04,0x00,0x08,0x42,0x00,0x01,0xb0,0x00,0x02,0x08,0x00,
  0x00,0x40,0x00,0x00,0xa0,0x00,0x00,0x40,0x00,0x00,0x00,0x00
};
static const unsigned char PROGMEM image_logo_bits[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
  0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0x7c,0x00,0x00,0x00,
  0x00,0x00,0x00,0x3b,0xb7,0x00,0x00,0x00,0x00,0x00,0x0c,0xde,
  0xdd,0x80,0x00,0x00,0x00,0x00,0x0d,0xe3,0xf6,0xc0,0x00,0x00,
  0x00,0x00,0x0e,0xfe,0xbf,0xe0,0x00,0x00,0x00,0x00,0x0e,0x77,
  0xb6,0xe0,0x00,0x00,0x00,0x00,0x0f,0x7a,0xcf,0x70,0x00,0x00,
  0x00,0x00,0x27,0xae,0x3b,0xd0,0x00,0x00,0x00,0x00,0x37,0xda,
  0xfd,0xf0,0x00,0x00,0x00,0x00,0x1b,0xee,0xf6,0x78,0x00,0x00,
  0x00,0x00,0x1f,0xf1,0xf7,0x98,0x00,0x00,0x00,0x00,0x0f,0xff,
  0xff,0x08,0x00,0x00,0x00,0x00,0x0f,0xff,0xfc,0xfe,0x00,0x00,
  0x00,0x00,0x13,0xff,0xfd,0xd9,0x00,0x00,0x00,0x00,0x0c,0x7f,
  0xf9,0x6b,0x00,0x00,0x00,0x00,0x0f,0x8f,0xfb,0xf7,0x00,0x00,
  0x00,0x00,0x03,0xf9,0xfb,0xac,0x00,0x00,0x00,0x00,0x00,0xfe,
  0xfb,0x98,0x00,0x00,0x00,0x00,0x10,0x3e,0xfa,0x60,0x00,0x00,
  0x00,0x00,0x21,0xfe,0xf1,0x80,0x00,0x00,0x00,0x00,0x20,0xfc,
  0xce,0x40,0x00,0x00,0x00,0x00,0x60,0x00,0x79,0x80,0x00,0x00,
  0x00,0x00,0x78,0x0f,0xce,0x00,0x00,0x00,0x00,0x00,0x1f,0xfc,
  0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x80,0x00,
  0x48,0x00,0x00,0x00,0x00,0x06,0x58,0x00,0x40,0x00,0x00,0x00,
  0x00,0x06,0x1c,0xe7,0x4a,0xe7,0x80,0x00,0x00,0x03,0x99,0xb6,
  0x4b,0x6d,0x80,0x00,0x00,0x00,0xd8,0xf4,0x4a,0x28,0x80,0x00,
  0x00,0x04,0x59,0x34,0x4a,0x2c,0x80,0x00,0x00,0x07,0xcd,0xf4,
  0x4a,0x27,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,
  0x00,0x00,0x00,0x00,0x00,0x0f,0x80,0x00,0x00,0x00,0x00,0x3e,
  0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x08,0x82,0x50,0x00,0x00,
  0x00,0x00,0x00,0x09,0x6f,0x78,0x00,0x00,0x00,0x00,0x00,0x09,
  0xe8,0x48,0x00,0x00,0x00,0x00,0x00,0x09,0x08,0x48,0x00,0x00,
  0x00,0x00,0x00,0x09,0xef,0x48,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
static const unsigned char PROGMEM image_clock_alarm_bits[] = {
  0x79,0x3c,0xb3,0x9a,0xed,0x6e,0xd0,0x16,0xa0,0x0a,0x41,0x04,0x41,
  0x04,0x81,0x02,0xc1,0x06,0x82,0x02,0x44,0x04,0x48,0x04,0x20,0x08,
  0x10,0x10,0x2d,0x68,0x43,0x84
};
static const unsigned char PROGMEM image_music_pause_bits[] = {
  0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,0x00
};
static const unsigned char PROGMEM image_ButtonRightSmall_bits[] = {
  0x80,0xc0,0xe0,0xc0,0x80
};
static const unsigned char PROGMEM image_motion_bits[] = {
  0x0f,0xe0,0x3f,0xf8,0x3f,0xf8,0x3f,0xf8,0x1f,0xf0,0x1f,0xf0,
  0x0f,0xe0,0x0f,0xe0,0x0f,0xe0,0x07,0xc0,0x07,0xc0,0x07,0xc0,
  0x0f,0xe0,0x0f,0xe0,0x0f,0xe0,0x07,0xc0
};
static const unsigned char PROGMEM image_gamepad_bits[] = {
 0x04,0x00,0x7f,0xc0,0xdf,0xe0,0x8e,0xa0,0xdf,0xe0,0xf1,0xe0,0xe0,0xe0,0xc0,0x60
};

// =================== FILE SYSTEM =================== 
void saveConfig() {
  File file = LittleFS.open("/config.txt", "w");
  if (!file) { Serial.println("Saving error of LittleFS!"); return; }
  file.println(lastSyncTimeStr);
  file.println(totalSleepTimeSeconds);
  file.println(currentTemp); 
  file.println(currentHum);
  file.println(stepCount);
  file.println(stepGoal);
  String histLine = "";
  for (int i = 0; i < 7; i++) { if (i > 0) histLine += ","; histLine += String(stepHist[i]); }
  file.println(histLine);
  file.println(bestScore);
  file.close();
}

void loadConfig() {
  if (!LittleFS.exists("/config.txt")) return;
  File file = LittleFS.open("/config.txt", "r");
  if (!file) return;

  String timeData = file.readStringUntil('\n'); timeData.trim(); timeData.toCharArray(lastSyncTimeStr, sizeof(lastSyncTimeStr));
  String sleepData = file.readStringUntil('\n'); sleepData.trim(); if (sleepData.length() > 0) totalSleepTimeSeconds = sleepData.toInt();
  String tempData = file.readStringUntil('\n'); tempData.trim(); if (tempData.length() > 0) currentTemp = tempData.toFloat();
  String humData = file.readStringUntil('\n'); humData.trim(); if (humData.length() > 0) currentHum = humData.toInt();
  String stepData = file.readStringUntil('\n'); stepData.trim(); if (stepData.length() > 0) stepCount = stepData.toInt();
  String goalData = file.readStringUntil('\n'); goalData.trim();
  if (goalData.length() > 0) {
    int g = goalData.toInt();
    if (g >= STEP_GOAL_MIN && g <= STEP_GOAL_MAX) stepGoal = g;
  }
  String histData = file.readStringUntil('\n'); histData.trim();
  if (histData.length() > 0) {
    int hi = 0, pos = 0;
    while (hi < 7 && pos < (int)histData.length()) {
      int comma = histData.indexOf(',', pos);
      String part = (comma < 0) ? histData.substring(pos) : histData.substring(pos, comma);
      int v = part.toInt();
      if (v < 0) v = 0; if (v > 999999) v = 999999;
      stepHist[hi++] = v;
      if (comma < 0) break;
      pos = comma + 1;
    }
  }
  String bestData = file.readStringUntil('\n'); bestData.trim();
  if (bestData.length() > 0) { int b = bestData.toInt(); if (b >= 0 && b < 1000000) bestScore = b; }
  file.close();
}

// =================== ACCELEROMETER ===================
void initAccel() {
  // NOTE: Adafruit_ADXL345::begin() calls Wire.begin() on default pins internally
  // and messes up our SDA=6/SCL=7 bus + OLED. So after begin() we ALWAYS
  // restore Wire.begin(I2C_SDA, I2C_SCL).
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!accel.begin()) {
    Serial.println("ADXL345 not found (0x53/0x1D). Motion OFF.");
    accelAvailable = false;
    Wire.begin(I2C_SDA, I2C_SCL); // restore bus for OLED
    return;
  }
  accelAvailable = true;
  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  Serial.println("ADXL345 OK (shared SDA=6 SCL=7)");
#if ACCEL_INT_PIN >= 0
  pinMode(ACCEL_INT_PIN, INPUT_PULLUP);
#endif
  Wire.begin(I2C_SDA, I2C_SCL); // restore bus for OLED
}

void updateAccel() {
  // Pedometer ONLY. Orientation / tilt / shake do NOTHING:
  // they don't switch cards, don't mute the buzzer, don't touch sleep.
  if (!accelAvailable) return;
  unsigned long now = millis();
  if (now - lastAccelRead < ACCEL_READ_MS) return;
  lastAccelRead = now;

  sensors_event_t ev;
  accel.getEvent(&ev);
  accelX = ev.acceleration.x / 9.80665f;
  accelY = ev.acceleration.y / 9.80665f;
  accelZ = ev.acceleration.z / 9.80665f;

  float mag = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);

  // --- Pedometer: simple magnitude peak detector (350ms refractory) ---
  if (!stepWasHigh && mag > 1.25f && (now - lastStepTime > 350)) {
    stepWasHigh = true;
  } else if (stepWasHigh && mag < 1.05f) {
    stepWasHigh = false;
    if (now - lastStepTime > 350) {
      stepCount++;
      lastStepTime = now;
      if (!goalDone && stepGoal > 0 && stepCount >= stepGoal) {
        goalDone = true; // celebration: rings like an alarm (any button mutes it)
        isBuzzing = true;
        buzzStartTime = now;
      }
      if (stepCount % 50 == 0) saveConfig();
    }
  }
}

void handleMotionEdit(bool isNext) {
  // MOTION card is steps only.
  // In STEPS sub-screen: NEXT = back to counter, PREV = reset counter.
  // In GOAL edit: NEXT = +500, PREV = -500 (wraps MIN<->MAX).
  if (motionState == MOTION_STEPS) {
    if (isNext) {
      motionState = MOTION_OVERVIEW;
    } else {
      stepCount = 0;
      goalDone = false;
      saveConfig();
    }
  } else if (motionState == MOTION_EDIT_GOAL) {
    if (isNext) {
      stepGoal += STEP_GOAL_STEP;
      if (stepGoal > STEP_GOAL_MAX) stepGoal = STEP_GOAL_MIN;
    } else {
      stepGoal -= STEP_GOAL_STEP;
      if (stepGoal < STEP_GOAL_MIN) stepGoal = STEP_GOAL_MAX;
    }
    if (stepCount < stepGoal) goalDone = false; // higher goal = celebrate again
    saveConfig();
  }
}

void checkMidnight(const struct tm &timeinfo) {
  // Midnight step reset: yesterday's result goes into the 7-day history.
  if (timeinfo.tm_yday == lastStepDay) return;
  if (lastStepDay >= 0) {
    for (int i = 6; i > 0; i--) stepHist[i] = stepHist[i - 1];
    stepHist[0] = stepCount;
  }
  stepCount = 0;
  goalDone = false;
  lastStepDay = timeinfo.tm_yday;
  saveConfig();
}

// =================== HARDWARE =================== 
void drawLoadingScreen(int progress) {
  display.clearDisplay();
  display.drawBitmap(-9, -6, image_logo_bits, 64, 64, SH110X_WHITE);
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(2);
  display.setFont(&Org_01);
  display.setCursor(40, 15);
  display.print("CERES.OS");
  display.setTextSize(1);
  display.setCursor(54, 48);
  display.print("LOADING: ");
  display.print(progress);
  display.print("%");
  display.drawRect(49, 26, 73, 10, SH110X_WHITE);
  int barWidth = map(progress, 0, 100, 0, 69);
  if (barWidth > 0) display.fillRect(51, 28, barWidth, 6, SH110X_WHITE);
  display.display();
}

int getBatteryPercentage() {
  static float smoothedVoltage = 0.0;
  long sumAdc = 0;
  for (int i = 0; i < 50; i++) { sumAdc += analogRead(BATTERY_PIN); delayMicroseconds(200); }
  float rawAdc = (float)sumAdc / 50;
  float pinVoltage = (rawAdc / 4095.0) * 3.1;
  float currentBatteryVoltage = pinVoltage * 2.0;
  if (smoothedVoltage < 2.0) smoothedVoltage = currentBatteryVoltage;
  else smoothedVoltage = (smoothedVoltage * 0.90) + (currentBatteryVoltage * 0.10);
  float percentage = ((smoothedVoltage - 3.4) / (4.2 - 3.4)) * 100.0;
  return constrain((int)percentage, 0, 100);
}

void enterDeepSleep() {
  saveConfig(); // keep steps / goal / history / high score before sleep
  display.clearDisplay();
  display.setFont(&Org_01);
  display.setTextSize(1);
  display.setCursor(20, 30);
  display.print("Turning OFF...");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();
  delay(100);
  Wire.end();
  delay(50);
  
  time(&sleepStartTime); 

  pinMode(BUTTON_PIN, INPUT); 
  esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, (gpio_int_type_t)GPIO_MODE_INPUT);
  esp_deep_sleep_start();
}

void handleBuzzer() {
  if (isBuzzing) {
    if ((millis() / 250) % 2 == 0) {
      digitalWrite(BUZZER_PIN, HIGH); 
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
    
    if (millis() - buzzStartTime > 30000) {
      isBuzzing = false;
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// =================== GAME: STARSHOOTER ===================
// GAME card works like ALARMS: after entering, NEXT/PREV still switch cards until PWR enters it.
// gameEntered=false (preview) -> NEXT/PREV = switch card, PWR = enter menu.
// gameEntered=true + !inGame (menu) -> NEXT/PREV = cursor, PWR = select.
// gameEntered=true + inGame (playing) -> NEXT=hold=up, PREV=hold=down, PWR=menu.
// After GAME OVER: PWR=retry, NEXT/PREV=menu (still inside, PWR in game = back to menu).
#define GAME_BULLETS 4
#define GAME_ENEMIES 4
#define GAME_TICK_MS 70
#define GAME_MENU_ROWS 2 // 0=STARSHOOTER, 1=EXIT (add new games as extra rows)
bool inGame = false;
bool gameEntered = false; // false = card preview only (like WATCH_NORMAL), true = inside (like WATCH_EDIT_*)
int gameMenuIdx = 0;
int shipY = 32;
int gameScore = 0;
int gameLives = 3;
bool gameOver = false;
struct GBullet { int x, y; bool on; };
struct GEnemy { int x, y, spd; bool on; };
GBullet gbul[GAME_BULLETS];
GEnemy geno[GAME_ENEMIES];
int starX[12], starY[12];
bool starsInit = false;
unsigned long lastGameTick = 0, lastShot = 0, lastSpawn = 0;

void gameReset() {
  shipY = 32; gameScore = 0; gameLives = 3; gameOver = false;
  for (int i = 0; i < GAME_BULLETS; i++) gbul[i].on = false;
  for (int i = 0; i < GAME_ENEMIES; i++) geno[i].on = false;
  lastShot = 0; lastSpawn = millis(); lastGameTick = 0;
  if (!starsInit) {
    for (int i = 0; i < 12; i++) { starX[i] = random(0, 127); starY[i] = random(12, 52); }
    starsInit = true;
  }
}

void gameFire() {
  for (int i = 0; i < GAME_BULLETS; i++) {
    if (!gbul[i].on) { gbul[i].on = true; gbul[i].x = 14; gbul[i].y = shipY - 1; lastShot = millis(); return; }
  }
}

void drawGame() {
  // FULLSCREEN: game fills the whole 128x64, no frames or arrows. PWR = back to menu.
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setFont(&Org_01);
  display.setTextSize(1);
  for (int i = 0; i < 12; i++) display.drawPixel(starX[i], starY[i], SH110X_WHITE);
  display.setCursor(0, 8); display.print("S"); display.print(gameScore);
  display.setCursor(108, 8); display.print("L"); display.print(gameLives);
  display.fillTriangle(4, shipY - 5, 4, shipY + 5, 13, shipY, SH110X_WHITE); // ship
  for (int i = 0; i < GAME_BULLETS; i++) if (gbul[i].on) display.fillRect(gbul[i].x, gbul[i].y, 4, 2, SH110X_WHITE);
  for (int e = 0; e < GAME_ENEMIES; e++) if (geno[e].on) {
    display.drawRect(geno[e].x, geno[e].y - 3, 6, 6, SH110X_WHITE);
    display.drawPixel(geno[e].x + 2, geno[e].y, SH110X_WHITE);
    display.drawPixel(geno[e].x + 3, geno[e].y, SH110X_WHITE);
  }
  display.setCursor(40, 57); display.print(gameOver ? "PWR=RETRY" : "PWR=EXIT");
  if (gameOver) {
    display.fillRect(35, 20, 58, 28, SH110X_BLACK);
    display.drawRect(35, 20, 58, 28, SH110X_WHITE);
    display.setCursor(37, 29); display.print("GAME OVER");
    display.setCursor(37, 39); display.print("PWR=RETRY");
  }
  display.display();
}

void updateGame(unsigned long now) {
  if (gameOver) return;
  if (now - lastGameTick < GAME_TICK_MS) return;
  lastGameTick = now;
  if (now - lastShot > 350) gameFire(); // auto-fire
  if (now - lastSpawn > 1200) {         // enemy spawn (faster with score)
    lastSpawn = now;
    for (int i = 0; i < GAME_ENEMIES; i++) if (!geno[i].on) {
      geno[i].on = true; geno[i].x = 121; geno[i].y = random(22, 46);
      geno[i].spd = 2 + gameScore / 150; if (geno[i].spd > 5) geno[i].spd = 5;
      break;
    }
  }
  for (int i = 0; i < GAME_BULLETS; i++) if (gbul[i].on) { gbul[i].x += 4; if (gbul[i].x > 126) gbul[i].on = false; }
  for (int e = 0; e < GAME_ENEMIES; e++) if (geno[e].on) {
    geno[e].x -= geno[e].spd;
    if (geno[e].x < 0) { geno[e].on = false; continue; }
    for (int i = 0; i < GAME_BULLETS; i++) if (gbul[i].on &&
        gbul[i].x + 4 >= geno[e].x && gbul[i].x <= geno[e].x + 6 &&
        gbul[i].y + 2 >= geno[e].y - 3 && gbul[i].y <= geno[e].y + 3) {
      gbul[i].on = false; geno[e].on = false; gameScore += 10;
      if (gameScore > bestScore) { bestScore = gameScore; saveConfig(); }
      break;
    }
    if (!geno[e].on) continue;
    if (geno[e].x < 16 && abs(geno[e].y - shipY) < 6) { // rammed the ship
      geno[e].on = false;
      gameLives--;
      if (gameLives <= 0) { gameLives = 0; gameOver = true; }
    }
  }
  for (int i = 0; i < 12; i++) { starX[i]--; if (starX[i] < 0) { starX[i] = 127; starY[i] = random(12, 52); } }
  drawGame();
}

void updateGameInput(bool nextHeld, bool prevHeld, unsigned long now) {
  // Hold-to-steer + buzzer mute on edge (like other cards).
  static bool lastN = false, lastP = false;
  static unsigned long lastSteer = 0;
  bool nEdge = nextHeld && !lastN;
  bool pEdge = prevHeld && !lastP;
  lastN = nextHeld; lastP = prevHeld;
  if ((nEdge || pEdge) && isBuzzing) { isBuzzing = false; digitalWrite(BUZZER_PIN, LOW); return; }
  if (gameOver) return; // after death NEXT/PREV go back to card navigation
  lastActivityTime = now; // playing = activity (inactivity sleep waits)
  if (now - lastSteer < 40) return;
  lastSteer = now;
  if (nextHeld) { shipY += 3; if (shipY > 48) shipY = 48; }
  if (prevHeld) { shipY -= 3; if (shipY < 20) shipY = 20; }
}

void drawGameMenu() {
  // Title menu in other cards' style: frames, arrows, gamepad icon, footer.
  // New game = new row + GAME_MENU_ROWS+1.
  // gameEntered=false -> preview, NEXT/PREV switch cards; true -> cursor, PWR=start.
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setFont(&Org_01);
  display.drawBitmap(0, 0, image_paint_2_bits, 32, 62, SH110X_WHITE);
  display.drawBitmap(95, 0, image_paint_2_copy_1_bits, 29, 62, SH110X_WHITE);
  display.drawBitmap(96, 2,  image_Pin_pointer_bits,        25, 15, SH110X_WHITE);
  display.drawBitmap(96, 43, image_Pin_pointer_copy_1_bits, 25, 15, SH110X_WHITE);
  display.drawBitmap(102, 25, image_gamepad_bits, 11, 8, SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 8); display.print("GAMES");
  display.setCursor(64, 8); display.print("B"); display.print(min(bestScore, 9999));
  display.setCursor(2, 22); display.print(gameMenuIdx == 0 ? ">SHOOTER" : " SHOOTER");
  display.setCursor(2, 32); display.print(gameMenuIdx == 1 ? ">EXIT" : " EXIT");
  display.setCursor(2, 42); display.print(gameEntered ? "PWR=START" : "PWR=ENTER");
  display.setFont(&Org_01); display.setCursor(4, 53); display.print(Sys_ver);
  display.display();
}

// =================== CARDS UI =================== 
void drawCardHome(String timeStr, String dateStr, int batPct, bool wifiConn, bool btConn) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(2);
  display.setFont(&Org_01);
  display.setCursor(2, 17); display.print(timeStr);  
  display.drawBitmap(0, 0, image_paint_2_bits, 32, 62, SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 27); display.print(dateStr);
  display.drawBitmap(96, 2, image_Pin_pointer_bits, 25, 15, SH110X_WHITE);
  display.drawBitmap(96, 43, image_Pin_pointer_copy_1_bits, 25, 15, SH110X_WHITE);
  display.drawBitmap(3, 32, image_battery_empty_bits, 24, 16, SH110X_WHITE);
  display.setFont(NULL); display.setCursor(8, 36);
  if(batPct < 10) display.setCursor(12, 36); 
  display.print(batPct);
  display.setFont(&Org_01);
  display.setCursor(4, 53); display.print(Sys_ver); 
  if (btConn) display.drawBitmap(28, 31, image_bluetooth_bits, 14, 16, SH110X_WHITE);
  if (wifiConn) display.drawBitmap(43, 31, image_wifi_bits, 19, 16, SH110X_WHITE);
  display.drawBitmap(95, 0, image_paint_2_copy_1_bits, 29, 62, SH110X_WHITE);
  display.setCursor(56, 12); display.print("T:"); display.print(currentTemp, 1); display.print(" C");
  display.setCursor(56, 18); display.print("H:"); display.print(currentHum, 0); display.print("%"); 
  display.drawBitmap(101, 21, image_menu_home_bits, 15, 16, SH110X_WHITE);
  display.display();
}

void drawCardWatch(WatchState state) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setFont(&Org_01);
  display.drawBitmap(0, 0, image_paint_2_bits, 32, 62, SH110X_WHITE);
  display.drawBitmap(95, 0, image_paint_2_copy_1_bits, 29, 62, SH110X_WHITE);
  display.drawBitmap(96, 2,  image_Pin_pointer_bits,        25, 15, SH110X_WHITE);
  display.drawBitmap(96, 43, image_Pin_pointer_copy_1_bits, 25, 15, SH110X_WHITE);
  display.drawBitmap(101, 21, image_clock_alarm_bits, 15, 16, SH110X_WHITE);  
  
  display.setTextSize(1);
  bool blink = (millis() / 500) % 2 == 0;
  char buf[8];

  // --- ALARM (Set Hour) ---
  display.setCursor(4, 9);
  display.print(alarmActive ? "ALARM ON" : "ALARM");
  
  display.setCursor(3, 31);
  display.print("Set Hour");
  
  sprintf(buf, "%02d:%02d", alarmHour, alarmMinute);
  if (state == WATCH_EDIT_ALARM_H && blink) sprintf(buf, "  :%02d", alarmMinute);
  else if (state == WATCH_EDIT_ALARM_M && blink) sprintf(buf, "%02d:  ", alarmHour);
  display.setCursor(10, 21);
  display.print(buf);
  
  // --- COUNTDOWN (Set Time) ---
  display.setCursor(48, 31);
  display.print("Set Time");
  
  long remain = 0;
  if (countdownActive) {
      remain = (countdownTargetTime - millis()) / 1000;
      if(remain < 0) remain = 0;
  } else {
      remain = (cdEditM * 60) + cdEditS;
  }
  int rm = remain / 60;
  int rs = remain % 60;
  sprintf(buf, "%02d:%02d", rm, rs);
  
  if (state == WATCH_EDIT_COUNTDOWN_MIN && blink) sprintf(buf, "  :%02d", rs);
  else if (state == WATCH_EDIT_COUNTDOWN_SEC && blink) sprintf(buf, "%02d:  ", rm);
  
  display.setCursor(55, 21);
  display.print(buf);
  
  // --- TIMER (Stopwatch) ---
  display.setCursor(4, 42);
  if (state == WATCH_EDIT_STOPWATCH && blink) display.print(">TIMER<");
  else display.print("TIMER:");
  
  unsigned long swTime = stopwatchElapsed;
  if (stopwatchActive) swTime += (millis() - stopwatchStartTime);
  unsigned long swSecs = swTime / 1000;
  int swM = (swSecs / 60) % 100;
  int swS = swSecs % 60;
  sprintf(buf, "%02d:%02d", swM, swS);
  
  display.setCursor(42, 42);
  display.print(buf);
  
  if (stopwatchActive) {
      display.drawBitmap(76, 36, image_music_pause_bits, 6, 8, SH110X_WHITE); 
  } else {
      display.drawBitmap(76, 36, image_ButtonRightSmall_bits, 3, 5, SH110X_WHITE); 
  }

  display.setFont(&Org_01);
  display.setCursor(4, 53); 
  display.print(Sys_ver);
  display.display();
}

void drawCardSystem(int batPct) {
  uint32_t flashTotal = ESP.getFlashChipSize() / 1024 / 1024;   // MB
  uint32_t sketchSize = ESP.getSketchSize() / 1024/ 1024;       // MB
  uint32_t flashFreeSpace = ESP.getFreeSketchSpace() / 1024;    // KB
  uint32_t ramTotal = ESP.getHeapSize() / 1024;                 // KB
  uint32_t ramFree = ESP.getFreeHeap() / 1024;                  // KB
  uint32_t ramUsed = ramTotal - ramFree;
  #if defined(CONFIG_IDF_TARGET_ESP32)
    const char* chipName = "ESP32";
  #elif defined(CONFIG_IDF_TARGET_ESP32C3)
    const char* chipName = "ESP32-C3";
  #elif defined(CONFIG_IDF_TARGET_ESP32S3)
    const char* chipName = "ESP32-S3";
  #else
    const char* chipName = "Unknown";
  #endif
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setFont(&Org_01);
  display.drawBitmap(0, 0, image_paint_2_bits, 32, 62, SH110X_WHITE);
  display.drawBitmap(95, 0, image_paint_2_copy_1_bits, 29, 62, SH110X_WHITE);
  display.drawBitmap(96, 2,  image_Pin_pointer_bits,        25, 15, SH110X_WHITE);
  display.drawBitmap(96, 43, image_Pin_pointer_copy_1_bits, 25, 15, SH110X_WHITE);
  display.drawBitmap(101, 21, image_menu_settings_sliders_two_bits, 15, 16, SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 8); display.print("Device:" );display.print(chipName);
  display.setCursor(2, 17); display.print("MEMORY:");
  display.drawRect(2, 20, 60, 6, SH110X_WHITE);
  int fillWidth = map(sketchSize, 0, flashTotal, 0, 56);
  display.fillRect(3, 21, fillWidth, 4, SH110X_WHITE);
  display.setCursor(64, 22); display.print(sketchSize);display.print("/");display.print(flashTotal); display.print("MB");
  display.setCursor(2, 33); display.print("BAT:"); display.print(batPct); display.print("%");
  display.setCursor(2, 42); display.print("UPTIME: ");
  unsigned long sec = millis() / 1000;
  char upbuf[12];
  sprintf(upbuf, "%02lu:%02lu:%02lu", sec/3600, (sec%3600)/60, sec%60);
  display.setCursor(38, 42); display.print(upbuf);
  display.setFont(&Org_01); display.setCursor(4, 53); display.print(Sys_ver); 
  display.display();
}

void drawCardWeather() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setFont(&Org_01);
  display.drawBitmap(0, 0, image_paint_2_bits, 32, 62, SH110X_WHITE);
  display.drawBitmap(95, 0, image_paint_2_copy_1_bits, 29, 62, SH110X_WHITE);
  display.drawBitmap(96, 2,  image_Pin_pointer_bits,        25, 15, SH110X_WHITE);
  display.drawBitmap(96, 43, image_Pin_pointer_copy_1_bits, 25, 15, SH110X_WHITE);
  display.drawBitmap(98, 21, image_weather_cloud_sunny_bits, 24, 16, SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 8); 
  display.print("WEATHER");
  display.setCursor(2, 20); 
  display.print("Temp: ");
  display.print(currentTemp, 1);
  display.print(" C");
  display.setCursor(2, 28); 
  display.print("Hum:  ");
  display.print(currentHum);
  display.print(" %");
  display.setCursor(2, 38); 
  if (isWifiConnected) {
      display.print("Sync: OK");
  } else {
      display.print("No Wi-Fi");
  }
  display.setFont(&Org_01); 
  display.setCursor(4, 53); 
  display.print(Sys_ver); 
  display.display();
}

void drawCardMotion() {
  // Same style as other cards: paint_2 frames, arrows, 15x16 icon, Sys_ver footer, Org_01 font.
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setFont(&Org_01);
  display.drawBitmap(0, 0, image_paint_2_bits, 32, 62, SH110X_WHITE);
  display.drawBitmap(95, 0, image_paint_2_copy_1_bits, 29, 62, SH110X_WHITE);
  display.drawBitmap(96, 2,  image_Pin_pointer_bits,        25, 15, SH110X_WHITE);
  display.drawBitmap(96, 43, image_Pin_pointer_copy_1_bits, 25, 15, SH110X_WHITE);
  display.drawBitmap(101, 21, image_motion_bits, 15, 16, SH110X_WHITE);
  display.setTextSize(1);

  if (!accelAvailable) {
    display.setCursor(2, 8);  display.print("MOTION");
    display.setCursor(2, 20); display.print("No ADXL345!");
    display.setCursor(2, 28); display.print("Check SDA=6");
    display.setCursor(2, 36); display.print("SCL=7 0x53");
    display.setFont(&Org_01); display.setCursor(4, 53); display.print(Sys_ver);
    display.display();
    return;
  }

  if (motionState == MOTION_STEPS) {
    // STEPS: distance/kcal sub-screen. PREV=reset, NEXT=steps.
    display.setCursor(2, 8); display.print("STEPS");
    float km = stepCount * 0.00075f;   // ~75cm/step
    float kcal = stepCount * 0.04f;
    display.setCursor(34, 18); display.print(km, 2); display.print("km");
    display.setCursor(34, 28); display.print(kcal, 0); display.print("kcal");
    display.setCursor(2, 36); display.print("Goal "); display.print(stepGoal);
    display.drawRect(2, 42, 90, 5, SH110X_WHITE);
    int fill = map(constrain(stepCount, 0, stepGoal), 0, stepGoal, 0, 86);
    if (fill > 0) display.fillRect(4, 43, fill, 3, SH110X_WHITE);
    display.setCursor(68, 8); display.print("PRV:RST");
  } else if (motionState == MOTION_EDIT_GOAL) {
    // GOAL EDIT: NEXT=+500, PREV=-500 (wraps). Blinking like WATCH edit.
    display.setCursor(2, 8); display.print("GOAL");
    bool blink = (millis() / 500) % 2 == 0;
    display.setTextSize(2);
    if (blink) { display.setCursor(34, 22); display.print(stepGoal); }
    display.setTextSize(1);
    display.setCursor(2, 36); display.print("N:+500");
    display.setCursor(56, 36); display.print("P:-500");
    display.drawRect(2, 42, 90, 5, SH110X_WHITE);
    int gfill = map(stepGoal, STEP_GOAL_MIN, STEP_GOAL_MAX, 0, 86);
    if (gfill > 0) display.fillRect(4, 43, gfill, 3, SH110X_WHITE);
  } else if (motionState == MOTION_HIST) {
    // HISTORY: 7 days (oldest->newest) + today as 8th bar, TOT at bottom.
    display.setCursor(2, 8); display.print("HIST 7D");
    int hvals[8];
    for (int i = 0; i < 7; i++) hvals[i] = stepHist[6 - i];
    hvals[7] = stepCount;
    int hmax = 1;
    long htot = 0;
    for (int i = 0; i < 8; i++) { if (hvals[i] > hmax) hmax = hvals[i]; htot += hvals[i]; }
    for (int i = 0; i < 8; i++) {
      int hh = (hvals[i] * 24) / hmax;
      if (hvals[i] > 0 && hh < 2) hh = 2;
      if (hh > 0) display.fillRect(36 + i * 7, 40 - hh, 5, hh, SH110X_WHITE);
    }
    display.setCursor(2, 46); display.print("TOT "); display.print(htot);
  } else {
    // OVERVIEW: big step counter + goal bar. PWR->sub-screen, NEXT/PREV->switch card.
    display.setCursor(2, 8); display.print("STEPS");
    display.setTextSize(2);
    display.setCursor(34, 22); display.print(stepCount);
    display.setTextSize(1);
    display.setCursor(2, 36);
    display.print("Goal "); display.print(stepGoal);
    display.print(" "); display.print(stepCount * 100 / stepGoal); display.print("%");
    display.drawRect(2, 42, 90, 5, SH110X_WHITE);
    int fill = map(constrain(stepCount, 0, stepGoal), 0, stepGoal, 0, 86);
    if (fill > 0) display.fillRect(4, 43, fill, 3, SH110X_WHITE);
  }
  display.setFont(&Org_01); display.setCursor(4, 53); display.print(Sys_ver);
  display.display();
}

void drawCard(String timeStr, String dateStr, int batPct, bool wifiConn, bool btConn) {
  switch (currentCard) {
    case CARD_HOME:     drawCardHome(timeStr, dateStr, batPct, wifiConn, btConn); break;
    case CARD_WATCH:    drawCardWatch(watchState); break;
    case CARD_WEATHER:  drawCardWeather(); break;
    case CARD_MOTION:   drawCardMotion(); break;
    case CARD_GAME:     if (inGame) drawGame(); else drawGameMenu(); break;
    case CARD_SYSTEM:   drawCardSystem(batPct); break;
    default: drawCardHome(timeStr, dateStr, batPct, wifiConn, btConn); break;
  }
}
// =================== EDITING MODE =================== 
void handleWatchEdit(bool isNext) {
    int dir = isNext ? 1 : -1;
    switch (watchState) {
        case WATCH_EDIT_ALARM_H:
            alarmHour = (alarmHour + dir + 24) % 24;
            break;
        case WATCH_EDIT_ALARM_M:
            alarmMinute = (alarmMinute + dir + 60) % 60;
            break;
        case WATCH_EDIT_COUNTDOWN_MIN:
            cdEditM = (cdEditM + dir);
            if(cdEditM < 0) cdEditM = 99;
            if(cdEditM > 99) cdEditM = 0;
            break;
        case WATCH_EDIT_COUNTDOWN_SEC:
            cdEditS = (cdEditS + dir + 60) % 60;
            break;
        case WATCH_EDIT_STOPWATCH:
            if (isNext) { 
                if (stopwatchActive) { // Pause
                    stopwatchActive = false;
                    stopwatchElapsed += (millis() - stopwatchStartTime);
                } else {               // Start
                    stopwatchActive = true;
                    stopwatchStartTime = millis();
                }
            } else { // Reset on PREV
                stopwatchActive = false;
                stopwatchElapsed = 0;
            }
            break;
        default: break;
    }
}

void handleShortPress() {
    if (isBuzzing) {
         isBuzzing = false;
         digitalWrite(BUZZER_PIN, LOW);
         return;
    }

    if (currentCard == CARD_MOTION) {
        // Short PWR: steps -> distance/kcal -> goal edit -> 7d history (like WATCH edit)
        motionState = (MotionState)((motionState + 1) % 4);
        return;
    }

    if (currentCard == CARD_GAME && gameEntered && inGame && gameOver) {
        gameReset(); // PWR after death = retry
        drawGame();
        return;
    }
    if (currentCard == CARD_GAME && gameEntered && inGame) {
        inGame = false; // PWR mid-game = back to title menu (still inside)
        drawGameMenu();
        return;
    }
    if (currentCard == CARD_GAME && gameEntered && !inGame) {
        if (gameMenuIdx == 0) { inGame = true; gameReset(); drawGame(); } // PWR = play
        else { gameEntered = false; inGame = false; drawGameMenu(); }     // EXIT -> leave to preview (like WATCH_NORMAL), NEXT/PREV switch cards again
        return;
    }
    if (currentCard == CARD_GAME && !gameEntered) {
        gameEntered = true; // 1st PWR on game card = enter (like entering alarm edit)
        inGame = false;
        gameMenuIdx = 0;
        drawGameMenu();
        return;
    }

    if (currentCard != CARD_WATCH) {
        currentCard = CARD_WATCH;
        watchState = WATCH_NORMAL;
    } else {
        WatchState oldState = watchState;
        watchState = (WatchState)((watchState + 1) % 6);
        
        if (watchState == WATCH_EDIT_ALARM_H) {
             alarmActive = true; 
        }
        if (oldState == WATCH_EDIT_COUNTDOWN_SEC) {
            if (cdEditM > 0 || cdEditS > 0) {
                countdownActive = true;
                countdownTargetTime = millis() + (cdEditM * 60000UL) + (cdEditS * 1000UL);
            } else {
                countdownActive = false;
            }
        }
        if (watchState == WATCH_EDIT_COUNTDOWN_MIN) {
             countdownActive = false; 
        }
    }
}

// =================== WI-FI & WEATHER =================== 
int loadWifiNetworkCount() { wifiPrefs.begin("wifi", true); int count = wifiPrefs.getInt("count", 0); wifiPrefs.end(); return count; }
bool buildWifiMulti() { wifiPrefs.begin("wifi", true); int count = wifiPrefs.getInt("count", 0); bool any = false; for (int i = 0; i < count && i < MAX_WIFI_NETWORKS; i++) { String s = wifiPrefs.getString(("ssid" + String(i)).c_str(), ""); String p = wifiPrefs.getString(("pass" + String(i)).c_str(), ""); if (s.length() > 0) { wifiMulti.addAP(s.c_str(), p.c_str()); any = true; } } wifiPrefs.end(); return any; }
void addWifiNetwork(const String &newSsid, const String &newPass) { wifiPrefs.begin("wifi", false); int count = wifiPrefs.getInt("count", 0); int nextSlot = wifiPrefs.getInt("nextSlot", 0); bool updated = false; for (int i = 0; i < count && i < MAX_WIFI_NETWORKS; i++) { String existing = wifiPrefs.getString(("ssid" + String(i)).c_str(), ""); if (existing == newSsid) { wifiPrefs.putString(("pass" + String(i)).c_str(), newPass); updated = true; break; } } if (!updated) { int slot; if (count < MAX_WIFI_NETWORKS) { slot = count; count++; wifiPrefs.putInt("count", count); } else { slot = nextSlot; nextSlot = (nextSlot + 1) % MAX_WIFI_NETWORKS; wifiPrefs.putInt("nextSlot", nextSlot); } wifiPrefs.putString(("ssid" + String(slot)).c_str(), newSsid); wifiPrefs.putString(("pass" + String(slot)).c_str(), newPass); } wifiPrefs.end(); }
void startWifiConnectAsync() { WiFi.mode(WIFI_STA); wifiCredentialsExist = buildWifiMulti(); if (wifiCredentialsExist) wifiMulti.run(); lastWifiRetry = millis(); }
void handleWifiNonBlocking() { unsigned long now = millis(); if (WiFi.status() == WL_CONNECTED) { if (!isWifiConnected) { isWifiConnected = true; configTzTime(TZ_INFO, ntpServer); saveConfig(); } return; } isWifiConnected = false; if (wifiCredentialsExist && (now - lastWifiRetry > WIFI_RETRY_INTERVAL)) { lastWifiRetry = now; wifiMulti.run(1500); } }
void drawWifiConfigScreen() { display.clearDisplay(); display.setTextColor(SH110X_WHITE); display.setFont(&Org_01); display.setTextSize(1); display.setCursor(4, 12); display.print("WIFI Configuration"); display.setCursor(4, 26); display.print("Connect Your Phone"); display.setCursor(4, 34); display.print("With this network:"); display.setCursor(4, 46); display.print(AP_CONFIG_NAME); display.setCursor(4, 58); display.print("Go to  website: 192.168.4.1"); display.display(); }
void startWifiConfigPortal() { drawWifiConfigScreen(); WiFi.disconnect(true, true); delay(200); WiFi.mode(WIFI_AP_STA); delay(200); WiFiManager wm; wm.setDebugOutput(false); wm.setConfigPortalTimeout(180); bool connected = wm.startConfigPortal(AP_CONFIG_NAME); if (connected) { addWifiNetwork(WiFi.SSID(), WiFi.psk()); wifiCredentialsExist = true; isWifiConnected = true; buildWifiMulti(); configTzTime(TZ_INFO, ntpServer); saveConfig(); } lastActivityTime = millis(); }

void updateWeather() {
  lastWeatherUpdate = millis(); 
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http; String latitude = "52.2297"; String longitude = "21.0122";
  http.begin("http://ip-api.com/json/?fields=status,lat,lon");
  int httpCode = http.GET();
  if (httpCode == 200) {
    StaticJsonDocument<256> locDoc;
    if (!deserializeJson(locDoc, http.getString()) && locDoc["status"] == "success") { latitude = locDoc["lat"].as<String>(); longitude = locDoc["lon"].as<String>(); }
  }
  http.end(); 
  String url = "http://api.open-meteo.com/v1/forecast?latitude=" + latitude + "&longitude=" + longitude + "&current=temperature_2m,relative_humidity_2m&timezone=auto";
  http.begin(url);
  if (http.GET() == 200) {
    StaticJsonDocument<512> weatherDoc;
    if (!deserializeJson(weatherDoc, http.getString())) { currentTemp = weatherDoc["current"]["temperature_2m"]; currentHum = weatherDoc["current"]["relative_humidity_2m"]; saveConfig(); }
  }
  http.end();
}

// =================== SETUP & LOOP =================== 
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_NEXT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PREV_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN,      INPUT_PULLUP);
  
  //active buzzer configuration
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  rgbLed.begin();
  rgbLed.clear();
  rgbLed.show();

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED) {
    time_t nowTime; time(&nowTime);
    if (sleepStartTime > 0 && nowTime > sleepStartTime) totalSleepTimeSeconds += (nowTime - sleepStartTime);
  }
  if (!LittleFS.begin(true)) Serial.println("Error LittleFS!");
  loadConfig(); 
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Wire.begin(I2C_SDA, I2C_SCL);
  initAccel(); // restores Wire.begin(6,7) inside
  Wire.begin(I2C_SDA, I2C_SCL); // extra guard against pin reset by Adafruit lib
  if (!display.begin(I2C_ADDRESS, true)) while (true);
  drawLoadingScreen(20); 
  startWifiConnectAsync(); 
  unsigned long wifiWaitStart = millis();
  int progress = 20;
  
  while (WiFi.status() != WL_CONNECTED && millis() - wifiWaitStart < 8000) {
    delay(250);
    if (progress < 75) {
      progress += 3;
      drawLoadingScreen(progress);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    isWifiConnected = true;
    drawLoadingScreen(80);
    
    configTzTime(TZ_INFO, ntpServer); 
    updateWeather();                  
    drawLoadingScreen(95);
    
    saveConfig();
  }
  
  drawLoadingScreen(100);
  delay(400);
  randomSeed(analogRead(BATTERY_PIN) + micros()); // randomness for game starfield
  lastActivityTime = millis();
}

void loop() {
  unsigned long now = millis();
  bool nextHeld = (digitalRead(BUTTON_NEXT_PIN) == LOW);
  bool prevHeld = (digitalRead(BUTTON_PREV_PIN) == LOW);
  
  handleBuzzer();
  updateAccel(); // pedometer only, doesn't touch cards or sleep

  if (countdownActive && now >= countdownTargetTime) {
      countdownActive = false;
      isBuzzing = true;
      buzzStartTime = now;
  }
  
  if (nextHeld && prevHeld) {
    if (!comboPressActive) {
      comboPressActive = true;
      comboPressStartTime = now;
    } else if (now - comboPressStartTime > COMBO_HOLD_MS) {
      comboPressActive = false;
      startWifiConfigPortal();
      return; 
    }
  } else {
    comboPressActive = false;
    if (nextHeld && now - lastNextPress > DEBOUNCE_MS) {
      lastNextPress = now;
      lastActivityTime = now;
      
      if (isBuzzing) { isBuzzing = false; digitalWrite(BUZZER_PIN, LOW); }
      else if (currentCard == CARD_WATCH && watchState != WATCH_NORMAL) handleWatchEdit(true);
      else if (currentCard == CARD_MOTION && (motionState == MOTION_STEPS || motionState == MOTION_EDIT_GOAL)) handleMotionEdit(true);
      else if (currentCard == CARD_GAME && gameEntered && inGame && !gameOver) { /* steering in updateGameInput */ }
      else if (currentCard == CARD_GAME && gameEntered && !inGame) { gameMenuIdx = (gameMenuIdx + 1) % GAME_MENU_ROWS; drawGameMenu(); }
      else if (currentCard == CARD_GAME && gameEntered && gameOver) { inGame = false; drawGameMenu(); }
      else { currentCard = (currentCard + 1) % CARD_COUNT; motionState = MOTION_OVERVIEW; gameEntered = false; inGame = false; }
    }
    
    if (prevHeld && now - lastPrevPress > DEBOUNCE_MS) {
      lastPrevPress = now;
      lastActivityTime = now;
      
      if (isBuzzing) { isBuzzing = false; digitalWrite(BUZZER_PIN, LOW); }
      else if (currentCard == CARD_WATCH && watchState != WATCH_NORMAL) handleWatchEdit(false);
      else if (currentCard == CARD_MOTION && (motionState == MOTION_STEPS || motionState == MOTION_EDIT_GOAL)) handleMotionEdit(false);
      else if (currentCard == CARD_GAME && gameEntered && inGame && !gameOver) { /* steering in updateGameInput */ }
      else if (currentCard == CARD_GAME && gameEntered && !inGame) { gameMenuIdx = (gameMenuIdx - 1 + GAME_MENU_ROWS) % GAME_MENU_ROWS; drawGameMenu(); }
      else if (currentCard == CARD_GAME && gameEntered && gameOver) { inGame = false; drawGameMenu(); }
      else { currentCard = (currentCard - 1 + CARD_COUNT) % CARD_COUNT; motionState = MOTION_OVERVIEW; gameEntered = false; inGame = false; }
    }
  }
  
  if (digitalRead(BUTTON_PIN) == LOW) {
        if (!isPressed) {
            isPressed = true;
            buttonPressedTime = now;
            lastActivityTime = now; 
        } else if (now - buttonPressedTime > 2000) {
            enterDeepSleep();
            while(digitalRead(BUTTON_PIN) == LOW) delay(10);
        }
  } else {
        if (isPressed) {
            unsigned long pressDuration = now - buttonPressedTime;
            if (pressDuration > 50 && pressDuration < 1000) {
                handleShortPress();
            }
            isPressed = false;
        }
  }
    
  if (now - lastActivityTime > INACTIVITY_TIMEOUT) {
    enterDeepSleep();           
  }
  
  handleWifiNonBlocking();

  if (now - lastBatteryUpdate > 10000 || lastBatteryUpdate == 0) {
      currentBatteryPct = getBatteryPercentage();
      lastBatteryUpdate = now;
  }

  if (now - lastDisplayUpdate > 200) { 
      lastDisplayUpdate = now;
      bool wifiStatus = isWifiConnected;
      bool btStatus   = false;
      struct tm timeinfo;
      
      static int lastMinuteChecked = -1;
      
      if (getLocalTime(&timeinfo, 10)) { 
        checkMidnight(timeinfo); // midnight step reset + 7d history
        if (alarmActive && timeinfo.tm_min != lastMinuteChecked) {
            if (timeinfo.tm_hour == alarmHour && timeinfo.tm_min == alarmMinute) {
                isBuzzing = true;
                buzzStartTime = now;
                alarmActive = false;
            }
            lastMinuteChecked = timeinfo.tm_min;
        }

        char bufTime[6], bufDate[11];
        sprintf(bufTime, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        sprintf(bufDate, "%02d.%02d.%04d",
                timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
        if (currentCard != CARD_GAME) drawCard(String(bufTime), String(bufDate), currentBatteryPct, wifiStatus, btStatus);
      } else {
        if (currentCard != CARD_GAME) drawCard("--:--", "--.--.----", currentBatteryPct, wifiStatus, btStatus);
      }
  }

  static int lastCardSeen = -1;
  if (currentCard != lastCardSeen) {
    lastCardSeen = currentCard;
    if (currentCard == CARD_GAME) { inGame = false; gameEntered = false; gameMenuIdx = 0; drawGameMenu(); } // NEXT/PREV entry = preview only, PWR enters (like alarms)
  }
  if (currentCard == CARD_GAME && gameEntered) {
    updateGameInput(nextHeld, prevHeld, now);
    if (inGame && !gameOver) updateGame(now);
  }

  if (now - lastWeatherUpdate > WEATHER_UPDATE_INTERVAL){
    updateWeather();
  }
}
