
# CERES.OS
Small operating system for diy smartwatches based on esp32c3 and others
<img width="1584" height="672" alt="ceres,os_banner" src="https://github.com/user-attachments/assets/c90ee43b-01e0-4130-83e8-d0e1ff2db6fd" />

CERES_OS is a custom-built, ultra-lightweight operating system tailored for ESP32-C3 smartwatches equipped with an OLED display. Built around a clean, multi-card user interface, CERES_OS delivers essential wearable functionality with minimal overhead.  

# Key Features:
 - Card-Based UI Navigation: Seamlessly cycle between the Home Screen, Watch Tools (Alarm, Countdown Timer, Stopwatch), Weather Dashboard, and System Status.
 - Automatic Time & Weather Sync: Fetches real-time time via NTP and accurate local weather metrics using non-blocking HTTP requests.
 - Power Management: Integrated ESP32 deep sleep support with quick wake-up handling and dynamic display time-outs.
 - Wi-Fi Manager: Easy setup portal for connecting to local Wi-Fi networks without hardcoding credentials.
## Versions
- v0.4.2 (stable) – 4 cards: Home, Watch tools, Weather, System → see Releases
- v1.1 (current) – adds Motion pedometer (ADXL345) + fullscreen Starshooter
  
Following program was developed with Gemini AI assistance

UI was designed using lopaka.app: https://lopaka.app/gallery 

# Hardware Support: 
Built-in drivers for Adafruit SH1106 OLED displays, WS2812 RGB LEDs, active buzzers, and analog battery monitoring. 

Parts used to create prototype are available in Parts folder
<img width="702" height="534" alt="prototype1" src="https://github.com/user-attachments/assets/2e700ae3-15ed-4b98-bc9e-9b3245e46eac" />
<img width="838" height="896" alt="prototype 2" src="https://github.com/user-attachments/assets/02863117-2f23-4929-b5fe-0c20792d9f94" />

 # Technical Notes & Hardware Required: 

ESP32-C3 microcontroller, 128x64 I2C OLED display (SH1106G), Active Buzzer, and standard push buttons.  
<img width="1132" height="599" alt="smartwach_circuit" src="https://github.com/user-attachments/assets/19b7ccc3-ed1e-4bd2-b609-7dbe81aa84dc" />

Circuit was made using https://app.cirkitdesigner.com 

<img width="2048" height="944" alt="18694c96-e9a9-4412-9aeb-9ef871dc4e35" src="https://github.com/user-attachments/assets/1f439288-3aff-4867-a7bc-5eaca23c2067" />

Core Libraries Required: Adafruit_SH110X, Adafruit_GFX, WiFiManager, ArduinoJson, Adafruit_NeoPixel, and LittleFS.  

My Esp32c3 had build in rgb led that interfered with other functions, that's why I added libraries to turn it off

# Configuration and instalation: 
Before flashing, set your default location coordinates (String latitude = "**.****"; String longitude = "**.****";) at the top of the main code file to fetch accurate weather data for your region.  
Using Arduino IDE, change dev board options in tools to ones showed on image:

<img width="360" height="315" alt="image" src="https://github.com/user-attachments/assets/f0e6edce-783b-4d21-9a0a-b784f35e2b11" />

# WI-fi configurations 

To configure WI-FI, press Preview and next button simultaneously for a few seconds and follow next instructions on display.
