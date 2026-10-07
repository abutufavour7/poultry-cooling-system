/*
 * ============================================================
 *   POULTRY FARM TEMPERATURE & HUMIDITY CONTROL SYSTEM
 *   Final Year Project — FUT Minna
 *   Platform: ESP32 + Blynk IoT
 * ============================================================
 *  HARDWARE:
 *    - ESP32 DevKit
 *    - DHT22 sensor          → GPIO4
 *    - HC-SR04 ultrasonic    → TRIG: GPIO5, ECHO: GPIO18
 *    - MOSFET 1 (fans)       → GPIO25 (PWM)
 *    - MOSFET 2 (exhaust)    → GPIO26 (PWM)
 *    - MOSFET 3 (humidifier) → GPIO27 (ON/OFF)
 *    - Relay (pump)          → GPIO14 (ON/OFF)
 *    - Age set button        → GPIO32
 *    - Display button        → GPIO33
 *    - Reset button          → EN pin (hardware)
 *    - LCD 16x2 I2C          → SDA: GPIO21, SCL: GPIO22
 * ============================================================
 *  BLYNK DATASTREAMS:
 *    V0  → Humidity display
 *    V1  → Temperature display
 *    V2  → Fan speed display (%)
 *    V3  → Exhaust speed display (%)
 *    V4  → Pump status (0/1)
 *    V5  → Humidifier status (0/1)
 *    V6  → Force pump ON (0/1) from app
 *    V7  → Fan speed slider (%) from app
 *    V8  → Exhaust speed slider (%) from app
 *    V9  → System status (0/1)
 *    V11 → Age set button from app (0/2)
 *    V12 → Age set display string
 *    V13 → Force humidifier ON (0/1) from app
 * ============================================================
 *  TANK DEPTH: ~7cm (sensor has 2cm blind spot)
 *    Full  → distance < 6cm → pump OFF
 *    Empty → distance > 6cm → pump ON
 * ============================================================
 *  AGE GROUPS AND THI SETPOINTS:
 *    Group 1: Week 0-2  → THI setpoint 82.5
 *    Group 2: Week 2-4  → THI setpoint 74.5
 *    Group 3: Week 4+   → THI setpoint 68.5
 * ============================================================
 *  ROOM CONDITION LOGIC:
 *    THI > Setpoint (HEAT):
 *      Humidity HIGH → Humid Heat → Fans PID, Exhaust FULL, Humidifier OFF
 *      Humidity LOW  → Dry Heat   → Fans PID, Exhaust OFF,  Humidifier ON
 *    THI < Setpoint (COOL):
 *      Humidity HIGH → Humid Cold → Fans OFF, Exhaust 25%,  Humidifier OFF
 *      Humidity LOW  → Optimal    → Fans OFF, Exhaust OFF,  Humidifier OFF
 * ============================================================
 *  THI FORMULA:
 *    THI = 0.8*T + (RH/100)*(T - 14.4) + 46.4
 * ============================================================
 *  LIBRARIES REQUIRED:
 *    - DHT sensor library by Adafruit
 *    - Adafruit Unified Sensor by Adafruit
 *    - LiquidCrystal_I2C by Frank de Brabander
 *    - Blynk by Volodymyr Shymanskyy
 * ============================================================
 */

// ============================================================
//  BLYNK CONFIGURATION
// ============================================================
#define BLYNK_TEMPLATE_ID    "TMPL2n2QQZhLW"
#define BLYNK_TEMPLATE_NAME  "Poultry Control"
#define BLYNK_AUTH_TOKEN     "_y2ShOJsjOS567XTtEvOS-c9Vxruc80k"
#define BLYNK_PRINT          Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
//  WiFi CREDENTIALS
// ============================================================
#define WIFI_SSID     "EXE 3.0"
#define WIFI_PASSWORD "11118888"

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define DHT_PIN          4
#define TRIG_PIN         5
#define ECHO_PIN         18
#define FAN_PWM_PIN      25
#define EXHAUST_PWM_PIN  26
#define HUMIDIFIER_PIN   27
#define PUMP_PIN         14
#define AGE_BTN_PIN      32
#define DISPLAY_BTN_PIN  33

// ============================================================
//  PWM CONFIGURATION
// ============================================================
#define PWM_FREQ         5000
#define PWM_RESOLUTION   8

// ============================================================
//  SENSOR AND DISPLAY CONFIGURATION
// ============================================================
#define DHT_TYPE         DHT22
#define LCD_ADDRESS      0x27
#define LCD_COLS         16
#define LCD_ROWS         2
#define SDA_PIN          21
#define SCL_PIN          22

// ============================================================
//  WATER LEVEL THRESHOLDS
//  Sensor mounted at top looking down.
//  Small distance = full tank, large distance = empty tank.
//  Hysteresis band (3-5cm) prevents pump chatter at the boundary.
// ============================================================
#define TANK_FULL_CM     3.0   // pump OFF when distance <= 3cm (tank full)
#define TANK_EMPTY_CM    5.0   // pump ON when distance > 5cm (tank empty)

// ============================================================
//  HUMIDITY THRESHOLDS
// ============================================================
#define HUMIDITY_LOW     60.0
#define HUMIDITY_HIGH    70.0

// ============================================================
//  PWM LIMITS
// ============================================================
#define PWM_MIN          0
#define PWM_MAX          255

// ============================================================
//  TIMING
// ============================================================
#define SENSOR_INTERVAL    2000
#define BLYNK_INTERVAL     3000
#define DEBOUNCE_DELAY     200
#define ULTRASONIC_TIMEOUT 3000
#define COOLDOWN_MS        120000  // 2 minutes cooldown after manual override

// ============================================================
//  AGE GROUP THI SETPOINTS
// ============================================================
const float THI_SETPOINTS[3] = {
  82.5,   // Group 1 — Week 0 to 2
  74.5,   // Group 2 — Week 2 to 4
  68.5    // Group 3 — Week 4 and above
};

const char* AGE_LABELS[3] = {
  "Wk 0-2",
  "Wk 2-4",
  "Wk 4+  "
};

// ============================================================
//  PID TUNING CONSTANTS
// ============================================================
const float Kp = 200;
const float Ki = 1.02;
const float Kd = 0;

const float INTEGRAL_MAX =  1000.0;
const float INTEGRAL_MIN = -1000.0;

// ============================================================
//  GLOBAL VARIABLES — Sensor Readings
// ============================================================
float temperature    = 0.0;
float humidity       = 0.0;
float thi            = 0.0;
float waterDistance  = 0.0;

// ============================================================
//  GLOBAL VARIABLES — Control State
// ============================================================
int   currentGroup   = 0;
float setpoint       = 82.5;

float pidError       = 0.0;
float lastError      = 0.0;
float integral       = 0.0;
float derivative     = 0.0;
float pidOutput      = 0.0;

int   fanPWM         = 0;
int   exhaustPWM     = 0;

bool  humidifierON   = false;
bool  pumpON         = false;
bool  displayNeedsUpdate = false;

// ============================================================
//  GLOBAL VARIABLES — Manual Override Controls
// ============================================================
bool  forcePumpON        = false;  // V6
int   appFanSpeed        = -1;     // V7: -1=auto, 0-100=manual %
int   appExhaustSpeed    = -1;     // V8: -1=auto, 0-100=manual %
bool  forceHumidifierON  = false;  // V13

// Cooldown tracking — after manual override is released
// fans ramp down gradually over COOLDOWN_MS
bool  fanCooldownActive     = false;
bool  exhaustCooldownActive = false;
unsigned long fanCooldownStart     = 0;
unsigned long exhaustCooldownStart = 0;
int   fanPWMAtCooldownStart     = 0;
int   exhaustPWMAtCooldownStart = 0;

// ============================================================
//  GLOBAL VARIABLES — Timing
// ============================================================
unsigned long lastSensorTime     = 0;
unsigned long lastBlynkSendTime  = 0;
unsigned long lastAgeBtnTime     = 0;
unsigned long lastDisplayBtnTime = 0;

// ============================================================
//  GLOBAL VARIABLES — Display
// ============================================================
int displayState = 0;

// ============================================================
//  OBJECTS
// ============================================================
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
void connectWiFiBlynk();
void readSensors();
float readUltrasonic();
void computeTHI();
void runConditionControl();
void runPumpControl();
void runCooldown(unsigned long currentTime);
void sendToBlynk();
void handleButtons(unsigned long currentTime);
void cycleAgeGroup();
void showDisplayPage1();
void showDisplayPage2();
void showAgeConfirmation();
void printSerialDebug();

// ============================================================
//  BLYNK — RECEIVE FROM APP
// ============================================================

// V6 — Force Pump ON/OFF from app
BLYNK_WRITE(V6) {
  forcePumpON = param.asInt();
  if (forcePumpON) {
    pumpON = true;
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("BLYNK: Force Pump ON");
  } else {
    pumpON = false;
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("BLYNK: Pump returned to auto");
  }
}

// V7 — Fan Speed Slider from app (0-100%)
// Slider at 0 = release manual override, PID takes back control
// Slider > 0 = manual override active
BLYNK_WRITE(V7) {
  int val = param.asInt();

  if (val <= 0 && appFanSpeed != -1) {
    // Release override — start 2 minute cooldown
    appFanSpeed = -1;
    fanCooldownActive   = true;
    fanCooldownStart    = millis();
    fanPWMAtCooldownStart = fanPWM;
    Serial.println("BLYNK: Fan override released — cooldown started");
  } else if (val > 0) {
    // Manual override active
    fanCooldownActive = false;  // cancel any active cooldown
    appFanSpeed = val;
    fanPWM = map(appFanSpeed, 0, 100, 0, 255);
    ledcWrite(FAN_PWM_PIN, fanPWM);
    Serial.print("BLYNK: Fan manual → ");
    Serial.print(appFanSpeed);
    Serial.println("%");
  }
}

// V8 — Exhaust Speed Slider from app (0-100%)
BLYNK_WRITE(V8) {
  int val = param.asInt();

  if (val <= 0 && appExhaustSpeed != -1) {
    // Release override — start 2 minute cooldown
    appExhaustSpeed = -1;
    exhaustCooldownActive   = true;
    exhaustCooldownStart    = millis();
    exhaustPWMAtCooldownStart = exhaustPWM;
    Serial.println("BLYNK: Exhaust override released — cooldown started");
  } else if (val > 0) {
    exhaustCooldownActive = false;
    appExhaustSpeed = val;
    exhaustPWM = map(appExhaustSpeed, 0, 100, 0, 255);
    ledcWrite(EXHAUST_PWM_PIN, exhaustPWM);
    Serial.print("BLYNK: Exhaust manual → ");
    Serial.print(appExhaustSpeed);
    Serial.println("%");
  }
}
// V13 — Force Humidifier ON from app
BLYNK_WRITE(V13) {
  forceHumidifierON = param.asInt();
  Serial.print("V13 received: ");
  Serial.println(forceHumidifierON);

  if (forceHumidifierON) {
    humidifierON = true;
    digitalWrite(HUMIDIFIER_PIN, HIGH);
    Serial.println("BLYNK: Force Humidifier ON");
  } else {
    humidifierON = false;
    digitalWrite(HUMIDIFIER_PIN, LOW);
    Serial.println("BLYNK: Humidifier back to auto");
  }
}

// V11 — Age Set Button from app (Push mode, 0/2)
// Each press cycles to next age group
BLYNK_WRITE(V11) {
  int val = param.asInt();
  if (val > 0) {
    // Button pressed — cycle age group
    cycleAgeGroup();
  }
}

// ============================================================
//  CYCLE AGE GROUP — shared by physical button and app button
// ============================================================
void cycleAgeGroup() {
  currentGroup++;
  if (currentGroup > 2) currentGroup = 0;
  setpoint = THI_SETPOINTS[currentGroup];

  // Reset PID to prevent windup on setpoint change
  integral  = 0.0;
  lastError = 0.0;
  pidOutput = 0.0;

  Serial.print("AGE: Group → ");
  Serial.print(AGE_LABELS[currentGroup]);
  Serial.print(" SP: ");
  Serial.println(setpoint);

  // Update Blynk age display
  if (Blynk.connected()) {
    Blynk.virtualWrite(V12, String(AGE_LABELS[currentGroup]) +
                             " SP:" + String(setpoint, 1));
  }

  showAgeConfirmation();
}

// ============================================================
//  SEND DATA TO BLYNK
// ============================================================
void sendToBlynk() {
  if (!Blynk.connected()) return;

  // Sensor readings
  Blynk.virtualWrite(V0, humidity);
  Blynk.virtualWrite(V1, temperature);

  // Actuator speeds — sync sliders with actual values
  int fanPercent     = map(fanPWM, 0, 255, 0, 100);
  int exhaustPercent = map(exhaustPWM, 0, 255, 0, 100);
  Blynk.virtualWrite(V2, fanPercent);
  Blynk.virtualWrite(V3, exhaustPercent);

  // Sync sliders to actual fan speed so they reflect reality
  if (appFanSpeed == -1) Blynk.virtualWrite(V7, fanPercent);
  if (appExhaustSpeed == -1) Blynk.virtualWrite(V8, exhaustPercent);

  // Actuator states
  Blynk.virtualWrite(V4, pumpON ? 1 : 0);
  Blynk.virtualWrite(V5, humidifierON ? 1 : 0);

  // System online
  Blynk.virtualWrite(V9, 1);

  // Age display
  Blynk.virtualWrite(V12, String(AGE_LABELS[currentGroup]) +
                           " SP:" + String(setpoint, 1));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(PUMP_PIN,       OUTPUT);
  pinMode(TRIG_PIN,       OUTPUT);
  pinMode(ECHO_PIN,       INPUT);
  pinMode(AGE_BTN_PIN,    INPUT_PULLUP);
  pinMode(DISPLAY_BTN_PIN, INPUT_PULLUP);

  ledcAttach(FAN_PWM_PIN,     PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(EXHAUST_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(FAN_PWM_PIN,     PWM_MIN);
  ledcWrite(EXHAUST_PWM_PIN, PWM_MIN);
  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(PUMP_PIN,       LOW);

  dht.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Poultry Control");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1500);

  // Connect WiFi and Blynk without blocking
  connectWiFiBlynk();

  setpoint = THI_SETPOINTS[currentGroup];

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  lcd.setCursor(0, 1);
  lcd.print(Blynk.connected() ? "Blynk: Online " : "Blynk: Offline");

  Serial.println("============================================");
  Serial.println("  Poultry Farm Control System — FUT Minna  ");
  Serial.println("============================================");
  Serial.print("Group: "); Serial.println(AGE_LABELS[currentGroup]);
  Serial.print("SP:    "); Serial.println(setpoint);
  Serial.print("Blynk: "); Serial.println(Blynk.connected() ? "Online" : "Offline");
  Serial.println("System ready.");
}

// ============================================================
//  NON-BLOCKING WIFI + BLYNK CONNECTION
//  System does not freeze if WiFi is unavailable
// ============================================================
void connectWiFiBlynk() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    lcd.setCursor(0, 1);
    lcd.print("WiFi OK         ");
    delay(500);

    // Configure Blynk without blocking
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(3000);  // 3 second timeout — does not block if fails

    if (Blynk.connected()) {
      Serial.println("Blynk connected!");
      lcd.setCursor(0, 1);
      lcd.print("Blynk Online    ");
    } else {
      Serial.println("Blynk failed — running offline.");
      lcd.setCursor(0, 1);
      lcd.print("Blynk Offline   ");
    }
  } else {
    Serial.println("\nWiFi failed — running offline.");
    lcd.setCursor(0, 1);
    lcd.print("WiFi Failed     ");
  }
  delay(1500);
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  Blynk.run();  // must be first line always — handles reconnects + incoming writes

  unsigned long currentTime = millis();

  // 1. Buttons
  handleButtons(currentTime);

  // 2. Cooldown ramp processing
  runCooldown(currentTime);

  // 3. Sensor + control every SENSOR_INTERVAL
  if (currentTime - lastSensorTime >= SENSOR_INTERVAL) {
    lastSensorTime = currentTime;
    readSensors();
    computeTHI();
    runConditionControl();
    runPumpControl();
    printSerialDebug();
    if (displayState > 0) displayNeedsUpdate = true;
  }

  // 4. Send to Blynk every BLYNK_INTERVAL
  if (currentTime - lastBlynkSendTime >= BLYNK_INTERVAL) {
    lastBlynkSendTime = currentTime;
    sendToBlynk();
  }

  // 5. Refresh display when needed
  if (displayNeedsUpdate) {
    displayNeedsUpdate = false;
    if (displayState == 1) showDisplayPage1();
    else if (displayState == 2) showDisplayPage2();
  }
}

// ============================================================
//  READ SENSORS
// ============================================================
void readSensors() {
  float newTemp = dht.readTemperature();
  float newHum  = dht.readHumidity();

  if (!isnan(newTemp) && !isnan(newHum)) {
    if (newTemp >= 0.0 && newTemp <= 50.0 &&
        newHum  >= 0.0 && newHum  <= 100.0) {
      temperature = newTemp;
      humidity    = newHum;
    } else {
      Serial.println("WARNING: DHT22 out of range.");
    }
  } else {
    Serial.println("WARNING: DHT22 read failed.");
  }

  waterDistance = readUltrasonic();
}

// ============================================================
//  HC-SR04
// ============================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);

  if (duration == 0) {
    Serial.println("WARNING: HC-SR04 timeout.");
    return 999.0;
  }

  return (duration * 0.0343) / 2.0;
}

// ============================================================
//  COMPUTE THI
// ============================================================
void computeTHI() {
  thi = (0.8 * temperature)
      + ((humidity / 100.0) * (temperature - 14.4))
      + 46.4;
}

// ============================================================
//  COOLDOWN RAMP
//  After manual override is released, fans ramp down
//  gradually over 2 minutes instead of cutting instantly
// ============================================================
void runCooldown(unsigned long currentTime) {

  if (fanCooldownActive && appFanSpeed == -1) {
    unsigned long elapsed = currentTime - fanCooldownStart;
    if (elapsed >= COOLDOWN_MS) {
      // Cooldown complete — PID takes full control
      fanCooldownActive = false;
      Serial.println("Fan cooldown complete — PID active");
    } else {
      // Ramp down linearly from start PWM to 0 over 2 minutes
      int rampPWM = map(elapsed, 0, COOLDOWN_MS,
                        fanPWMAtCooldownStart, 0);
      fanPWM = rampPWM;
      ledcWrite(FAN_PWM_PIN, fanPWM);
    }
  }

  if (exhaustCooldownActive && appExhaustSpeed == -1) {
    unsigned long elapsed = currentTime - exhaustCooldownStart;
    if (elapsed >= COOLDOWN_MS) {
      exhaustCooldownActive = false;
      Serial.println("Exhaust cooldown complete — auto active");
    } else {
      int rampPWM = map(elapsed, 0, COOLDOWN_MS,
                        exhaustPWMAtCooldownStart, 0);
      exhaustPWM = rampPWM;
      ledcWrite(EXHAUST_PWM_PIN, exhaustPWM);
    }
  }
}

// ============================================================
//  CONDITION-BASED CONTROL
// ============================================================
void runConditionControl() {
  bool heatCondition = (thi > setpoint);
  bool humidHigh     = (humidity > HUMIDITY_HIGH);

  if (heatCondition) {
    pidError   = thi - setpoint;
    integral  += pidError;
    integral   = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
    derivative = pidError - lastError;
    lastError  = pidError;
    pidOutput  = (Kp * pidError)
               + (Ki * integral)
               + (Kd * derivative);

    // Only apply PID if no manual override AND no cooldown active
    if (appFanSpeed == -1 && !fanCooldownActive) {
      fanPWM = (int)constrain(pidOutput, PWM_MIN, PWM_MAX);
      ledcWrite(FAN_PWM_PIN, fanPWM);
    }

    if (humidHigh) {
      // HUMID HEAT
      if (appExhaustSpeed == -1 && !exhaustCooldownActive) {
        exhaustPWM = PWM_MAX;
        ledcWrite(EXHAUST_PWM_PIN, exhaustPWM);
      }
      if (!forceHumidifierON && humidifierON) {
        humidifierON = false;
        digitalWrite(HUMIDIFIER_PIN, LOW);
      }
      Serial.println("CONDITION: Humid Heat");

    } else {
      // DRY HEAT
      if (appExhaustSpeed == -1 && !exhaustCooldownActive) {
        exhaustPWM = PWM_MIN;
        ledcWrite(EXHAUST_PWM_PIN, exhaustPWM);
      }
      if (!forceHumidifierON && !humidifierON) {
        humidifierON = true;
        digitalWrite(HUMIDIFIER_PIN, HIGH);
      }
      Serial.println("CONDITION: Dry Heat");
    }

  } else {
    // COOL
    if (appFanSpeed == -1 && !fanCooldownActive) {
      fanPWM = PWM_MIN;
      ledcWrite(FAN_PWM_PIN, fanPWM);
    }

    integral  = 0.0;
    lastError = 0.0;
    pidOutput = 0.0;

    if (!forceHumidifierON && humidifierON) {
      humidifierON = false;
      digitalWrite(HUMIDIFIER_PIN, LOW);
    }

    if (humidHigh) {
      // HUMID COLD
      if (appExhaustSpeed == -1 && !exhaustCooldownActive) {
        exhaustPWM = 64;
        ledcWrite(EXHAUST_PWM_PIN, exhaustPWM);
      }
      Serial.println("CONDITION: Humid Cold");
    } else {
      // OPTIMAL
      if (appExhaustSpeed == -1 && !exhaustCooldownActive) {
        exhaustPWM = PWM_MIN;
        ledcWrite(EXHAUST_PWM_PIN, exhaustPWM);
      }
      Serial.println("CONDITION: Optimal");
    }
  }
}

// ============================================================
//  WATER PUMP CONTROL
// ============================================================
void runPumpControl() {
  if (forcePumpON) return;  // app override active

  if (waterDistance > TANK_EMPTY_CM && !pumpON) {
    // Tank empty — turn pump ON
    pumpON = true;
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("ACTION: Pump ON — tank empty");

  } else if (waterDistance <= TANK_FULL_CM && pumpON) {
    // Tank full — turn pump OFF
    pumpON = false;
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("ACTION: Pump OFF — tank full");
  }
  // Between TANK_FULL_CM and TANK_EMPTY_CM: no change (hysteresis band)
}

// ============================================================
//  HANDLE BUTTONS
// ============================================================
void handleButtons(unsigned long currentTime) {

  // --- Age Set Button (GPIO32) ---
  if (digitalRead(AGE_BTN_PIN) == LOW) {
    if (currentTime - lastAgeBtnTime >= DEBOUNCE_DELAY) {
      lastAgeBtnTime = currentTime;
      cycleAgeGroup();
    }
  }

  // --- Display Button (GPIO33) ---
  if (digitalRead(DISPLAY_BTN_PIN) == LOW) {
    if (currentTime - lastDisplayBtnTime >= DEBOUNCE_DELAY) {
      lastDisplayBtnTime = currentTime;

      displayState++;
      if (displayState > 2) displayState = 0;
      displayNeedsUpdate = true;

      if (displayState == 1) {
        showDisplayPage1();
        Serial.println("DISPLAY: Page 1");
      } else if (displayState == 2) {
        showDisplayPage2();
        Serial.println("DISPLAY: Page 2");
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("System Running");
        Serial.println("DISPLAY: Off");
      }
    }
  }
}

// ============================================================
//  DISPLAY PAGE 1 — Environmental readings
// ============================================================
void showDisplayPage1() {
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("THI:");
  lcd.print(thi, 1);
  lcd.print(" SP:");
  lcd.print(setpoint, 1);
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C H:");
  lcd.print((int)humidity);
  lcd.print("%");
}

// ============================================================
//  DISPLAY PAGE 2 — Actuator status
// ============================================================
void showDisplayPage2() {
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(AGE_LABELS[currentGroup]);
  lcd.print(" Fan:");
  lcd.print(map(fanPWM, 0, 255, 0, 100));
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Hum:");
  lcd.print(humidifierON ? "ON " : "OFF");
  lcd.print(" Pmp:");
  lcd.print(pumpON ? "ON " : "OFF");
}

// ============================================================
//  AGE CONFIRMATION DISPLAY
//  Shows for 2 seconds then returns to current display state
// ============================================================
void showAgeConfirmation() {
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Age: ");
  lcd.print(AGE_LABELS[currentGroup]);
  lcd.setCursor(0, 1);
  lcd.print("Setpoint: ");
  lcd.print(setpoint, 1);
  delay(2000);

  // Return to active display state
  if (displayState == 1) showDisplayPage1();
  else if (displayState == 2) showDisplayPage2();
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Running");
  }
}

// ============================================================
//  SERIAL DEBUG
// ============================================================
void printSerialDebug() {
  Serial.println("============================================");
  Serial.print("Group: ");      Serial.print(AGE_LABELS[currentGroup]);
  Serial.print("  SP: ");       Serial.println(setpoint);
  Serial.print("Temp: ");       Serial.print(temperature, 2);
  Serial.print("C  Hum: ");     Serial.print(humidity, 1);
  Serial.println("%");
  Serial.print("THI: ");        Serial.println(thi, 2);
  Serial.print("PID Err: ");    Serial.print(pidError, 3);
  Serial.print("  Out: ");      Serial.println(pidOutput, 2);
  Serial.print("Fan: ");        Serial.print(map(fanPWM, 0, 255, 0, 100));
  Serial.print("%  Exh: ");     Serial.print(map(exhaustPWM, 0, 255, 0, 100));
  Serial.println("%");
  Serial.print("Humidifier: "); Serial.println(humidifierON ? "ON" : "OFF");
  Serial.print("Water: ");      Serial.print(waterDistance, 1);
  Serial.print("cm  Pump: ");   Serial.println(pumpON ? "ON" : "OFF");
  Serial.print("Fan cooldown: "); Serial.println(fanCooldownActive ? "Active" : "Off");
  Serial.print("Blynk: ");      Serial.println(Blynk.connected() ? "Online" : "Offline");
  Serial.println("============================================");
}
