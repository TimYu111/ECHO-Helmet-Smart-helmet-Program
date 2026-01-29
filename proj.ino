#include <Wire.h>
#include <LiquidCrystal_I2C.h>

unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_MS = 1000;  // lcd refresh rate

const unsigned long BRAKE_BLINK_MS = 200;
unsigned long lastBlinkBrakeMs = 0;
bool brakeBlinkOn = true;
const uint8_t I2C_ADDR = 0x27;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

const int TRIG_PIN = 9;                // ultrasonic sensor
const int ECHO_PIN = 10;

const int MOTOR_L_PIN = 5;
const int MOTOR_R_PIN = 6;

const int BTN_LEFT  = 2;               // left
const int BTN_RIGHT = 3;               // right
const int BTN_BRAKE = 4;               // brake

// thresholds
const float DIST_WARN_CM   = 10.0;     // warn distance
const float SPEED_WARN_CMS = 100.0;    // approaching speed

// timing constants
const unsigned long SAMPLE_PERIOD_MS = 100;
const unsigned long BLINK_FAST_MS    = 200;
const unsigned long BLINK_SLOW_MS    = 500;
const unsigned long VIB_PULSE_MS     = 150;
const unsigned long VIB_GAP_MS       = 250;

LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLS, LCD_ROWS);

uint8_t ARROW_LEFT[8]  = { B00100, B01100, B11111, B01100, B00100, B00000, B00000, B00000 };
uint8_t ARROW_RIGHT[8] = { B00100, B00110, B11111, B00110, B00100, B00000, B00000, B00000 };

enum TurnState { TURN_NONE, TURN_LEFT, TURN_RIGHT, TURN_BRAKE };
TurnState turnState = TURN_NONE;

bool backlightOn = true;
bool warnActive  = false;
bool dangerActive = false;

const unsigned long DEBOUNCE_MS = 30;
bool btnL_last = HIGH, btnR_last = HIGH, btnB_last = HIGH;
unsigned long btnL_lastChange = 0, btnR_lastChange = 0, btnB_lastChange = 0;

// distance and speed variables
float dist_cm = 999.0;
float dist_cm_filtered = 999.0;
float last_dist_cm_filtered = 999.0;
unsigned long lastSampleMs = 0;
unsigned long lastSpeedTs  = 0;
float approach_speed_cms = 0.0;

unsigned long lastBlinkBacklightMs = 0;
unsigned long lastBlinkArrowMs     = 0;
unsigned long lastVibToggleMs      = 0;   
bool vibOn = false;                        


const uint8_t MPU_ADDR = 0x68;
const int BUZZER_PIN = 7; 


const float IMPACT_G_THRESHOLD = 2;  // adjustable g value DEPENDING ON WHAT YOU NEED~~
const unsigned long IMPACT_DEBOUNCE_MS = 300;
const unsigned long IMPACT_ALERT_MS    = 2000;
const unsigned long BUZZ_BEEP_MS       = 200;
const unsigned long BUZZ_GAP_MS        = 200;
const unsigned int  BUZZ_FREQ_HZ       = 5000; // buzzer frequency

bool impactActive = false;
unsigned long impactStartMs = 0;
unsigned long lastImpactEvalMs = 0;
unsigned long lastBuzzToggleMs = 0;
bool buzzOn = false;


const float ACC_SENS_LSB_PER_G = 8192.0f;


void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  uint8_t n = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len, (uint8_t)true);
  for (uint8_t i = 0; i < len && Wire.available(); ++i) {
    buf[i] = Wire.read();
  }
}

bool initMPU6050() {
  Wire.begin();
  delay(50);

  mpuWriteReg(0x6B, 0x00);
  delay(10);
  mpuWriteReg(0x1C, 0x10);
  delay(10);
  uint8_t id=0; mpuReadBytes(0x75, &id, 1);
  return (id == 0x68);
}

// reading g value
void readAccelG(float &ax_g, float &ay_g, float &az_g) {
  uint8_t raw[6];
  mpuReadBytes(0x3B, raw, 6); // ACCELEROMETER
  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  ax_g = ax / ACC_SENS_LSB_PER_G;
  ay_g = ay / ACC_SENS_LSB_PER_G;
  az_g = az / ACC_SENS_LSB_PER_G;
}
void showImpact(float g_excess) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("IMPACT! ALERT");
  lcd.setCursor(0,1); lcd.print("|a|-1g:");
  lcd.print(g_excess, 1);
  lcd.print("g");
}

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 10000UL);
  if (duration == 0) return NAN;
  return (duration * 0.0343f) / 2.0f;
}

void setVibration(bool on, uint8_t duty = 200) {
  vibOn = on;
  analogWrite(MOTOR_L_PIN, on ? duty : 0);
  analogWrite(MOTOR_R_PIN, on ? duty : 0);
}

void lcdBacklight(bool on) {
  backlightOn = on;
  if (on) lcd.backlight();
  else    lcd.noBacklight();
}

bool debounceRead(int pin, bool &lastState, unsigned long &lastChange) {
  bool reading = digitalRead(pin);
  unsigned long now = millis();
  if (reading != lastState && (now - lastChange) > DEBOUNCE_MS) {
    lastState = reading;
    lastChange = now;
    return true;
  }
  return false;
}

void showIdleSafe() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("READY / SAFE");
  lcd.setCursor(0,1);
  lcd.print("Dist:");
  if (isnan(dist_cm)) lcd.print("--");
  else lcd.print((int)dist_cm);
  lcd.print("cm  v:");
  lcd.print((int)approach_speed_cms);
}

void showWarning() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("VEHICLE NEAR!");
  lcd.setCursor(0,1);
  lcd.print("d<"); lcd.print((int)DIST_WARN_CM);
  lcd.print("cm  v:"); lcd.print((int)approach_speed_cms);
}

void showTurnLeft(bool arrowOn) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("LEFT  ");
  if (arrowOn) { lcd.setCursor(7,0); lcd.write(byte(0)); }
  lcd.setCursor(0,1); lcd.print("<---- TURN");
}

void showTurnRight(bool arrowOn) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("RIGHT ");
  if (arrowOn) { lcd.setCursor(7,0); lcd.write(byte(1)); }
  lcd.setCursor(0,1); lcd.print("TURN ---->");
}

void showBrake() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("BRAKE");
  lcd.setCursor(0,1); lcd.print("STOP !");
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(MOTOR_L_PIN, OUTPUT);
  pinMode(MOTOR_R_PIN, OUTPUT);

  pinMode(BTN_LEFT,  INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_BRAKE, INPUT_PULLUP);

  // impact and buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
  bool imu_ok = initMPU6050();

  lcd.init();
  lcdBacklight(true);
  lcd.createChar(0, ARROW_LEFT);
  lcd.createChar(1, ARROW_RIGHT);

  Serial.begin(115200);
  Serial.println("Smart Helmet Booting...");
  if (imu_ok) Serial.println("MPU6050 init OK");
  else        Serial.println("MPU6050 init FAIL (check wiring/address)");

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Smart Helmet");
  lcd.setCursor(0,1); lcd.print("Init...");
  delay(800);
}

void loop() {
  unsigned long now = millis();

  static unsigned long lastLCDUpdate = 0;
  const unsigned long LCD_UPDATE_MS = 200;
  bool lcdUpdateDue = (now - lastLCDUpdate >= LCD_UPDATE_MS);
  if (lcdUpdateDue) {
    lastLCDUpdate = now;
  }

  if (debounceRead(BTN_LEFT,  btnL_last, btnL_lastChange)) {
    if (btnL_last == LOW) turnState = TURN_LEFT;
    else if (turnState == TURN_LEFT) turnState = TURN_NONE;
  }
  if (debounceRead(BTN_RIGHT, btnR_last, btnR_lastChange)) {
    if (btnR_last == LOW) turnState = TURN_RIGHT;
    else if (turnState == TURN_RIGHT) turnState = TURN_NONE;
  }
  if (debounceRead(BTN_BRAKE, btnB_last, btnB_lastChange)) {
    if (btnB_last == LOW) turnState = TURN_BRAKE;
    else if (turnState == TURN_BRAKE) turnState = TURN_NONE;
  }

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;

    float d = readDistanceCM();
    if (!isnan(d) && d < 600.0) {
      const float alpha = 0.3f;
      if (isnan(dist_cm_filtered) || dist_cm_filtered > 900.0) {
        dist_cm_filtered = d;
        last_dist_cm_filtered = d;
        lastSpeedTs = now;
      } else {
        last_dist_cm_filtered = dist_cm_filtered;
        dist_cm_filtered = alpha * d + (1.0f - alpha) * dist_cm_filtered;

        unsigned long dt = now - lastSpeedTs;
        if (dt > 0) {
          float delta = last_dist_cm_filtered - dist_cm_filtered;
          approach_speed_cms = (delta * 1000.0f) / (float)dt;
        }
        lastSpeedTs = now;
      }
      dist_cm = d;
    }

    bool distWarn  = (!isnan(dist_cm_filtered) && dist_cm_filtered <= DIST_WARN_CM);
    bool speedWarn = (approach_speed_cms >= SPEED_WARN_CMS);
    warnActive  = (distWarn || speedWarn);
    dangerActive = warnActive;
  }

  // buzzer and impact

  if (now - lastImpactEvalMs >= SAMPLE_PERIOD_MS) {
    lastImpactEvalMs = now;
    float ax, ay, az;
    readAccelG(ax, ay, az);
    float a_mag = sqrt(ax*ax + ay*ay + az*az);
    float g_excess = (a_mag > 1.0f) ? (a_mag - 1.0f) : 0.0f; //g value

    static unsigned long lastImpactTime = 0;
    if (g_excess >= IMPACT_G_THRESHOLD &&
        (now - lastImpactTime) > IMPACT_DEBOUNCE_MS) {
      impactActive = true;
      impactStartMs = now;
      lastImpactTime = now;
      buzzOn = true;
      tone(BUZZER_PIN, BUZZ_FREQ_HZ);
      lastBuzzToggleMs = now;
      if (!backlightOn) lcdBacklight(true);
      Serial.print("IMPACT DETECTED: g_excess=");
      Serial.println(g_excess, 2);
    }

    // timeout
    if (impactActive && (now - impactStartMs >= IMPACT_ALERT_MS)) {
      impactActive = false;
      buzzOn = false;
      noTone(BUZZER_PIN);
      setVibration(false);
    }


    static float last_g_excess = 0.0f;
    last_g_excess = g_excess;
  }


  if (impactActive) {
    if (now - lastBlinkBacklightMs >= BLINK_FAST_MS) {
      lastBlinkBacklightMs = now;
      lcdBacklight(!backlightOn);
    }
    unsigned long interval = buzzOn ? BUZZ_BEEP_MS : BUZZ_GAP_MS;
    if (now - lastBuzzToggleMs >= interval) {
      lastBuzzToggleMs = now;
      buzzOn = !buzzOn;
      if (buzzOn) tone(BUZZER_PIN, BUZZ_FREQ_HZ);
      else        noTone(BUZZER_PIN);
    }

    if (lcdUpdateDue) {

      float ax, ay, az; readAccelG(ax, ay, az);
      float g_excess = max(0.0f, sqrt(ax*ax + ay*ay + az*az) - 1.0f);
      showImpact(g_excess);
    }
  }
  else if (turnState == TURN_BRAKE) {
    if (now - lastBlinkBrakeMs >= BRAKE_BLINK_MS) {
      lastBlinkBrakeMs = now;
      brakeBlinkOn = !brakeBlinkOn;
      lcdBacklight(brakeBlinkOn);
    }
    if (lcdUpdateDue) showBrake(); 
    setVibration(false);
    noTone(BUZZER_PIN);
    buzzOn = false;

  } else {
    if (warnActive) {
      if (now - lastBlinkBacklightMs >= BLINK_FAST_MS) {
        lastBlinkBacklightMs = now;
        lcdBacklight(!backlightOn);
      }
      if (lcdUpdateDue) showWarning();

      if (now - lastVibToggleMs >= (vibOn ? VIB_PULSE_MS : VIB_GAP_MS)) {
        lastVibToggleMs = now;
        setVibration(!vibOn);
      }

      // make sure buzzeron=false
      noTone(BUZZER_PIN);
      buzzOn = false;

    } else {
      if (!backlightOn) lcdBacklight(true);
      setVibration(false);

      static bool arrowBlinkOn = true;
      if (now - lastBlinkArrowMs >= BLINK_SLOW_MS) {
        lastBlinkArrowMs = now;
        arrowBlinkOn = !arrowBlinkOn;
      }

      if (lcdUpdateDue) {
        if (turnState == TURN_LEFT)      showTurnLeft(arrowBlinkOn);
        else if (turnState == TURN_RIGHT) showTurnRight(arrowBlinkOn);
        else                              showIdleSafe();
      }

      noTone(BUZZER_PIN);
      buzzOn = false;
    }
  }


  static unsigned long lastLog = 0;
  if (now - lastLog >= 250) {
    lastLog = now;
    Serial.print("dist_raw(cm)="); Serial.print(dist_cm);
    Serial.print(", dist_f(cm)=");  Serial.print(dist_cm_filtered);
    Serial.print(", closing(cm/s)="); Serial.print(approach_speed_cms);
    Serial.print(", warn=");        Serial.print(warnActive);
    Serial.print(", turnState=");   Serial.print((int)turnState);
    float ax, ay, az; readAccelG(ax, ay, az);
    float a_mag = sqrt(ax*ax + ay*ay + az*az);
    Serial.print(", |a|(g)="); Serial.print(a_mag, 2);
    Serial.print(", impact="); Serial.println(impactActive);
  }
}

