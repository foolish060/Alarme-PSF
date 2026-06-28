/********* AlarmCam — robuste anti-faux déclenchements (avec PIN + 2 tensions + NVS) *********
 * ESP32 + ICM-42670-P + BluetoothSerial (SPP)
 * - 1 info par ligne
 * - Auth par PIN: PASS? / PASS=<pin> (auth) puis PASS=<nouveau> (changement)
 * - Reset PIN matériel: maintenir BTN_RESET enfoncé au power-on => PIN=0000
 * - Détection mouvement identique (SENS_MG, HITS, GAP, GYRO)
 * - Tensions:
 *     VOLT / VOLT_MV   -> Alimentation (graph)
 *     VBAT / VBAT_MV   -> Batterie (icône)
 * - Nom BT: NAME? / NAME=<...> (appliqué à chaud + sauvegardé)
 * - Persistance: NVS (Preferences)
 ***************************************************************************************/

#include <Wire.h>
#include "BluetoothSerial.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <Preferences.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth classique désactivé dans cette build. Activez-le (ESP32 core)."
#endif

/***************** PINS *****************/
// I2C IMU
#define SDA_PIN 21
#define SCL_PIN 22

// ADC Alimentation 12V (via diviseur ~5.68x -> ex. 220k+47k)
#define ADC_SUPPLY_PIN 32
// ADC Batterie 3..4.1V (via diviseur ~2x -> ex. 100k+100k)
#define ADC_BAT_PIN    33

// Bouton reset PIN (tiré à VCC via pullup interne, act. à GND)
#define PIN_RESET_BTN  25

/***************** IMU ICM-42670-P *****************/
#define IMU_ADDR 0x68
#define REG_WHO_AM_I            0x75
#define REG_SIGNAL_PATH_RESET   0x02
#define REG_PWR_MGMT0           0x1F
#define REG_GYRO_CONFIG0        0x20
#define REG_ACCEL_CONFIG0       0x21
#define REG_TEMP_DATA1          0x09 // 14 octets : T, Ax,Ay,Az, Gx,Gy,Gz

static inline bool i2cWrite(uint8_t r, uint8_t v){
  Wire.beginTransmission(IMU_ADDR); Wire.write(r); Wire.write(v);
  return Wire.endTransmission(true)==0;
}
static inline bool i2cReadN(uint8_t r, uint8_t*buf, uint8_t n){
  Wire.beginTransmission(IMU_ADDR); Wire.write(r);
  if(Wire.endTransmission(true)!=0) return false;
  delayMicroseconds(50);
  int got=Wire.requestFrom((int)IMU_ADDR,(int)n,(int)true);
  if(got!=n) return false;
  for(int i=0;i<n;i++) buf[i]=Wire.read();
  return true;
}

/***************** Détection mouvement *****************/
float emaAccel = 0.0f;
bool  emaInit  = false;

// ICM42670 @ ±4g => 8192 LSB/g
const float ACC_LSB_PER_G = 8192.0f;
const float LSB_PER_MG    = ACC_LSB_PER_G / 1000.0f; // 8.192

// Limites (éventuel mode dev plus tard)
uint32_t SENS_MIN_MG = 50;
uint32_t SENS_MAX_MG = 100000;

// Réglages (défauts costauds = peu sensible)
uint32_t SENS_MG = 5000;
uint32_t THRESH_ACCEL = (uint32_t)(5000 * LSB_PER_MG);
uint8_t  REQ_HITS = 5;
uint8_t  hitCount = 0;
uint32_t GAP_MS   = 5000;
unsigned long lastAlarmMs = 0;

// Gyro optionnel
bool USE_GYRO = false;
const uint32_t THRESH_GYRO = 12000;

/***************** Tensions (ADC) *****************/
// Gains de diviseur (adapter à ton montage)
const float SUPPLY_DIV_GAIN = (220000.0f + 47000.0f) / 47000.0f; // ≈ 5.68
const float VBAT_DIV_GAIN   = (100000.0f + 100000.0f) / 100000.0f; // ≈ 2.00

esp_adc_cal_characteristics_t adc_chars;
float ADC_VREF = 1100.0f; // mV (calibration ESP32)

static inline uint32_t readMilliVoltsADC(int pin) {
  uint32_t raw = analogRead(pin);
  return esp_adc_cal_raw_to_voltage(raw, &adc_chars); // mV au pin
}

uint32_t readSupplyMilliVolts() {
  uint32_t mv_pin = readMilliVoltsADC(ADC_SUPPLY_PIN);
  float vin = (float)mv_pin * SUPPLY_DIV_GAIN; // mV estimés alim
  if (vin < 0) vin = 0;
  return (uint32_t)vin;
}

uint32_t readBatteryMilliVolts() {
  uint32_t mv_pin = readMilliVoltsADC(ADC_BAT_PIN);
  float vin = (float)mv_pin * VBAT_DIV_GAIN; // mV estimés batterie
  if (vin < 0) vin = 0;
  return (uint32_t)vin;
}

/***************** Bluetooth SPP *****************/
BluetoothSerial SerialBT;
String btName = "Alarme Cameras";

volatile uint32_t motionCount = 0;
bool detectionEnabled = true;

// Période d’envoi périodique des tensions (minutes)
uint32_t pvMinutes = 30;
unsigned long lastPVms = 0;

/***************** Auth PIN + NVS *****************/
Preferences prefs;
String pinCode = "0000";
bool authOK = false;

void saveSettings() {
  prefs.putString("pin", pinCode);
  prefs.putString("name", btName);
  prefs.putUInt("sens", SENS_MG);
  prefs.putUChar("hits", REQ_HITS);
  prefs.putUInt("gap", GAP_MS);
  prefs.putUChar("gyro", USE_GYRO ? 1 : 0);
  prefs.putUInt("pvmin", pvMinutes);
}

void loadSettings() {
  pinCode = prefs.getString("pin", "0000");
  btName  = prefs.getString("name", "Alarme Cameras");
  SENS_MG = prefs.getUInt("sens", SENS_MG);
  REQ_HITS= prefs.getUChar("hits", REQ_HITS);
  GAP_MS  = prefs.getUInt("gap", GAP_MS);
  USE_GYRO= prefs.getUChar("gyro", USE_GYRO?1:0) ? true : false;
  pvMinutes = prefs.getUInt("pvmin", pvMinutes);
  // Recalcule le seuil accel
  if (SENS_MG < SENS_MIN_MG) SENS_MG = SENS_MIN_MG;
  if (SENS_MG > SENS_MAX_MG) SENS_MG = SENS_MAX_MG;
  THRESH_ACCEL = (uint32_t)(SENS_MG * LSB_PER_MG);
}

/***************** Utilitaires I/O *****************/
char rxBuf[96];
uint8_t rxLen = 0;

void printlnBT(const char* s) { SerialBT.println(s); }

void printAlarmEvent() {
  printlnBT("EVENT=ALARM");
  printlnBT("STATE=ON");
  char s[32];
  snprintf(s, sizeof(s), "COUNT=%lu", (unsigned long)motionCount);
  printlnBT(s);
}

void printVoltagesOnce() {
  // Alim (graph)
  uint32_t mvSupply = readSupplyMilliVolts();
  float vSupply = mvSupply / 1000.0f;
  char s1[32], s2[32];
  snprintf(s1, sizeof(s1), "VOLT=%.2f", vSupply);
  snprintf(s2, sizeof(s2), "VOLT_MV=%lu", (unsigned long)mvSupply);
  printlnBT(s1); printlnBT(s2);

  // Batterie (icône)
  uint32_t mvBat = readBatteryMilliVolts();
  float vBat = mvBat / 1000.0f;
  char s3[32], s4[32];
  snprintf(s3, sizeof(s3), "VBAT=%.2f", vBat);
  snprintf(s4, sizeof(s4), "VBAT_MV=%lu", (unsigned long)mvBat);
  printlnBT(s3); printlnBT(s4);
}

void printStatus() {
  printlnBT("STATUS=OK");
  printlnBT(detectionEnabled ? "STATE=ON" : "STATE=OFF");
  char s[48];
  snprintf(s, sizeof(s), "NAME=%s", btName.c_str()); printlnBT(s);
  snprintf(s, sizeof(s), "SENS_MG=%lu", (unsigned long)SENS_MG); printlnBT(s);
  snprintf(s, sizeof(s), "HITS=%u", REQ_HITS); printlnBT(s);
  snprintf(s, sizeof(s), "GAP=%lu", (unsigned long)GAP_MS); printlnBT(s);
  printlnBT(USE_GYRO ? "GYRO=ON" : "GYRO=OFF");
  snprintf(s, sizeof(s), "PVMIN=%lu", (unsigned long)pvMinutes); printlnBT(s);
  snprintf(s, sizeof(s), "COUNT=%lu", (unsigned long)motionCount); printlnBT(s);
}

void applySensitivityMg(uint32_t mg){
  if (mg < SENS_MIN_MG) mg = SENS_MIN_MG;
  if (mg > SENS_MAX_MG) mg = SENS_MAX_MG;
  SENS_MG = mg;
  THRESH_ACCEL = (uint32_t)(SENS_MG * LSB_PER_MG);
}

/***************** Commandes *****************/
bool isAuthCommand(const char* line) {
  return (strncmp(line, "PASS=", 5)==0) || (strncmp(line, "AUTH=", 5)==0) || (strcmp(line,"PASS?")==0);
}

bool checkAndMaybeAuthenticate(const char* line) {
  if (strncmp(line, "PASS=", 5)==0 || strncmp(line, "AUTH=", 5)==0) {
    const char* p = (line[1]=='A') ? (line+5) : (line+5); // AUTH= or PASS=
    // Si déjà authentifié => PASS=<...> devient "changement de PIN"
    if (authOK && strncmp(line, "PASS=", 5)==0) {
      String np = String(p); np.trim();
      if (np.length()==0 || np.length()>8) { printlnBT("ERR"); return true; }
      // simple règle: uniquement chiffres (optionnel)
      for (size_t i=0;i<np.length();++i) { if (np[i]<'0' || np[i]>'9') { printlnBT("ERR"); return true; } }
      pinCode = np;
      saveSettings();
      printlnBT("OK");
      printlnBT("PASS_CHANGED");
      return true;
    }
    // Sinon: tentative d'auth
    String in = String(p); in.trim();
    if (in == pinCode) {
      authOK = true;
      printlnBT("OK");
    } else {
      printlnBT("ERR");
    }
    return true;
  }
  if (strcmp(line, "PASS?")==0) {
    char s[24];
    snprintf(s, sizeof(s), "PASS_LEN=%u", (unsigned)pinCode.length());
    printlnBT(s);
    printlnBT("PASS=****");
    return true;
  }
  return false;
}

void handleCommand(const char* line) {
  // 1) Auth en premier
  if (checkAndMaybeAuthenticate(line)) return;

  // 2) Bloquer si pas authentifié
  if (!authOK) { printlnBT("ERR"); return; }

  // 3) Commandes standards (identiques + extensions)
  if (strcmp(line, "ON")==0)  { detectionEnabled = true;  printlnBT("OK"); printlnBT("STATE=ON");  return; }
  if (strcmp(line, "OFF")==0) { detectionEnabled = false; printlnBT("OK"); printlnBT("STATE=OFF"); return; }
  if (strcmp(line, "RESET")==0) { motionCount = 0; printlnBT("OK"); printlnBT("COUNT=0"); return; }
  if (strcmp(line, "STATUS?")==0) { printStatus(); return; }

  if (strncmp(line, "SENS=", 5)==0) {
    uint32_t mg = strtoul(line+5, nullptr, 10);
    applySensitivityMg(mg);
    saveSettings();
    printlnBT("OK");
    char s[32]; snprintf(s, sizeof(s), "SENS_MG=%lu", (unsigned long)SENS_MG); printlnBT(s);
    return;
  }
  if (strcmp(line, "SENS?")==0) {
    char s[32]; snprintf(s, sizeof(s), "SENS_MG=%lu", (unsigned long)SENS_MG); printlnBT(s);
    return;
  }

  if (strncmp(line, "HITS=", 5)==0) {
    uint32_t n = strtoul(line+5, nullptr, 10);
    if (n < 1) n = 1; if (n > 20) n = 20;
    REQ_HITS = (uint8_t)n; hitCount = 0;
    saveSettings();
    printlnBT("OK");
    char s[16]; snprintf(s, sizeof(s), "HITS=%u", REQ_HITS); printlnBT(s);
    return;
  }
  if (strcmp(line, "HITS?")==0) {
    char s[16]; snprintf(s, sizeof(s), "HITS=%u", REQ_HITS); printlnBT(s);
    return;
  }

  if (strncmp(line, "GAP=", 4)==0) {
    uint32_t ms = strtoul(line+4, nullptr, 10);
    if (ms < 500) ms = 500; if (ms > 600000) ms = 600000;
    GAP_MS = ms;
    saveSettings();
    printlnBT("OK");
    char s[24]; snprintf(s, sizeof(s), "GAP=%lu", (unsigned long)GAP_MS); printlnBT(s);
    return;
  }
  if (strcmp(line, "GAP?")==0) {
    char s[24]; snprintf(s, sizeof(s), "GAP=%lu", (unsigned long)GAP_MS); printlnBT(s);
    return;
  }

  if (strncmp(line, "GYRO=", 5)==0) {
    if (strcmp(line+5,"ON")==0)  { USE_GYRO = true;  saveSettings(); printlnBT("OK"); printlnBT("GYRO=ON");  return; }
    if (strcmp(line+5,"OFF")==0) { USE_GYRO = false; saveSettings(); printlnBT("OK"); printlnBT("GYRO=OFF"); return; }
  }
  if (strcmp(line, "GYRO?")==0) { printlnBT(USE_GYRO ? "GYRO=ON" : "GYRO=OFF"); return; }

  if (strncmp(line, "PVMIN=", 6)==0) {
    uint32_t m = strtoul(line+6, nullptr, 10);
    if (m < 1) m = 1; if (m > 360) m = 360;
    pvMinutes = m;
    saveSettings();
    printlnBT("OK");
    char s[24]; snprintf(s, sizeof(s), "PVMIN=%lu", (unsigned long)pvMinutes); printlnBT(s);
    return;
  }
  if (strcmp(line, "PVMIN?")==0) {
    char s[24]; snprintf(s, sizeof(s), "PVMIN=%lu", (unsigned long)pvMinutes); printlnBT(s);
    return;
  }

  if (strncmp(line, "NAME=", 5)==0) {
    String newName = String(line+5); newName.trim();
    if (newName.length() >= 3 && newName.length() < 24) {
      // Sauvegarde + rename à chaud
      btName = newName;
      saveSettings();
      printlnBT("OK");
      printlnBT("REBOOTING_NAME");
      delay(200);
      SerialBT.end(); delay(200);
      SerialBT.begin(btName);
      printlnBT("READY");
      printlnBT("STATE=ON");
      return;
    } else {
      printlnBT("ERR");
      return;
    }
  }
  if (strcmp(line, "NAME?")==0) {
    char s[48];
    snprintf(s, sizeof(s), "NAME=%s", btName.c_str()); printlnBT(s);
    return;
  }

  // Tensions
  if (strcmp(line, "VOLT?")==0) { // alimentation
    uint32_t mv = readSupplyMilliVolts();
    float v = mv / 1000.0f;
    char s1[32], s2[32];
    snprintf(s1, sizeof(s1), "VOLT=%.2f", v);
    snprintf(s2, sizeof(s2), "VOLT_MV=%lu", (unsigned long)mv);
    printlnBT(s1); printlnBT(s2);
    return;
  }
  if (strcmp(line, "BAT?")==0) { // batterie
    uint32_t mv = readBatteryMilliVolts();
    float v = mv / 1000.0f;
    char s1[32], s2[32];
    snprintf(s1, sizeof(s1), "VBAT=%.2f", v);
    snprintf(s2, sizeof(s2), "VBAT_MV=%lu", (unsigned long)mv);
    printlnBT(s1); printlnBT(s2);
    return;
  }

  printlnBT("ERR");
}

void pollBT() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c=='\r') continue;
    if (c=='\n') {
      rxBuf[rxLen] = 0;
      if (rxLen > 0) handleCommand(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf)-1) {
      rxBuf[rxLen++] = c;
    }
  }
}

/***************** SETUP *****************/
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== AlarmCam — robuste + PIN + 2 tensions ===");

  // Reset PIN matériel (bouton tiré à VCC)
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);

  // NVS
  prefs.begin("alarmcam", false); // RW
  loadSettings();

  // Check reset PIN au boot (appuyé => LOW)
  if (digitalRead(PIN_RESET_BTN) == LOW) {
    // petite temporisation pour éviter un faux contact
    delay(200);
    if (digitalRead(PIN_RESET_BTN) == LOW) {
      pinCode = "0000";
      saveSettings();
      Serial.println(">> PIN reset via bouton: 0000");
    }
  }

  // I2C IMU
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  Wire.setTimeOut(50);
  delay(10);

  // IMU init
  i2cWrite(REG_SIGNAL_PATH_RESET, 0x10); delay(80);
  i2cWrite(REG_PWR_MGMT0, 0x0F); delay(10);
  i2cWrite(REG_GYRO_CONFIG0,  0x08);
  i2cWrite(REG_ACCEL_CONFIG0, 0x48);
  delay(20);

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_SUPPLY_PIN, ADC_11db);
  analogSetPinAttenuation(ADC_BAT_PIN,    ADC_11db);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, (uint32_t)ADC_VREF, &adc_chars);

  // Bluetooth (nom depuis NVS)
  SerialBT.begin(btName);
  printlnBT("READY");
  printlnBT("STATE=ON");
  printlnBT("NEED_AUTH"); // indicatif: l'app doit envoyer PASS=<pin>
  printStatus();          // infos (SENS/HITS/GAP/...)

  // Auth à faire par l'app (PASS=<pin>)
  authOK = false;
}

/***************** LOOP *****************/
void loop() {
  // Lecture IMU
  uint8_t raw[14];
  if (i2cReadN(REG_TEMP_DATA1, raw, sizeof(raw))) {
    int16_t ax = (int16_t)((raw[2]<<8)|raw[3]);
    int16_t ay = (int16_t)((raw[4]<<8)|raw[5]);
    int16_t az = (int16_t)((raw[6]<<8)|raw[7]);
    int16_t gx = (int16_t)((raw[8]<<8)|raw[9]);
    int16_t gy = (int16_t)((raw[10]<<8)|raw[11]);
    int16_t gz = (int16_t)((raw[12]<<8)|raw[13]);

    uint32_t aMag = (uint32_t)(abs(ax)+abs(ay)+abs(az));
    uint32_t gMag = (uint32_t)(abs(gx)+abs(gy)+abs(gz));

    if(!emaInit){ emaAccel = (float)aMag; emaInit = true; }
    else { emaAccel = 0.995f*emaAccel + 0.005f*(float)aMag; }

    uint32_t devA_raw = (uint32_t)fabs((float)aMag - emaAccel);
    bool overAccel = (devA_raw > THRESH_ACCEL);
    bool overGyro  = USE_GYRO ? (gMag > THRESH_GYRO) : false;
    bool over = overAccel || overGyro;

    unsigned long now = millis();

    if (detectionEnabled) {
      if (over) {
        if (hitCount < 250) hitCount++;
      } else {
        if (hitCount > 0) hitCount--;
      }

      if (hitCount >= REQ_HITS && (now - lastAlarmMs >= GAP_MS)) {
        lastAlarmMs = now;
        hitCount = 0;
        motionCount++;
        printAlarmEvent();
      }
    }
  }

  // Envoi périodique (alim + batterie)
  unsigned long nowMs = millis();
  if (nowMs - lastPVms >= pvMinutes * 60000UL) {
    lastPVms = nowMs;
    printVoltagesOnce();
  }

  pollBT();
  delay(5);
}
