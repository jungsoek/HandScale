// ===== ESP32-C3/S3 BLE (Bluedroid) + OLED + HX711 + EEPROM 3-Point Calibration =====
#include <NimBLEDevice.h>

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "HX711.h"
#include <EEPROM.h>
#include <math.h>

// -------------------- OLED 설정 --------------------
#define SDA_PIN 6
#define SCL_PIN 7
#define OLED_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

// -------------------- 핀 설정 --------------------
#define BUTTON_PIN 3
#define SPEAKER_PIN 2
#define HX_DT_PIN  4
#define HX_SCK_PIN 5

// -------------------- 타이밍 상수 --------------------
#define BUTTON_DEBOUNCE_MS 50
#define BLE_RESTART_DELAY_MS 500
#define LOOP_DELAY_MS 10
#define DISPLAY_UPDATE_INTERVAL 100

#define SHORT_PRESS_MS 500
#define TARE_PRESS_MS 1000
#define LONG_PRESS_MS 3000
#define HOLD_STABLE_TIME_MS 500  // 0.5초 동안 안정적일 때 HOLD

// -------------------- BLE UUID --------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// -------------------- EEPROM 설정 --------------------
#define EEPROM_SIZE 128
#define EEPROM_ADDR_A1  0
#define EEPROM_ADDR_B1  8
#define EEPROM_ADDR_A2  16
#define EEPROM_ADDR_B2  24
#define EEPROM_ADDR_M2  32
#define EEPROM_ADDR_SCALE_FACTOR 40   // 추가

// -------------------- HX711 scale --------------------
// #define scale_factor 52.60f   // 삭제
float scaleFactor = 52.60f;      // 변경

// -------------------- 실제 기준 추 무게 --------B------------
const float CAL_WEIGHT_1KG  = 1.48f;
const float CAL_WEIGHT_10KG = 14.41f;
const float CAL_WEIGHT_20KG = 19.84f;   // 실제 무게 재서 바꾸면 더 정확함

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HX711 scale;

// -------------------- Filter 관련 --------------------
#define LPF_SAMPLE_COUNT 8
#define IIR_ALPHA 0.4f
float stableMinWeight = 0.0f;

float lpfBuffer[LPF_SAMPLE_COUNT] = {0};
uint8_t lpfIndex = 0;
uint8_t lpfFilled = 0;

bool filterInitialized = false;
float filteredWeight = 0.0f;

// BLE 객체
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
NimBLEAdvertising* pAdvertising = nullptr;

// OTA 객체
const String otaSsid = "Refeed_Test_C3";
const char* otaPassword = "";   // 오픈 AP면 빈 문자열

const char* scaleUser = "admin";
const char* scalePass = "refeed123";

WebServer server(80);
bool otaMode = false;
bool scaleLoginOk = false;

// -------------------- 상태 변수 --------------------
bool speakerOn = false;
unsigned long speakerStart = 0;
const unsigned long SPEAKER_DURATION = 300;

volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;
bool displayInitialized = false;

unsigned long lastButtonPress = 0;
bool lastButtonState = HIGH;
bool buttonPressed = false;
unsigned long pressStart = 0;
bool longPressActive = false;
bool tareExecutedThisPress = false;

unsigned long lastConnectionTime = 0;
unsigned long connectionLostTime = 0;
bool connectionStable = false;

float currentWeight = 0.0f;
unsigned long lastDisplayUpdate = 0;

// -------------------- HOLD 관련 --------------------
bool isHold = false;
float holdWeight = 0.0f;
float prevWeight = 0.0f;
unsigned long stableStartTime = 0;

// -------------------- 3점 보정 계수 --------------------
float calA1 = 1.0f;
float calB1 = 0.0f;
float calA2 = 1.0f;
float calB2 = 0.0f;
float calBreakM2 = 10.05f;

// 캘리브레이션 측정값 저장
float calM1 = 0.0f;
float calM2 = 0.0f;
float calM3 = 0.0f;

// -------------------- 캘리브레이션 상태 --------------------
enum CalibrationState {
  CAL_IDLE,
  CAL_WAIT_START_RELEASE,
  CAL_WAIT_EMPTY_CONFIRM,
  CAL_WAIT_1KG_CONFIRM,
  CAL_SHOW_1KG_DONE,
  CAL_WAIT_10KG_CONFIRM,
  CAL_SHOW_10KG_DONE,
  CAL_WAIT_20KG_CONFIRM,
  CAL_SHOW_20KG_DONE,
  CAL_SHOW_AB1,
  CAL_SHOW_AB2,
  CAL_FINISH
};

CalibrationState calState = CAL_IDLE;
unsigned long calStateStart = 0;

// -------------------- 표시 함수 --------------------
void showConnectionStatus(bool connected) {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(0, 8);
  display.print(connected ? "BT ON" : "BT OFF");
  display.display();
}

void showReadyStatus() {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("READY");
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Scale: OK");
  display.display();
}

void showResult(float weight) {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(0, 8);
  display.print(String(weight, 1));
  display.print(" Kg");

  if (isHold) {
    display.setTextSize(1);
    display.setCursor(98, 0);
    display.print("HOLD");
  }

  display.display();
}

void showDataSent() {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 8);
  display.print("DATA SENT");
  display.display();
}

void showNoBT() {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 8);
  display.print("NO BT");
  display.display();
}

void showMessage(const String& msg, int textSize = 2) {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(textSize);
  display.setCursor(0, 8);
  display.print(msg);
  display.display();
}

void showAB1(float a, float b) {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("A1=");
  display.print(a, 4);
  display.setCursor(0, 16);
  display.print("B1=");
  display.print(b, 4);
  display.display();
}

void showAB2(float a, float b) {
  if (!displayInitialized) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("A2=");
  display.print(a, 4);
  display.setCursor(0, 16);
  display.print("B2=");
  display.print(b, 4);
  display.display();
}

// -------------------- EEPROM 함수 --------------------
void saveCalibrationData() {
  EEPROM.put(EEPROM_ADDR_A1, calA1);
  EEPROM.put(EEPROM_ADDR_B1, calB1);
  EEPROM.put(EEPROM_ADDR_A2, calA2);
  EEPROM.put(EEPROM_ADDR_B2, calB2);
  EEPROM.put(EEPROM_ADDR_M2, calBreakM2);
  EEPROM.commit();

  Serial.printf("SAVE: A1=%.6f, B1=%.6f, A2=%.6f, B2=%.6f, M2=%.6f\n",
                calA1, calB1, calA2, calB2, calBreakM2);
}

void loadCalibrationData() {
  EEPROM.get(EEPROM_ADDR_A1, calA1);
  EEPROM.get(EEPROM_ADDR_B1, calB1);
  EEPROM.get(EEPROM_ADDR_A2, calA2);
  EEPROM.get(EEPROM_ADDR_B2, calB2);
  EEPROM.get(EEPROM_ADDR_M2, calBreakM2);

  bool invalid =
    isnan(calA1) || isinf(calA1) || fabs(calA1) < 0.000001f || fabs(calA1) > 1000.0f ||
    isnan(calB1) || isinf(calB1) || fabs(calB1) > 1000.0f ||
    isnan(calA2) || isinf(calA2) || fabs(calA2) < 0.000001f || fabs(calA2) > 1000.0f ||
    isnan(calB2) || isinf(calB2) || fabs(calB2) > 1000.0f ||
    isnan(calBreakM2) || isinf(calBreakM2) || calBreakM2 <= 0.0f || calBreakM2 > 100.0f;

  if (invalid) {
    calA1 = 1.0f;
    calB1 = 0.0f;
    calA2 = 1.0f;
    calB2 = 0.0f;
    calBreakM2 = CAL_WEIGHT_10KG;
  }

  Serial.printf("LOAD: A1=%.6f, B1=%.6f, A2=%.6f, B2=%.6f, M2=%.6f\n",
                calA1, calB1, calA2, calB2, calBreakM2);
}

void saveScaleFactor() {
  EEPROM.put(EEPROM_ADDR_SCALE_FACTOR, scaleFactor);
  EEPROM.commit();
  Serial.printf("SAVE SCALE FACTOR: %.4f\n", scaleFactor);
}

void loadScaleFactor() {
  EEPROM.get(EEPROM_ADDR_SCALE_FACTOR, scaleFactor);

  if (isnan(scaleFactor) || isinf(scaleFactor) || fabs(scaleFactor) < 0.0001f || fabs(scaleFactor) > 100000.0f) {
    scaleFactor = 52.60f;   // 기본값
  }

  Serial.printf("LOAD SCALE FACTOR: %.4f\n", scaleFactor);
}

// -------------------- BLE 콜백 --------------------
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override {
    Serial.println("BLE connected");
    deviceConnected = true;
    connectionStable = true;
    lastConnectionTime = millis();
    showConnectionStatus(true);
  }

  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
    Serial.println("BLE disconnected");
    deviceConnected = false;
    connectionStable = false;
    connectionLostTime = millis();
    showConnectionStatus(false);
    pAdvertising->start();
  }
};

class MyCharCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    std::string v = c->getValue();

    if (!v.empty()) {
      Serial.print("[BLE RX] ");
      Serial.println(v.c_str());
    }
  }
};

// OTA 함수
void startOTAServer() {
  Serial.println("\n--- [ROA KITCHEN] Web OTA Mode Start ---");

  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(500);

  WiFi.mode(WIFI_AP);
  delay(200);

  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  String mac = WiFi.softAPmacAddress();
  String uniqueOtaSsid = otaSsid +
                         mac.substring(mac.length() - 5, mac.length() - 3) +
                         mac.substring(mac.length() - 2, mac.length());

  bool success = WiFi.softAP(uniqueOtaSsid, otaPassword, 1, 0, 2);

  if (!success) {
    Serial.println("OTA AP start FAIL");
    if (displayInitialized) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.print("OTA FAIL");
      display.display();
    }
    return;
  }

  IPAddress ip = WiFi.softAPIP();

  Serial.println("==========================================");
  Serial.print("AP READY! SSID: ");
  Serial.println(uniqueOtaSsid);
  Serial.print("URL: http://");
  Serial.println(ip);
  Serial.println("==========================================");

  if (displayInitialized) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("OTA MODE");
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print(ip.toString());
    display.display();
  }

  // 1) 메인 페이지: 펌웨어 업로드 + 설정 페이지 이동 버튼
  server.on("/", HTTP_GET, []() {
    String page = "<html><body>";
    page += "<h2>Refeed OTA</h2>";

    page += "<h3>Firmware Update</h3>";
    page += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    page += "<input type='file' name='update'><br><br>";
    page += "<input type='submit' value='Update Now'>";
    page += "</form>";

    page += "<hr>";
    page += "<h3>Protected Settings</h3>";
    page += "<a href='/login'><button type='button'>Scale Factor Login</button></a>";

    page += "</body></html>";
    server.send(200, "text/html", page);
  });

  // 2) 로그인 페이지
  server.on("/login", HTTP_GET, []() {
    String page = "<html><body>";
    page += "<h2>Scale Factor Login</h2>";
    page += "<form method='POST' action='/login'>";
    page += "ID: <input type='text' name='username'><br><br>";
    page += "PW: <input type='password' name='password'><br><br>";
    page += "<input type='submit' value='Login'>";
    page += "</form>";
    page += "<br><a href='/'>Back</a>";
    page += "</body></html>";
    server.send(200, "text/html", page);
  });

  // 3) 로그인 처리
  server.on("/login", HTTP_POST, []() {
    if (!server.hasArg("username") || !server.hasArg("password")) {
      server.send(400, "text/plain", "Missing credentials");
      return;
    }

    String username = server.arg("username");
    String password = server.arg("password");

    if (username == scaleUser && password == scalePass) {
      scaleLoginOk = true;
      server.sendHeader("Location", "/scale", true);
      server.send(302, "text/plain", "");
      return;
    }

    String page = "<html><body>";
    page += "<h2>Login Failed</h2>";
    page += "<p>Invalid ID or password.</p>";
    page += "<a href='/login'>Try Again</a>";
    page += "</body></html>";
    server.send(401, "text/html", page);
  });

  // 4) scaleFactor 설정 페이지
  server.on("/scale", HTTP_GET, []() {
    if (!scaleLoginOk) {
      server.sendHeader("Location", "/login", true);
      server.send(302, "text/plain", "");
      return;
    }

    String page = "<html><body>";
    page += "<h2>Scale Factor Setting</h2>";
    page += "<p>Current scaleFactor: <b>" + String(scaleFactor, 4) + "</b></p>";
    page += "<form method='POST' action='/set_scale'>";
    page += "<input type='number' step='0.0001' name='value' value='" + String(scaleFactor, 4) + "'>";
    page += "<input type='submit' value='Save Scale Factor'>";
    page += "</form>";
    page += "<br><a href='/'>Back</a>";
    page += "</body></html>";
    server.send(200, "text/html", page);
  });

  // 5) scaleFactor 저장
  server.on("/set_scale", HTTP_POST, []() {
    if (!scaleLoginOk) {
      server.sendHeader("Location", "/login", true);
      server.send(302, "text/plain", "");
      return;
    }

    if (!server.hasArg("value")) {
      server.send(400, "text/plain", "Missing value");
      return;
    }

    float newValue = server.arg("value").toFloat();

    if (isnan(newValue) || isinf(newValue) || fabs(newValue) < 0.0001f || fabs(newValue) > 100000.0f) {
      server.send(400, "text/plain", "Invalid scale factor");
      return;
    }

    scaleFactor = newValue;
    saveScaleFactor();

    String msg = "<html><body>";
    msg += "<h2>Saved</h2>";
    msg += "<p>scaleFactor = <b>" + String(scaleFactor, 4) + "</b></p>";
    msg += "<p>Rebooting...</p>";
    msg += "</body></html>";

    server.send(200, "text/html", msg);
    delay(1000);
    ESP.restart();
  });

  // 6) 펌웨어 업데이트
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain",
                Update.hasError() ? "Update FAIL" : "Update SUCCESS! Rebooting...");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update Start: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
  otaMode = true;
}

// -------------------- BLE 초기화 --------------------
void initializeBLE() {
  Serial.println("NimBLE init...");

  NimBLEDevice::init("Refeedinc");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ |
    NIMBLE_PROPERTY::WRITE |
    NIMBLE_PROPERTY::NOTIFY
  );

  pCharacteristic->setValue("0.0");
  pCharacteristic->setCallbacks(new MyCharCallbacks());

  pService->start();

  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("NimBLE advertising started");
}

// -------------------- 스피커 제어 --------------------
void triggerSpeakerBeep() {
  speakerOn = true;
  speakerStart = millis();
  digitalWrite(SPEAKER_PIN, HIGH);
}

void handleSpeaker() {
  if (speakerOn && millis() - speakerStart >= SPEAKER_DURATION) {
    speakerOn = false;
    digitalWrite(SPEAKER_PIN, LOW);
  }
}

// -------------------- 측정 함수 --------------------
float readBaseMeasuredKg(uint8_t times = 1) {
  float raw = scale.get_units(times);
  float measuredKg = fabs(raw / 1000.0f);
  return measuredKg;
}

float getCorrectedWeightKg(uint8_t times = 1) {
  float baseMeasuredKg = readBaseMeasuredKg(times);
  float corrected = 0.0f;

  if (baseMeasuredKg <= calBreakM2) {
    corrected = calA1 * baseMeasuredKg + calB1;
  } else {
    corrected = calA2 * baseMeasuredKg + calB2;
  }

  if (corrected < 0.0f) corrected = 0.0f;
  return corrected;
}

float applyLPF(float input) {
  lpfBuffer[lpfIndex] = input;
  lpfIndex = (lpfIndex + 1) % LPF_SAMPLE_COUNT;

  if (lpfFilled < LPF_SAMPLE_COUNT) {
    lpfFilled++;
  }

  float sum = 0.0f;
  for (uint8_t i = 0; i < lpfFilled; i++) {
    sum += lpfBuffer[i];
  }

  return sum / lpfFilled;
}

float applyIIR(float input) {
  if (!filterInitialized) {
    filteredWeight = input;
    filterInitialized = true;
  } else {
    filteredWeight += IIR_ALPHA * (input - filteredWeight);
  }

  return filteredWeight;
}

float getFilteredWeightKg(uint8_t times = 3) {
  float raw = getCorrectedWeightKg(times);
  float lpf = applyLPF(raw);
  float iir = applyIIR(lpf);

  if (iir < 0.0f) iir = 0.0f;
  return iir;
}

void resetFilterState() {
  for (int i = 0; i < LPF_SAMPLE_COUNT; i++) {
    lpfBuffer[i] = 0.0f;
  }
  lpfIndex = 0;
  lpfFilled = 0;
  filterInitialized = false;
  filteredWeight = 0.0f;
}

// -------------------- HOLD / TARE 보조 --------------------
void clearHoldState() {
  isHold = false;
  holdWeight = 0.0f;
  stableStartTime = 0;
  prevWeight = 0.0f;
  stableMinWeight = 0.0f;
}

void performTare() {
  scale.tare();
  delay(200);

  resetFilterState();
  clearHoldState();
  currentWeight = getFilteredWeightKg(2);

  showMessage("TARE OK", 2);
  triggerSpeakerBeep();
  // ESP.restart();        // 하드웨어 리부팅 (전원 껐다 켠 효과)
  Serial.println("TARE executed");
}

// -------------------- 캘리브레이션 시작 --------------------
void startCalibration() {
  Serial.println("Calibration start");
  scale.tare();
  delay(200);

  resetFilterState();
  clearHoldState();
  
  calM1 = 0.0f;
  calM2 = 0.0f;
  calM3 = 0.0f;

  calState = CAL_WAIT_START_RELEASE;
  showMessage("EMPTY", 2);
}

// -------------------- 캘리브레이션 상태 처리 --------------------
void handleCalibrationState() {
  switch (calState) {
    case CAL_IDLE:
    case CAL_WAIT_START_RELEASE:
    case CAL_WAIT_EMPTY_CONFIRM:
    case CAL_WAIT_1KG_CONFIRM:
    case CAL_WAIT_10KG_CONFIRM:
    case CAL_WAIT_20KG_CONFIRM:
      break;

    case CAL_SHOW_1KG_DONE:
      if (millis() - calStateStart >= 1200) {
        calState = CAL_WAIT_10KG_CONFIRM;
        showMessage("Put 10kg", 2);
      }
      break;

    case CAL_SHOW_10KG_DONE:
      if (millis() - calStateStart >= 1200) {
        calState = CAL_WAIT_20KG_CONFIRM;
        showMessage("Put 20kg", 2);
      }
      break;

    case CAL_SHOW_20KG_DONE:
      if (millis() - calStateStart >= 1200) {
        bool ok12 = fabs(calM2 - calM1) > 0.001f;
        bool ok23 = fabs(calM3 - calM2) > 0.001f;

        if (ok12 && ok23) {
          calA1 = (CAL_WEIGHT_10KG - CAL_WEIGHT_1KG) / (calM2 - calM1);
          calB1 = CAL_WEIGHT_1KG - calA1 * calM1;

          calA2 = (CAL_WEIGHT_20KG - CAL_WEIGHT_10KG) / (calM3 - calM2);
          calB2 = CAL_WEIGHT_10KG - calA2 * calM2;

          calBreakM2 = calM2;

          saveCalibrationData();

          Serial.printf("CAL DONE: A1=%.6f, B1=%.6f, A2=%.6f, B2=%.6f\n",
                        calA1, calB1, calA2, calB2);

          calState = CAL_SHOW_AB1;
          calStateStart = millis();
          showAB1(calA1, calB1);
        } else {
          Serial.println("CAL FAIL");
          showMessage("CAL FAIL", 2);
          calState = CAL_FINISH;
          calStateStart = millis();
        }
      }
      break;

    case CAL_SHOW_AB1:
      if (millis() - calStateStart >= 2500) {
        calState = CAL_SHOW_AB2;
        calStateStart = millis();
        showAB2(calA2, calB2);
      }
      break;

    case CAL_SHOW_AB2:
      if (millis() - calStateStart >= 2500) {
        calState = CAL_FINISH;
        calStateStart = millis();
        showReadyStatus();
      }
      break;

    case CAL_FINISH:
      if (millis() - calStateStart >= 300) {
        calState = CAL_IDLE;
      }
      break;
  }
}

// -------------------- BLE 전송 --------------------
void sendBLEData(const String& data) {
  if (deviceConnected && pCharacteristic != nullptr) {
    pCharacteristic->setValue((uint8_t*)data.c_str(), data.length());
    pCharacteristic->notify();
    Serial.printf("BLE TX: %s\n", data.c_str());
    triggerSpeakerBeep();
  } else {
    Serial.println("BLE not connected");
  }
}

void performMeasurement() {
  float sendWeight = isHold ? holdWeight : currentWeight;

  Serial.print("Send weight: ");
  Serial.println(sendWeight, 1);

  if (deviceConnected && pCharacteristic != nullptr) {
    String dataToSend = String(sendWeight, 1) + "@%^";
    sendBLEData(dataToSend);
    showDataSent();
  } else {
    showNoBT();
    triggerSpeakerBeep();
  }
}

// -------------------- 버튼 처리 --------------------
void handleButtonInput() {
  bool cur = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  if (cur == LOW && lastButtonState == HIGH) {
    pressStart = now;
    tareExecutedThisPress = false;
  }

  // 평상시 1초 이상 누르면 TARE 1회 실행
  if (calState == CAL_IDLE && cur == LOW && !tareExecutedThisPress && (now - pressStart >= TARE_PRESS_MS)) {
    tareExecutedThisPress = true;
    performTare();
  }

  // 평상시 3초 길게 누르면 캘리브레이션 시작
  if (calState == CAL_IDLE && cur == LOW && !longPressActive && (now - pressStart >= LONG_PRESS_MS)) {
    longPressActive = true;
    startCalibration();
  }

  if (cur != lastButtonState) {
    if (now - lastButtonPress > BUTTON_DEBOUNCE_MS) {
      if (cur == LOW && !buttonPressed) {
        buttonPressed = true;
      } else if (cur == HIGH && buttonPressed) {
        unsigned long pressDuration = now - pressStart;

        if (calState != CAL_IDLE) {
          if (calState == CAL_WAIT_START_RELEASE) {
            calState = CAL_WAIT_EMPTY_CONFIRM;
            Serial.println("Calibration release ok");
          }
          else if (calState == CAL_WAIT_EMPTY_CONFIRM) {
            calState = CAL_WAIT_1KG_CONFIRM;
            showMessage("Put 1kg", 2);
          }
          else if (calState == CAL_WAIT_1KG_CONFIRM) {
            calM1 = readBaseMeasuredKg(15);
            Serial.printf("1kg measured: %.4f kg\n", calM1);
            showMessage("CAL DONE", 2);
            calState = CAL_SHOW_1KG_DONE;
            calStateStart = millis();
          }
          else if (calState == CAL_WAIT_10KG_CONFIRM) {
            calM2 = readBaseMeasuredKg(15);
            Serial.printf("10kg measured: %.4f kg\n", calM2);
            showMessage("CAL DONE", 2);
            calState = CAL_SHOW_10KG_DONE;
            calStateStart = millis();
          }
          else if (calState == CAL_WAIT_20KG_CONFIRM) {
            calM3 = readBaseMeasuredKg(15);
            Serial.printf("20kg measured: %.4f kg\n", calM3);
            showMessage("CAL DONE", 2);
            calState = CAL_SHOW_20KG_DONE;
            calStateStart = millis();
          }
        } else {
          // 0.5초 이하 짧은 클릭이면 현재 LCD 값 전송
          if (!longPressActive && !tareExecutedThisPress && pressDuration <= SHORT_PRESS_MS) {
            performMeasurement();
          }
        }

        buttonPressed = false;
        longPressActive = false;
      }

      lastButtonPress = now;
    }

    lastButtonState = cur;
  }
}

// -------------------- BLE 연결 관리 --------------------
void handleBLEConnection() {
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Advertising restart");
    delay(BLE_RESTART_DELAY_MS);
    if (pAdvertising) pAdvertising->start();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void monitorConnectionQuality() {
  if (deviceConnected) {
    unsigned long connectedTime = millis() - lastConnectionTime;
    if (connectedTime > 5000 && !connectionStable) {
      connectionStable = true;
      Serial.println("BLE stable");
    }
  } else if (connectionLostTime > 0) {
    unsigned long disconnectedTime = millis() - connectionLostTime;
    if (disconnectedTime > 30000) {
      Serial.println("BLE re-advertise");
      if (pAdvertising) pAdvertising->start();
      connectionLostTime = 0;
    }
  }
}

// -------------------- setup --------------------
void setup() {
  Serial.begin(115200);
  Serial.println("System start");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);

  // OLED 먼저 켜서 OTA 표시 가능하게
  Wire.begin(SDA_PIN, SCL_PIN);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    displayInitialized = true;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("BOOT");
    display.display();
  }

  EEPROM.begin(EEPROM_SIZE);
  loadCalibrationData();
  loadScaleFactor();

  // 전원 켤 때 버튼 눌려 있으면 OTA 모드 진입
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(1500);
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Enter OTA mode");
      startOTAServer();
      return;
    }
  }

  // 여기부터 일반 모드 초기화
  if (displayInitialized) {
    display.clearDisplay();
    display.setTextSize(3);
    display.setCursor(0, 8);
    display.print("ReFeed");
    display.display();
    Serial.println("OLED OK");
  } else {
    Serial.println("OLED FAIL");
  }

  scale.begin(HX_DT_PIN, HX_SCK_PIN);
  scale.set_scale(scaleFactor);
  scale.tare();
  delay(200);
  Serial.println("HX711 OK");

  initializeBLE();
  showReadyStatus();
}

// -------------------- loop --------------------
void loop() {

  if (otaMode) {
    server.handleClient();

    static int last_conn = -1;
    int current_conn = WiFi.softAPgetStationNum();
    if (current_conn != last_conn) {
      Serial.printf("Current Connections: %d\n", current_conn);
      last_conn = current_conn;
    }

    delay(10);
    return;
  }

  handleButtonInput();
  handleCalibrationState();
  handleBLEConnection();
  monitorConnectionQuality();
  handleSpeaker();

  if (calState == CAL_IDLE && millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL) {
    currentWeight = getFilteredWeightKg(2);

    if (!isHold) {
      if (currentWeight < 0.1f) {
          // 0.1kg 미만일 때는 Hold 로직을 건너뛰고 변수만 초기화
          stableStartTime = 0;
          stableMinWeight = 0.0f;
      } 
      else if (currentWeight >= 1.0f && fabs(currentWeight - prevWeight) < 0.02f) {
        if (stableStartTime == 0) {
          stableStartTime = millis();
          stableMinWeight = currentWeight;
        } else {
          if (currentWeight < stableMinWeight) {
            stableMinWeight = currentWeight;
          }
        }

        if (millis() - stableStartTime >= HOLD_STABLE_TIME_MS) {
          isHold = true;
          holdWeight = stableMinWeight;   // 최소값으로 확정
          triggerSpeakerBeep();
          Serial.printf("HOLD: %.2f kg\n", holdWeight);
        }
      } else {
        stableStartTime = 0;
        stableMinWeight = 0.0f;
      }

      // prevWeight 업데이트를 if/else 밖으로 빼서 정상적으로 갱신되도록 함
      prevWeight = currentWeight;
    }

    float baseMeasuredKg = readBaseMeasuredKg(3);

    Serial.printf("display=%.2f kg / raw=%.2f kg / hold=%d / A1=%.4f / B1=%.4f / A2=%.4f / B2=%.4f\n",
                  isHold ? holdWeight : currentWeight,
                  baseMeasuredKg,
                  isHold,
                  calA1, calB1, calA2, calB2);

    // 이제 return 되지 않고 무조건 여기로 내려오므로 화면이 갱신됩니다.
    showResult(isHold ? holdWeight : currentWeight);
    lastDisplayUpdate = millis();
  }

  delay(LOOP_DELAY_MS);
}