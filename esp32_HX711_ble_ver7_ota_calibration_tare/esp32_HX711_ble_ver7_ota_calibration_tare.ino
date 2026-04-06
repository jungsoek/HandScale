// ===== ESP32-C3/S3 BLE (Bluedroid) + OLED + HX711 + EEPROM 3-Point Calibration =====
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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

// -------------------- EEPROM 설정 --------------------
#define EEPROM_SIZE 128
#define EEPROM_ADDR_A1  0
#define EEPROM_ADDR_B1  8
#define EEPROM_ADDR_A2  16
#define EEPROM_ADDR_B2  24
#define EEPROM_ADDR_M2  32   // 구간 분기 기준용 (10.05kg 측정점)

// -------------------- BLE UUID --------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// -------------------- HX711 scale --------------------
#define scale_factor 60.0f

// -------------------- 실제 기준 추 무게 --------------------
const float CAL_WEIGHT_1KG  = 1.12f;
const float CAL_WEIGHT_10KG = 13.63f;
const float CAL_WEIGHT_20KG = 19.85f;   // 실제 무게 재서 바꾸면 더 정확함

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HX711 scale;

// BLE 객체
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
BLEAdvertising* pAdvertising = nullptr;

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

// -------------------- BLE 콜백 --------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    Serial.println("BLE connected");
    deviceConnected = true;
    connectionStable = true;
    lastConnectionTime = millis();
    showConnectionStatus(true);
  }

  void onDisconnect(BLEServer* s) override {
    Serial.println("BLE disconnected");
    deviceConnected = false;
    connectionStable = false;
    connectionLostTime = millis();
    showConnectionStatus(false);
    s->getAdvertising()->start();
  }
};

class MyCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    if (v.length() > 0) {
      Serial.print("[BLE RX] ");
      Serial.println(v);
    }
  }
};

// -------------------- BLE 초기화 --------------------
void initializeBLE() {
  Serial.println("BLE init...");

  BLEDevice::init("Refeedinc");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharCallbacks());
  pCharacteristic->setValue("0@%^");

  pService->start();

  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE advertising started");
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

// -------------------- HOLD / TARE 보조 --------------------
void clearHoldState() {
  isHold = false;
  holdWeight = 0.0f;
  stableStartTime = 0;
  prevWeight = currentWeight;
}

void performTare() {
  scale.tare();
  delay(200);

  clearHoldState();
  currentWeight = getCorrectedWeightKg(3);

  showMessage("TARE OK", 2);
  triggerSpeakerBeep();

  Serial.println("TARE executed");
}

// -------------------- 캘리브레이션 시작 --------------------
void startCalibration() {
  Serial.println("Calibration start");
  scale.tare();
  delay(200);

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

  EEPROM.begin(EEPROM_SIZE);
  loadCalibrationData();

  // OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    displayInitialized = true;
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 8);
    display.print("REFEED");
    display.display();
    Serial.println("OLED OK");
  } else {
    Serial.println("OLED FAIL");
  }

  // HX711
  scale.begin(HX_DT_PIN, HX_SCK_PIN);
  scale.set_scale(scale_factor);
  scale.tare();
  delay(200);
  Serial.println("HX711 OK");

  initializeBLE();
  showReadyStatus();
}

// -------------------- loop --------------------
void loop() {
  handleButtonInput();
  handleCalibrationState();
  handleBLEConnection();
  monitorConnectionQuality();
  handleSpeaker();

  if (calState == CAL_IDLE && millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL) {
    currentWeight = getCorrectedWeightKg(1);

    // -------------------- HOLD 로직 --------------------
    if (!isHold) {
      if (currentWeight >= 1.0f && fabs(currentWeight - prevWeight) < 0.05f) {
        if (stableStartTime == 0) stableStartTime = millis();

        if (millis() - stableStartTime >= 500) {
          isHold = true;
          holdWeight = currentWeight;
          triggerSpeakerBeep();
          Serial.printf("HOLD: %.2f kg\n", holdWeight);
        }
      } else {
        stableStartTime = 0;
      }

      prevWeight = currentWeight;
    }

    float baseMeasuredKg = readBaseMeasuredKg(1);
    Serial.printf("display=%.2f kg / base=%.2f kg / hold=%d / A1=%.4f / B1=%.4f / A2=%.4f / B2=%.4f\n",
                  isHold ? holdWeight : currentWeight,
                  baseMeasuredKg,
                  isHold,
                  calA1, calB1, calA2, calB2);

    showResult(isHold ? holdWeight : currentWeight);
    lastDisplayUpdate = millis();
  }

  delay(LOOP_DELAY_MS);
}