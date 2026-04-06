#include <Wire.h>
#include <ICM20948.h> // 사용하는 라이브러리에 따라 헤더는 달라질 수 있음

ICM20948 imu;

// 원점 저장을 위한 변수 (오프셋)
float offsetRoll = 0;
float offsetPitch = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // ESP32-C3 Mini의 기본 I2C 핀 (SDA: 8, SCL: 9)
  
  if (!imu.begin()) {
    Serial.println("ICM-20948 연결 실패!");
    while (1);
  }
}

// 현재 상태를 '수평(0도)'으로 설정하는 함수
void calibrateLevel() {
  float sumRoll = 0, sumPitch = 0;
  int samples = 100;

  Serial.println("수평 원점을 잡는 중... 기기를 움직이지 마세요.");
  
  for (int i = 0; i < samples; i++) {
    imu.readSensor();
    float ax = imu.getAccelX_mss();
    float ay = imu.getAccelY_mss();
    float az = imu.getAccelZ_mss();

    // 현재 가속도 기반 각도 계산
    sumRoll += atan2(ay, az) * 180.0 / PI;
    sumPitch += atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    delay(10);
  }

  offsetRoll = sumRoll / samples;
  offsetPitch = sumPitch / samples;
  
  Serial.print("원점 설정 완료! Roll Offset: ");
  Serial.print(offsetRoll);
  Serial.print(", Pitch Offset: ");
  Serial.println(offsetPitch);
}

void loop() {
  imu.readSensor();
  
  // 1. 현재 가도 측정
  float rawRoll = atan2(imu.getAccelY_mss(), imu.getAccelZ_mss()) * 180.0 / PI;
  float rawPitch = atan2(-imu.getAccelX_mss(), sqrt(imu.getAccelY_mss() * imu.getAccelY_mss() + imu.getAccelZ_mss() * imu.getAccelZ_mss())) * 180.0 / PI;

  // 2. 원점(Offset) 적용하여 최종 각도 계산
  float finalRoll = rawRoll - offsetRoll;
  float finalPitch = rawPitch - offsetPitch;

  // 3. 로드셀 보정에 사용할 통합 기울기(theta) 계산
  // 3차원 공간에서의 절대 기울기 각도
  float theta = sqrt(finalRoll * finalRoll + finalPitch * finalPitch);
  
  // 4. 로드셀 값 보정 (예시)
  // float correctedWeight = rawLoadcellValue / cos(theta * PI / 180.0);

  Serial.printf("Roll: %.2f | Pitch: %.2f | Total Tilt: %.2f\n", finalRoll, finalPitch, theta);
  delay(100);
}