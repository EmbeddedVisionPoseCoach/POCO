#include <Wire.h>   // MPU6050과 I2C 통신을 하기 위한 라이브러리
#include <Servo.h>  // 서보 모터 제어 라이브러리

// 센서 값을 쉽게 읽기 위한 라이브러리
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

// =====================================================
// 객체 생성
// =====================================================
Adafruit_MPU6050 mpu;     // MPU6050 기울기 센서 객체
Servo cameraServo;        // 카메라 수평 보정용 서보모터 객체

// =====================================================
// 핀 설정
// =====================================================
const int SERVO_PIN = 12;       // 서보모터 신호선 연결 핀
const int BUZZER_PIN = 8;       // 부저 연결 핀
const int NEOPIXEL_PIN = 6;     // 네오픽셀 DIN 연결 핀

const int LED_COUNT = 8;        // 네오픽셀 LED 개수

// 네오픽셀 객체 생성
// NEO_GRB : 색상 순서가 GRB 방식인 네오픽셀 사용
// NEO_KHZ800 : 대부분의 WS2812B 네오픽셀이 사용하는 통신 속도
Adafruit_NeoPixel strip(LED_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// =====================================================
// 서보모터 설정
// =====================================================
int servoAngle = 170;            // 서보모터 시작 각도

const int SERVO_MIN = 146;       // 서보모터 최소 각도 제한
const int SERVO_MAX = 180;      // 서보모터 최대 각도 제한

const float TARGET_ANGLE = 0.0; // 목표 수평 각도
const float TOLERANCE = 0.5;    // ±1.5도 이내면 수평으로 판단

const int SERVO_STEP = 1;       // 한 번 움직일 때 변경할 각도
const int MOVE_DELAY = 80;      // 서보모터 이동 후 대기 시간

const int MAX_LEVELING_COUNT = 150; // 수평 보정 최대 반복 횟수



// =====================================================
// 사용자 설정 알림 횟수
// PyQt → Python → Arduino 로 전달받아 변경될 값
//
// 사용자가 설정 가능:
// - 자세 알림 횟수
// - 졸음 알림 횟수
//
// 사용자가 설정 불가:
// - ON/OFF 시간(ms)
// =====================================================

// 자세 알림 횟수
// Asymmetry, ForwardHead, ChinPropping
int postureAlertCount = 3;


// 졸음 알림 횟수
// Drowsy
int drowsyAlertCount = 3;


// =====================================================
// setup
// 아두이노가 처음 켜질 때 한 번 실행되는 부분
// =====================================================
void setup() {
  Serial.begin(115200);   // 라즈베리파이와 시리얼 통신 속도
  Wire.begin();           // I2C 통신 시작

  pinMode(BUZZER_PIN, OUTPUT);

  // 부저 초기 상태 OFF
  digitalWrite(BUZZER_PIN, LOW);

  // 네오픽셀 초기화
  strip.begin();          // 네오픽셀 사용 시작
  strip.setBrightness(30); // 밝기 설정, 0~255
  strip.show();           // 현재 설정을 LED에 반영, 처음에는 꺼진 상태

  // 서보모터 초기화
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoAngle);
  delay(500);

  // MPU6050 연결 확인
  if (!mpu.begin()) {
    Serial.println("MPU6050_FAIL");

    // 센서 연결 실패 시 더 이상 진행하지 않음
    while (1) {
      delay(10);
    }
  }

  // MPU6050 측정 범위 설정
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // 가속도 측정 범위
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // 자이로 측정 범위
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // 필터 대역폭 설정

  // 라즈베리파이에서 아두이노 준비 상태 확인용
  Serial.println("ARDUINO_READY");
}

// =====================================================
// loop
// 계속 반복 실행되는 부분
// 라즈베리파이에서 문자열 명령을 받으면 해당 기능 실행
// =====================================================
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.print("Received:");
    Serial.println(command);

    // =====================================================
    // 자세 알림 횟수 설정
    // -----------------------------------------------------
    // 라즈베리파이에서 "SET_POSTURE_COUNT:5" 형태로 보내면
    // postureAlertCount 값을 5로 변경한다.
    //
    // 예:
    // SET_POSTURE_COUNT:3
    // SET_POSTURE_COUNT:5
    // =====================================================
    if (command.startsWith("SET_POSTURE_COUNT:")) {
      int count = command.substring(18).toInt();

      // 너무 작거나 큰 값이 들어오는 것을 방지
      count = constrain(count, 1, 10);

      postureAlertCount = count;

      Serial.print("POSTURE_COUNT_SET:");
      Serial.println(postureAlertCount);

      return;
    }

    // =====================================================
    // 졸음 알림 횟수 설정
    // -----------------------------------------------------
    // 라즈베리파이에서 "SET_DROWSY_COUNT:5" 형태로 보내면
    // drowsyAlertCount 값을 5로 변경한다.
    //
    // 예:
    // SET_DROWSY_COUNT:3
    // SET_DROWSY_COUNT:5
    // =====================================================
    if (command.startsWith("SET_DROWSY_COUNT:")) {
      int count = command.substring(17).toInt();

      // 너무 작거나 큰 값이 들어오는 것을 방지
      count = constrain(count, 1, 10);

      drowsyAlertCount = count;

      Serial.print("DROWSY_COUNT_SET:");
      Serial.println(drowsyAlertCount);

      return;
    }
    

    // 카메라 수평 보정 시작
    if (command == "START_LEVELING") {
      Serial.println("LEVELING_START");
      autoLevelCameraOnce();
    }

    // 정상 자세 초록 (Green)
    else if (command == "Optimal") {
      blinkFeedback(
        0, 255, 0,

        1,    // LED 깜빡임 횟수
        3000,  // ON 시간(ms)
        0,  // OFF 시간(ms)
        0, 0, 0
      );
    }

    // 비대칭 하늘색 / 시안(Cyan 계열)
    else if (command == "Asymmetric") {
      blinkFeedback(
        0, 180, 255,

        postureAlertCount,  // LED 깜빡임 횟수
        500,    // ON 시간(ms)
        500,    // OFF 시간(ms)

        postureAlertCount,  // 부저 횟수
        200,    // 부저 ON(ms)
        200     // 부저 OFF(ms)

      );
    }

    // 거북목 주황 (Orange)
    else if (command == "ForwardHead") {
      blinkFeedback(
        255, 120, 0,

        postureAlertCount,  // LED 깜빡임 횟수
        500,    // ON 시간(ms)
        500,    // OFF 시간(ms)

        postureAlertCount,  // 부저 횟수
        200,    // 부저 ON(ms)
        200     // 부저 OFF(ms)
      );
    }

    // 턱굄 빨강 (Red)
    else if (command == "ChinPropping") {
      blinkFeedback(
        255, 0, 0,

        postureAlertCount,  // LED 깜빡임 횟수
        500,    // ON 시간(ms)
        500,    // OFF 시간(ms)

        postureAlertCount,  // 부저 횟수
        200,    // 부저 ON(ms)
        200     // 부저 OFF(ms)

      );
    }

    // ==================================================
    // 졸림 상태(Drowsy)
    // --------------------------------------------------
    // Face 모델에서 Drowsy가 감지되면
    // 자세보다 우선해서 알림을 준다.
    //
    // 색상:
    // 보라색 (255,0,255)
    //
    // 동작:
    // LED drowsyAlertCount회 점멸
    // 부저 drowsyAlertCount회 출력
    //
    // 우선순위:
    // Drowsy
    // ↓
    // ChinPropping
    // ↓
    // ForwardHead
    // ↓
    // Asymmetric
    // ↓
    // Optimal
    // ==================================================

    else if (command == "Drowsy") {

        blinkFeedback(

            // RGB 색상
            // 보라색
            255, 0, 255,

            // LED 점멸
            drowsyAlertCount,  // LED 점멸 횟수
            500,    // ON(ms)
            500,    // OFF(ms)

            // 부저 출력
            drowsyAlertCount,  // 부저 출력 횟수
            200,    // ON(ms)
            200     // OFF(ms)

        );
    }

    // ==================================================
    // 강한 경고 알람 (StrongAlert)
    // --------------------------------------------------
    // 같은 자세/졸음 알람이 반복되어
    // 설정 횟수에 도달했을 때 실행
    //
    // 예:
    // ForwardHead 반복
    // ↓
    // StrongAlert
    //
    // 특징:
    // - 빨강 LED
    // - 부저 더 많이 반복
    // ==================================================
    else if (command == "StrongAlert") {

        blinkFeedback(

            // RGB
            // 강한 경고: 빨강
            255, 0, 0,


            // LED
            6,      // 횟수 (강하게)
            200,    // ON(ms)
            100,    // OFF(ms)


            // 부저
            6,      // 횟수
            200,    // ON(ms)
            100     // OFF(ms)

        );

        Serial.println(
            "STRONG_ALERT_DONE"
        );
    }



  }
}

// =====================================================
// 카메라 수평 보정 함수
// MPU6050에서 현재 기울기를 읽고,
// 목표 각도에 가까워질 때까지 서보모터를 조금씩 움직임
// =====================================================
void autoLevelCameraOnce() {
  int count = 0;

  while (true) {
    float angle = getRollAngle();
    float error = angle - TARGET_ANGLE;

    Serial.print("ANGLE:");
    Serial.print(angle);
    Serial.print(",SERVO:");
    Serial.println(servoAngle);

    // 목표 각도와 현재 각도 차이가 허용 범위 안이면 보정 완료
    if (abs(error) <= TOLERANCE) {
      Serial.println("LEVELING_DONE");
      return;
    }

    // 너무 오래 반복하면 실패 처리
    if (count >= MAX_LEVELING_COUNT) {
      Serial.println("LEVELING_FAILED");
      return;
    }

    // 기울어진 방향에 따라 서보 각도 조정
    // 실제 움직임 방향이 반대라면 +, -만 서로 바꾸면 됨
    if (error > 0) {
      servoAngle += SERVO_STEP;
    } else {
      servoAngle -= SERVO_STEP;
    }

    // 서보모터가 제한 각도를 넘지 않도록 보호
    servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);

    // 서보모터 이동
    cameraServo.write(servoAngle);

    count++;
    delay(MOVE_DELAY);
  }
}

// =====================================================
// MPU6050 Roll 각도 계산 함수
// y축, z축 가속도 값을 이용해서 기울기 각도를 계산
// =====================================================
float getRollAngle() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float roll = atan2(ay, az) * 180.0 / PI;

  return roll;
}


// =====================================================
// 알람 정지 함수
// 부저와 네오픽셀을 모두 끔
// =====================================================
void stopAlarm() {
  digitalWrite(BUZZER_PIN, LOW);
  turnOffNeoPixel();

  Serial.println("ALARM_STOP");
}

// =====================================================
// 네오픽셀 전체 색상 설정 함수
// r, g, b 값은 각각 0~255 범위
// =====================================================
void setAllColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }

  strip.show();
}

// =====================================================
// 네오픽셀 끄기 함수
// 모든 LED 색상을 검정, 즉 OFF 상태로 설정
// =====================================================
void turnOffNeoPixel() {
  setAllColor(0, 0, 0);
}

// =====================================================
// 네오픽셀 + 부저 동시 시작 피드백 함수
// 네오픽셀과 부저가 동시에 시작하지만,
// 각각 반복 횟수와 ON/OFF 시간이 다르게 동작함
// =====================================================
void blinkFeedback(
  uint8_t r,
  uint8_t g,
  uint8_t b,

  int pixelBlinkCount,
  unsigned long pixelOnTime,
  unsigned long pixelOffTime,

  int buzzerBlinkCount,
  unsigned long buzzerOnTime,
  unsigned long buzzerOffTime
) {
  int pixelCount = 0;
  int buzzerCount = 0;

  bool pixelOn = false;
  bool buzzerOn = false;

  bool pixelDone = (pixelBlinkCount == 0);
  bool buzzerDone = (buzzerBlinkCount == 0);

  unsigned long pixelPreviousTime = millis();
  unsigned long buzzerPreviousTime = millis();

  // 처음 시작할 때 네오픽셀 ON
  if (!pixelDone) {
    setAllColor(r, g, b);
    pixelOn = true;
  }

  // 처음 시작할 때 부저 ON
  if (!buzzerDone) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
  }

  // 네오픽셀과 부저가 둘 다 끝날 때까지 반복
  while (!pixelDone || !buzzerDone) {
    unsigned long now = millis();

    // =========================
    // 네오픽셀 시간 체크
    // =========================
    if (!pixelDone) {
      if (pixelOn && now - pixelPreviousTime >= pixelOnTime) {
        turnOffNeoPixel();
        pixelOn = false;
        pixelPreviousTime = now;
      }

      else if (!pixelOn && now - pixelPreviousTime >= pixelOffTime) {
        pixelCount++;

        if (pixelCount >= pixelBlinkCount) {
          pixelDone = true;
          turnOffNeoPixel();
        } else {
          setAllColor(r, g, b);
          pixelOn = true;
          pixelPreviousTime = now;
        }
      }
    }

    // =========================
    // 부저 시간 체크
    // =========================
    if (!buzzerDone) {
      if (buzzerOn && now - buzzerPreviousTime >= buzzerOnTime) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerOn = false;
        buzzerPreviousTime = now;
      }

      else if (!buzzerOn && now - buzzerPreviousTime >= buzzerOffTime) {
        buzzerCount++;

        if (buzzerCount >= buzzerBlinkCount) {
          buzzerDone = true;
          digitalWrite(BUZZER_PIN, LOW);
        } else {
          digitalWrite(BUZZER_PIN, HIGH);
          buzzerOn = true;
          buzzerPreviousTime = now;
        }
      }
    }
  }

  Serial.println("FEEDBACK_DONE");
}