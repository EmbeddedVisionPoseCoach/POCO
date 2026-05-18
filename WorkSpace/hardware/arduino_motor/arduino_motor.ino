#include <Wire.h>
#include <Servo.h>

#define MPU_ADDR 0x68
#define SERVO_PIN 9

Servo cameraServo;

// 제어 상태
bool controlEnabled = false;

// 서보 기본값
int servoCenter = 90;
int servoMin = 45;
int servoMax = 135;

// 보정 강도
float Kp = 1.2;

// 필터값
float filteredRoll = 0;
float alpha = 0.9;

// 기준 기울기
float baselineRoll = 0;

// 센서값
float accX, accY, accZ;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoCenter);

  // MPU6050 깨우기
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(1000);

  // 처음 전원 켰을 때 기준값 저장
  baselineRoll = readRollAngle();
  filteredRoll = baselineRoll;

  Serial.println("Arduino Ready");
  Serial.println("Waiting for START command...");
}

void loop() {
  checkSerialCommand();

  if (controlEnabled) {
    controlTiltServo();
  }

  delay(30);
}

void checkSerialCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START") {
      baselineRoll = readRollAngle();
      filteredRoll = baselineRoll;

      controlEnabled = true;

      Serial.println("CONTROL START");
      Serial.print("Baseline Roll: ");
      Serial.println(baselineRoll);
    }

    else if (cmd == "STOP") {
      controlEnabled = false;
      cameraServo.write(servoCenter);

      Serial.println("CONTROL STOP");
    }

    else if (cmd == "CENTER") {
      cameraServo.write(servoCenter);

      Serial.println("SERVO CENTER");
    }
  }
}

void controlTiltServo() {
  float currentRoll = readRollAngle();

  // EMA 필터 적용
  filteredRoll = alpha * filteredRoll + (1 - alpha) * currentRoll;

  // 기준 자세 대비 기울어진 정도
  float rollError = filteredRoll - baselineRoll;

  // 반대 방향으로 서보 보정
  int servoAngle = servoCenter - (rollError * Kp);

  // 서보 각도 제한
  servoAngle = constrain(servoAngle, servoMin, servoMax);

  cameraServo.write(servoAngle);

  Serial.print("Roll: ");
  Serial.print(filteredRoll);
  Serial.print(" | Error: ");
  Serial.print(rollError);
  Serial.print(" | Servo: ");
  Serial.println(servoAngle);
}

float readRollAngle() {
  int16_t rawX, rawY, rawZ;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  rawX = Wire.read() << 8 | Wire.read();
  rawY = Wire.read() << 8 | Wire.read();
  rawZ = Wire.read() << 8 | Wire.read();

  accX = rawX / 16384.0;
  accY = rawY / 16384.0;
  accZ = rawZ / 16384.0;

  // 좌우 기울기 Roll 계산
  float roll = atan2(accY, accZ) * 180.0 / PI;

  return roll;
}