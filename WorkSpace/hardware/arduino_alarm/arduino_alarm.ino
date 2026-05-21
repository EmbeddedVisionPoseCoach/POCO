const int BUZZER_PIN = 8;
// 부저 핀 설정
const int LED_PIN = 7;

String command = "";
// 받은 문자열 저장

void setup()
{
    pinMode(LED_PIN, OUTPUT);

    pinMode(BUZZER_PIN, OUTPUT);
    // 8번핀 출력으로 사용

    digitalWrite(BUZZER_PIN, LOW);
    // 부저 꺼진 상태로 set

    Serial.begin(9600);
    // 시리얼 통신 시작

    Serial.println("Buzzer Ready");
}

void loop()
{
    if (Serial.available())
    // 시리얼 데이터 들어왔는지 확인
    {
        command = Serial.readStringUntil('\n');
        // \n 전까지 읽기

        command.trim();
        // 앞뒤 공백 제거

        Serial.print("받은 명령: ");
        Serial.println(command);
        // 받은 명령 출력

        if (command == "BUZZ_ON")
        // 부저 ON
        {
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(LED_PIN, HIGH); // LED 켜기
            delay(500);                  // 0.5초 대기

            digitalWrite(LED_PIN, LOW);  // LED 끄기
            delay(500);

            Serial.println("부저 ON");
        }

        else if (command == "BUZZ_OFF")
        // 부저 OFF
        {
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(LED_PIN, LOW);  // LED 끄기

            Serial.println("부저 OFF");
        }
    }
}


