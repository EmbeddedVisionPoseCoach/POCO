const int VIB_PIN = 9;

void setup() {
  pinMode(VIB_PIN, OUTPUT);
  digitalWrite(VIB_PIN, LOW);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "VIB_ON") {
      digitalWrite(VIB_PIN, HIGH);
      Serial.println("VIBRATION ON");
    }
    else if (command == "VIB_OFF") {
      digitalWrite(VIB_PIN, LOW);
      Serial.println("VIBRATION OFF");
    }
  }
}