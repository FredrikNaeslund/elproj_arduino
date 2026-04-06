// Robot car controller for ESP32
// Each motor has one pin for forward and one for backward.
// Setting a pin HIGH drives that direction; both LOW = stop.
//
// Pin assignments:
#define LEFT_FWD  2
#define LEFT_BWD  3
#define RIGHT_FWD 4
#define RIGHT_BWD 5

void moveForward() {
  digitalWrite(LEFT_FWD,  HIGH);
  digitalWrite(LEFT_BWD,  LOW);
  digitalWrite(RIGHT_FWD, HIGH);
  digitalWrite(RIGHT_BWD, LOW);
}

void moveBackward() {
  digitalWrite(LEFT_FWD,  LOW);
  digitalWrite(LEFT_BWD,  HIGH);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_BWD, HIGH);
}

void turnLeft() {
  digitalWrite(LEFT_FWD,  LOW);
  digitalWrite(LEFT_BWD,  HIGH);
  digitalWrite(RIGHT_FWD, HIGH);
  digitalWrite(RIGHT_BWD, LOW);
}

void turnRight() {
  digitalWrite(LEFT_FWD,  HIGH);
  digitalWrite(LEFT_BWD,  LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_BWD, HIGH);
}

void stopAll() {
  digitalWrite(LEFT_FWD,  LOW);
  digitalWrite(LEFT_BWD,  LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_BWD, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_FWD,  OUTPUT);
  pinMode(LEFT_BWD,  OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  stopAll();
  Serial.println("Ready. Commands: w=forward s=backward a=left d=right space=stop");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'w': moveForward();  Serial.println("Forward");  break;
      case 's': moveBackward(); Serial.println("Backward"); break;
      case 'a': turnLeft();     Serial.println("Left");     break;
      case 'd': turnRight();    Serial.println("Right");    break;
      case ' ': stopAll();      Serial.println("Stop");     break;
    }
  }
}
