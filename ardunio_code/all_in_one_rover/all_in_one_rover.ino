#include <Servo.h>
#include <SoftwareSerial.h>

// Bluetooth SoftwareSerial pins
const uint8_t BT_RX = 2;
const uint8_t BT_TX = 3;
SoftwareSerial BT(BT_RX, BT_TX);

// L298N Motor Driver pins
const uint8_t ENA = 11;
const uint8_t IN1 = 9;
const uint8_t IN2 = 8;
const uint8_t IN3 = 7;
const uint8_t IN4 = 6;
const uint8_t ENB = 5;

// HC-SR04 Ultrasonic pins
const uint8_t TRIG = A3;
const uint8_t ECHO = A2;

// SG90 Micro Servo pin
const uint8_t SERVO_PIN = A4;
Servo servo;

// Operating Modes
enum Mode : uint8_t { MANUAL, OBSTACLE, PATH };
Mode mode = MANUAL;

// Motor PWM speeds (0 - 255)
uint8_t leftSpeed = 160;
uint8_t rightSpeed = 160;

// Autonomous navigation thresholds & angles
const uint8_t SAFE_DISTANCE = 15;
const uint8_t CENTER = 90;
const uint8_t LEFT_ANGLE = 150;
const uint8_t RIGHT_ANGLE = 30;

// Non-blocking 500ms safety auto-brake in Manual mode
const unsigned long MANUAL_MOVE_TIME = 500;
unsigned long manualMoveStartTime = 0;
bool isManualMoving = false;
const char* lastManualDirection = "FORWARD";

// Helper: send telemetry string to both Bluetooth and USB Hardware Serial
void sendTelemetry(const __FlashStringHelper* msg) {
  BT.println(msg);
  Serial.println(msg);
}

void sendTelemetry(const char* msg) {
  BT.println(msg);
  Serial.println(msg);
}

void printManualStop() {
  BT.print(lastManualDirection);
  BT.println(F("_STOP"));
  Serial.print(lastManualDirection);
  Serial.println(F("_STOP"));
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  BT.setTimeout(30);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  servo.attach(SERVO_PIN);
  servo.write(CENTER);

  stopMotors();

  sendTelemetry(F("ROVER_READY"));
  sendTelemetry(F("MODE:MANUAL"));
}

void loop() {
  if (BT.available()) {
    String cmd = BT.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
  }

  // Non-blocking 500ms safety auto-brake in Manual mode
  if (mode == MANUAL && isManualMoving) {
    if (millis() - manualMoveStartTime >= MANUAL_MOVE_TIME) {
      stopMotors();
      isManualMoving = false;
      printManualStop();
    }
  }

  if (mode == OBSTACLE) {
    obstacleMode();
  }
}

bool smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (BT.available()) {
      String cmd = BT.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        handleCommand(cmd);
        if (mode != OBSTACLE) {
          stopMotors();
          return false;
        }
      }
    }
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        handleCommand(cmd);
        if (mode != OBSTACLE) {
          stopMotors();
          return false;
        }
      }
    }
  }
  return true;
}

void handleCommand(const String& cmd) {
  if (cmd == "M") {
    mode = MANUAL;
    isManualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendTelemetry(F("MODE:MANUAL"));
    return;
  }

  if (cmd == "O") {
    mode = OBSTACLE;
    isManualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendTelemetry(F("MODE:OBSTACLE"));
    return;
  }

  if (cmd == "P") {
    mode = PATH;
    isManualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendTelemetry(F("MODE:PATH"));
    return;
  }

  if (cmd == "S") {
    mode = MANUAL;
    isManualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendTelemetry(F("STOP"));
    sendTelemetry(F("MODE:MANUAL"));
    return;
  }

  if (cmd.startsWith("V:")) {
    int speed = cmd.substring(2).toInt();
    if (speed >= 0 && speed <= 255) {
      leftSpeed = (uint8_t)speed;
      rightSpeed = (uint8_t)speed;
      if (digitalRead(IN1) != LOW || digitalRead(IN2) != LOW || 
          digitalRead(IN3) != LOW || digitalRead(IN4) != LOW) {
        analogWrite(ENA, leftSpeed);
        analogWrite(ENB, rightSpeed);
      }
      BT.print(F("SPEED:"));
      BT.println(speed);
      Serial.print(F("SPEED:"));
      Serial.println(speed);
    }
    return;
  }

  if (cmd.startsWith("L:")) {
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      int newLeft = cmd.substring(2, comma).toInt();
      int newRight = cmd.substring(comma + 3).toInt();

      if (newLeft >= 0 && newLeft <= 255 && newRight >= 0 && newRight <= 255) {
        leftSpeed = (uint8_t)newLeft;
        rightSpeed = (uint8_t)newRight;
        if (digitalRead(IN1) != LOW || digitalRead(IN2) != LOW || 
            digitalRead(IN3) != LOW || digitalRead(IN4) != LOW) {
          analogWrite(ENA, leftSpeed);
          analogWrite(ENB, rightSpeed);
        }
        BT.print(F("SPEED:L"));
        BT.print(leftSpeed);
        BT.print(F(",R"));
        BT.println(rightSpeed);
        Serial.print(F("SPEED:L"));
        Serial.print(leftSpeed);
        Serial.print(F(",R"));
        Serial.println(rightSpeed);
      }
    }
    return;
  }

  // Manual driving commands (500ms non-blocking safety pulse)
  if (cmd == "F") {
    mode = MANUAL;
    forward();
    manualMoveStartTime = millis();
    isManualMoving = true;
    lastManualDirection = "FORWARD";
    sendTelemetry(F("FORWARD"));
    return;
  }

  if (cmd == "B") {
    mode = MANUAL;
    backward();
    manualMoveStartTime = millis();
    isManualMoving = true;
    lastManualDirection = "BACKWARD";
    sendTelemetry(F("BACKWARD"));
    return;
  }

  if (cmd == "L") {
    mode = MANUAL;
    turnLeft();
    manualMoveStartTime = millis();
    isManualMoving = true;
    lastManualDirection = "LEFT";
    sendTelemetry(F("LEFT"));
    return;
  }

  if (cmd == "R") {
    mode = MANUAL;
    turnRight();
    manualMoveStartTime = millis();
    isManualMoving = true;
    lastManualDirection = "RIGHT";
    sendTelemetry(F("RIGHT"));
    return;
  }

  if (mode == PATH) {
    executePath(cmd);
  }
}

void setMotors(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  bool wasStopped = (digitalRead(IN1) == LOW && digitalRead(IN2) == LOW && 
                     digitalRead(IN3) == LOW && digitalRead(IN4) == LOW);

  digitalWrite(IN1, a);
  digitalWrite(IN2, b);
  digitalWrite(IN3, c);
  digitalWrite(IN4, d);

  // If starting from standstill, provide a quick 35ms kick-pulse to overcome gearbox stiction
  if (wasStopped && (leftSpeed < 180 || rightSpeed < 180)) {
    analogWrite(ENA, (leftSpeed > 210 ? leftSpeed : 210));
    analogWrite(ENB, (rightSpeed > 210 ? rightSpeed : 210));
    delay(35);
  }

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void forward()   { setMotors(HIGH, LOW, HIGH, LOW); }
void backward()  { setMotors(LOW, HIGH, LOW, HIGH); }
void turnLeft()  { setMotors(LOW, HIGH, HIGH, LOW); }
void turnRight() { setMotors(HIGH, LOW, LOW, HIGH); }

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

uint16_t distanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 25000);

  if (duration == 0)
    return 400;

  // Ultra-fast integer division (58.2 us per cm round trip)
  return (uint16_t)(duration / 58);
}

void obstacleMode() {
  uint16_t dist = distanceCM();
  
  BT.print(F("DIST:"));
  BT.println(dist);
  Serial.print(F("DIST:"));
  Serial.println(dist);

  if (dist > SAFE_DISTANCE) {
    forward();
    if (!smartDelay(100)) return;
    return;
  }

  stopMotors();
  sendTelemetry(F("OBSTACLE"));

  if (!smartDelay(200)) return;

  servo.write(LEFT_ANGLE);
  if (!smartDelay(400)) return;
  uint16_t leftDist = distanceCM();

  servo.write(RIGHT_ANGLE);
  if (!smartDelay(400)) return;
  uint16_t rightDist = distanceCM();

  servo.write(CENTER);
  if (!smartDelay(150)) return;

  if (leftDist > SAFE_DISTANCE && leftDist > rightDist) {
    sendTelemetry(F("TURN_LEFT"));
    turnLeft();
    if (!smartDelay(600)) return;
  } else if (rightDist > SAFE_DISTANCE) {
    sendTelemetry(F("TURN_RIGHT"));
    turnRight();
    if (!smartDelay(600)) return;
  } else {
    sendTelemetry(F("BOTH_BLOCKED"));
    backward();
    if (!smartDelay(500)) return;
    turnRight();
    if (!smartDelay(900)) return;
  }

  stopMotors();
}

void executePath(const String& path) {
  int len = path.length();
  int start = 0;

  while (start < len && mode == PATH) {
    int comma = path.indexOf(',', start);
    String command;

    if (comma == -1)
      command = path.substring(start);
    else
      command = path.substring(start, comma);

    command.trim();

    if (command.length() > 0) {
      executeStep(command);
    }

    if (comma == -1 || mode != PATH)
      break;

    start = comma + 1;
  }

  stopMotors();
  if (mode == PATH) {
    sendTelemetry(F("PATH_COMPLETE"));
  }
}

void executeStep(const String& command) {
  if (command == "S") {
    stopMotors();
    return;
  }

  int colon = command.indexOf(':');
  if (colon == -1) return;

  char direction = command.charAt(0);
  long duration = command.substring(colon + 1).toInt();

  if (duration <= 0) return;

  // Real-time telemetry to app
  BT.print(F("PATH_STEP:"));
  BT.println(command);
  Serial.print(F("PATH_STEP:"));
  Serial.println(command);

  switch (direction) {
    case 'F': forward();   break;
    case 'B': backward();  break;
    case 'L': turnLeft();  break;
    case 'R': turnRight(); break;
    default:  return;
  }

  unsigned long start = millis();
  unsigned long lastSonarCheck = 0;

  while (millis() - start < (unsigned long)duration) {
    // Collision avoidance safety guard during forward path movement
    if (direction == 'F' && millis() - lastSonarCheck >= 80) {
      lastSonarCheck = millis();
      uint16_t dist = distanceCM();
      if (dist > 0 && dist <= SAFE_DISTANCE) {
        stopMotors();
        BT.print(F("PATH_OBSTACLE:"));
        BT.println(dist);
        Serial.print(F("PATH_OBSTACLE:"));
        Serial.println(dist);
        mode = MANUAL;
        return;
      }
    }

    if (BT.available()) {
      String cmd = BT.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        handleCommand(cmd);
        if (mode != PATH) {
          stopMotors();
          return;
        }
      }
    }
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        handleCommand(cmd);
        if (mode != PATH) {
          stopMotors();
          return;
        }
      }
    }
  }

  stopMotors();
  delay(120); // 120ms settling pause between steps to eliminate slip & back-EMF
}
