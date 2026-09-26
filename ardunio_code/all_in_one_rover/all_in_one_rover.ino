#include <Servo.h>
#include <SoftwareSerial.h>

const int BT_RX = 2;
const int BT_TX = 3;
SoftwareSerial BT(BT_RX, BT_TX);

const int ENA = 11;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENB = 5;

const int TRIG = A3;
const int ECHO = A2;

const int SERVO_PIN = A4;
Servo servo;

enum Mode { MANUAL, OBSTACLE, PATH };
Mode mode = MANUAL;

int leftSpeed = 160;
int rightSpeed = 160;

const int SAFE_DISTANCE = 15;

const int CENTER = 90;
const int LEFT_ANGLE = 150;
const int RIGHT_ANGLE = 30;

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

  BT.println("ROVER_READY");
  BT.println("MODE:MANUAL");
  Serial.println("ROVER_READY");
  Serial.println("MODE:MANUAL");
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

void handleCommand(String cmd) {
  if (cmd == "M") {
    mode = MANUAL;
    stopMotors();
    servo.write(CENTER);
    BT.println("MODE:MANUAL");
    Serial.println("MODE:MANUAL");
    return;
  }

  if (cmd == "O") {
    mode = OBSTACLE;
    stopMotors();
    servo.write(CENTER);
    BT.println("MODE:OBSTACLE");
    Serial.println("MODE:OBSTACLE");
    return;
  }

  if (cmd == "P") {
    mode = PATH;
    stopMotors();
    servo.write(CENTER);
    BT.println("MODE:PATH");
    Serial.println("MODE:PATH");
    return;
  }

  if (cmd == "S") {
    mode = MANUAL;
    stopMotors();
    servo.write(CENTER);
    BT.println("STOP");
    BT.println("MODE:MANUAL");
    Serial.println("STOP");
    Serial.println("MODE:MANUAL");
    return;
  }

  if (cmd.startsWith("V:")) {
    int speed = cmd.substring(2).toInt();
    if (speed >= 0 && speed <= 255) {
      leftSpeed = speed;
      rightSpeed = speed;
      if (digitalRead(IN1) != LOW || digitalRead(IN2) != LOW || 
          digitalRead(IN3) != LOW || digitalRead(IN4) != LOW) {
        analogWrite(ENA, leftSpeed);
        analogWrite(ENB, rightSpeed);
      }
      BT.print("SPEED:");
      BT.println(speed);
      Serial.print("SPEED:");
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
        leftSpeed = newLeft;
        rightSpeed = newRight;
        if (digitalRead(IN1) != LOW || digitalRead(IN2) != LOW || 
            digitalRead(IN3) != LOW || digitalRead(IN4) != LOW) {
          analogWrite(ENA, leftSpeed);
          analogWrite(ENB, rightSpeed);
        }
        BT.print("SPEED:L");
        BT.print(leftSpeed);
        BT.print(",R");
        BT.println(rightSpeed);
        Serial.print("SPEED:L");
        Serial.print(leftSpeed);
        Serial.print(",R");
        Serial.println(rightSpeed);
      }
    }
    return;
  }

  // Manual driving commands (Continuous drive until another direction or STOP is received)
  if (cmd == "F") {
    mode = MANUAL;
    forward();
    BT.println("FORWARD");
    Serial.println("FORWARD");
    return;
  }

  if (cmd == "B") {
    mode = MANUAL;
    backward();
    BT.println("BACKWARD");
    Serial.println("BACKWARD");
    return;
  }

  if (cmd == "L") {
    mode = MANUAL;
    turnLeft();
    BT.println("LEFT");
    Serial.println("LEFT");
    return;
  }

  if (cmd == "R") {
    mode = MANUAL;
    turnRight();
    BT.println("RIGHT");
    Serial.println("RIGHT");
    return;
  }

  if (mode == PATH) {
    executePath(cmd);
  }
}

void setMotors(int a, int b, int c, int d) {
  digitalWrite(IN1, a);
  digitalWrite(IN2, b);
  digitalWrite(IN3, c);
  digitalWrite(IN4, d);

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

long distanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 25000);

  if (duration == 0)
    return 400;

  return duration * 0.0343 / 2;
}

void obstacleMode() {
  long dist = distanceCM();
  
  BT.print("DIST:");
  BT.println(dist);
  Serial.print("DIST:");
  Serial.println(dist);

  if (dist > SAFE_DISTANCE) {
    forward();
    if (!smartDelay(100)) return;
    return;
  }

  stopMotors();
  BT.println("OBSTACLE");
  Serial.println("OBSTACLE");

  if (!smartDelay(200)) return;

  servo.write(LEFT_ANGLE);
  if (!smartDelay(400)) return;
  long leftDist = distanceCM();

  servo.write(RIGHT_ANGLE);
  if (!smartDelay(400)) return;
  long rightDist = distanceCM();

  servo.write(CENTER);
  if (!smartDelay(150)) return;

  if (leftDist > SAFE_DISTANCE && leftDist > rightDist) {
    BT.println("TURN_LEFT");
    Serial.println("TURN_LEFT");
    turnLeft();
    if (!smartDelay(600)) return;
  } else if (rightDist > SAFE_DISTANCE) {
    BT.println("TURN_RIGHT");
    Serial.println("TURN_RIGHT");
    turnRight();
    if (!smartDelay(600)) return;
  } else {
    BT.println("BOTH_BLOCKED");
    Serial.println("BOTH_BLOCKED");
    backward();
    if (!smartDelay(500)) return;
    turnRight();
    if (!smartDelay(900)) return;
  }

  stopMotors();
}

void executePath(String path) {
  int start = 0;

  while (start < path.length() && mode == PATH) {
    int comma = path.indexOf(',', start);
    String command;

    if (comma == -1)
      command = path.substring(start);
    else
      command = path.substring(start, comma);

    command.trim();

    executeStep(command);

    if (comma == -1 || mode != PATH)
      break;

    start = comma + 1;
  }

  stopMotors();
  if (mode == PATH) {
    BT.println("PATH_COMPLETE");
    Serial.println("PATH_COMPLETE");
  }
}

void executeStep(String command) {
  if (command == "S") {
    stopMotors();
    return;
  }

  int colon = command.indexOf(':');
  if (colon == -1) return;

  char direction = command.charAt(0);
  long duration = command.substring(colon + 1).toInt();

  if (duration <= 0) return;

  switch (direction) {
    case 'F': forward();   break;
    case 'B': backward();  break;
    case 'L': turnLeft();  break;
    case 'R': turnRight(); break;
    default:  return;
  }

  unsigned long start = millis();
  while (millis() - start < (unsigned long)duration) {
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
}
