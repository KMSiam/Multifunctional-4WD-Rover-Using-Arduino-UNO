#include <Servo.h>
#include <SoftwareSerial.h>

// ---------- Pins ----------
const byte BT_RX = 2;
const byte BT_TX = 3;
const byte ENA = 11, IN1 = 9, IN2 = 8;
const byte IN3 = 7, IN4 = 6, ENB = 5;
const byte TRIG = A3, ECHO = A2;
const byte SERVO_PIN = A4;

SoftwareSerial BT(BT_RX, BT_TX);
Servo servo;

// ---------- Modes ----------
enum Mode { MANUAL, OBSTACLE, PATH };
Mode mode = MANUAL;

// ---------- Settings ----------
byte leftSpeed = 160;
byte rightSpeed = 160;
const byte SAFE_DISTANCE = 25;
const byte CENTER = 90;
const byte LEFT_ANGLE = 150;
const byte RIGHT_ANGLE = 30;
const unsigned long MANUAL_TIME = 500;

// ---------- Manual timer ----------
unsigned long manualStart = 0;
bool manualMoving = false;
const char *lastDirection = "FORWARD";

// ---------- Input ----------
String btInput = "";
String serialInput = "";
const unsigned int MAX_INPUT = 300;

// ---------- Draw-path queue ----------
struct PathStep {
  char dir;
  unsigned long time;
};

const byte MAX_STEPS = 80;
PathStep steps[MAX_STEPS];
byte stepCount = 0;
byte currentStep = 0;
bool pathStepRunning = false;
unsigned long pathStart = 0;
unsigned long pathDuration = 0;
char pathDirection = 0;
unsigned long lastPathSonar = 0;

// ============================================================
// Communication
// ============================================================

void sendText(const __FlashStringHelper *msg) {
  BT.println(msg);
  Serial.println(msg);
}

void readPort(Stream &port, String &buffer);
void handleCommand(const String &cmd);

// Reads complete commands without blocking with readStringUntil().
void readPort(Stream &port, String &buffer) {
  while (port.available()) {
    char c = port.read();

    if (c == '\n') {
      buffer.trim();
      if (buffer.length()) handleCommand(buffer);
      buffer = "";
    } else if (c != '\r') {
      if (buffer.length() < MAX_INPUT) {
        buffer += c;
      } else {
        buffer = "";  // discard an oversized command
      }
    }
  }
}

// ============================================================
// Motors
// ============================================================

void setMotors(byte a, byte b, byte c, byte d) {
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
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ============================================================
// Ultrasonic
// ============================================================

uint16_t distanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 25000);
  if (duration == 0) return 400;
  return duration / 58;
}

// ============================================================
// Draw Path
// ============================================================

void clearPath() {
  stepCount = 0;
  currentStep = 0;
  pathStepRunning = false;
}

// Add one step. Consecutive equal directions are merged.
void addPathStep(char dir, unsigned long duration) {
  if (duration == 0) return;

  if (stepCount > 0 && steps[stepCount - 1].dir == dir) {
    steps[stepCount - 1].time += duration;
    return;
  }

  if (stepCount >= MAX_STEPS) {
    sendText(F("PATH_QUEUE_FULL"));
    return;
  }

  steps[stepCount].dir = dir;
  steps[stepCount].time = duration;
  stepCount++;
}

// Converts: F:1000,R:500,F:800,S
void addPathCommands(const String &data) {
  int start = 0;

  while (start < data.length()) {
    int comma = data.indexOf(',', start);
    String item = (comma == -1)
                    ? data.substring(start)
                    : data.substring(start, comma);

    item.trim();

    if (item == "S") {
      break;
    }

    int colon = item.indexOf(':');
    if (colon > 0) {
      char dir = item.charAt(0);
      unsigned long duration = item.substring(colon + 1).toInt();

      if ((dir == 'F' || dir == 'B' || dir == 'L' || dir == 'R') &&
          duration > 0) {
        addPathStep(dir, duration);
      }
    }

    if (comma == -1) break;
    start = comma + 1;
  }
}

void startCurrentPathStep() {
  if (currentStep >= stepCount) {
    pathStepRunning = false;
    stopMotors();
    sendText(F("PATH_COMPLETE"));
    return;
  }

  pathDirection = steps[currentStep].dir;
  pathDuration = steps[currentStep].time;
  pathStart = millis();
  lastPathSonar = 0;
  pathStepRunning = true;

  switch (pathDirection) {
    case 'F': forward();   break;
    case 'B': backward();  break;
    case 'L': turnLeft();  break;
    case 'R': turnRight(); break;
  }

  BT.print(F("PATH_STEP:"));
  BT.print(pathDirection);
  BT.print(':');
  BT.println(pathDuration);

  Serial.print(F("PATH_STEP:"));
  Serial.print(pathDirection);
  Serial.print(':');
  Serial.println(pathDuration);
}

void startPath(const String &data) {
  clearPath();
  addPathCommands(data);
  currentStep = 0;
  pathStepRunning = false;

  if (stepCount == 0) {
    sendText(F("PATH_EMPTY"));
    return;
  }

  sendText(F("PATH_START"));
  startCurrentPathStep();
}

void updatePath() {
  if (mode != PATH || stepCount == 0) return;

  // Add more commands while the rover is already moving.
  readPort(BT, btInput);
  readPort(Serial, serialInput);

  if (!pathStepRunning) {
    startCurrentPathStep();
    return;
  }

  // Safety check only during forward movement.
  if (pathDirection == 'F' && millis() - lastPathSonar >= 100) {
    lastPathSonar = millis();
    uint16_t dist = distanceCM();

    if (dist <= SAFE_DISTANCE) {
      stopMotors();
      clearPath();
      mode = MANUAL;

      BT.print(F("PATH_OBSTACLE:"));
      BT.println(dist);
      Serial.print(F("PATH_OBSTACLE:"));
      Serial.println(dist);
      return;
    }
  }

  if (millis() - pathStart >= pathDuration) {
    pathStepRunning = false;
    currentStep++;

    // Do NOT stop here. The next path segment starts immediately.
    if (currentStep < stepCount) {
      startCurrentPathStep();
    } else {
      stopMotors();
      clearPath();
      sendText(F("PATH_COMPLETE"));
    }
  }
}

// ============================================================
// Obstacle avoidance
// ============================================================

bool waitAndListen(unsigned long ms) {
  unsigned long start = millis();

  while (millis() - start < ms) {
    readPort(BT, btInput);
    readPort(Serial, serialInput);

    if (mode != OBSTACLE) {
      stopMotors();
      return false;
    }
  }
  return true;
}

void obstacleMode() {
  uint16_t front = distanceCM();

  BT.print(F("DIST:"));
  BT.println(front);
  Serial.print(F("DIST:"));
  Serial.println(front);

  if (front > SAFE_DISTANCE) {
    forward();
    waitAndListen(100);
    return;
  }

  stopMotors();
  sendText(F("OBSTACLE"));

  servo.write(LEFT_ANGLE);
  if (!waitAndListen(300)) return;
  uint16_t left = distanceCM();

  servo.write(RIGHT_ANGLE);
  if (!waitAndListen(300)) return;
  uint16_t right = distanceCM();

  servo.write(CENTER);
  if (!waitAndListen(100)) return;

  if (left > SAFE_DISTANCE && left >= right) {
    sendText(F("TURN_LEFT"));
    turnLeft();
    waitAndListen(450);
  } else if (right > SAFE_DISTANCE) {
    sendText(F("TURN_RIGHT"));
    turnRight();
    waitAndListen(450);
  } else {
    sendText(F("BOTH_BLOCKED"));
    backward();
    if (!waitAndListen(350)) return;
    turnRight();
    waitAndListen(700);
  }

  stopMotors();
}

// ============================================================
// Commands
// ============================================================

void handleCommand(const String &cmd) {
  // Modes
  if (cmd == "M") {
    mode = MANUAL;
    clearPath();
    manualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendText(F("MODE:MANUAL"));
    return;
  }

  if (cmd == "O") {
    mode = OBSTACLE;
    clearPath();
    manualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendText(F("MODE:OBSTACLE"));
    return;
  }

  if (cmd == "P") {
    mode = PATH;
    clearPath();
    manualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendText(F("MODE:PATH"));
    return;
  }

  if (cmd == "S") {
    mode = MANUAL;
    clearPath();
    manualMoving = false;
    stopMotors();
    servo.write(CENTER);
    sendText(F("STOP"));
    sendText(F("MODE:MANUAL"));
    return;
  }

  // Speed: V:160
  if (cmd.startsWith("V:")) {
    int speed = cmd.substring(2).toInt();
    if (speed >= 0 && speed <= 255) {
      leftSpeed = speed;
      rightSpeed = speed;
      BT.print(F("SPEED:"));
      BT.println(speed);
      Serial.print(F("SPEED:"));
      Serial.println(speed);
    }
    return;
  }

  // Separate left/right speed: L:150,R:170
  // Set this before entering PATH mode.
  if (cmd.startsWith("L:") && cmd.indexOf(",R:") > 0 && mode != PATH) {
    int comma = cmd.indexOf(',');
    int left = cmd.substring(2, comma).toInt();
    int right = cmd.substring(comma + 3).toInt();

    if (left >= 0 && left <= 255 && right >= 0 && right <= 255) {
      leftSpeed = left;
      rightSpeed = right;
      BT.print(F("SPEED:L"));
      BT.print(leftSpeed);
      BT.print(F(",R"));
      BT.println(rightSpeed);
      Serial.print(F("SPEED:L"));
      Serial.print(leftSpeed);
      Serial.print(F(",R"));
      Serial.println(rightSpeed);
    }
    return;
  }

  // Draw path commands. In PATH mode, L:500 means LEFT for 500 ms.
  if (mode == PATH && cmd.length() >= 3) {
    char c = cmd.charAt(0);
    if (c == 'F' || c == 'B' || c == 'L' || c == 'R') {
      if (!pathStepRunning && currentStep == 0 && stepCount == 0) {
        startPath(cmd);
      } else {
        addPathCommands(cmd);
      }
      return;
    }
  }

  // Manual movement
  if (cmd == "F" || cmd == "B" || cmd == "L" || cmd == "R") {
    mode = MANUAL;
    clearPath();

    if (cmd == "F") {
      forward();
      lastDirection = "FORWARD";
      sendText(F("FORWARD"));
    } else if (cmd == "B") {
      backward();
      lastDirection = "BACKWARD";
      sendText(F("BACKWARD"));
    } else if (cmd == "L") {
      turnLeft();
      lastDirection = "LEFT";
      sendText(F("LEFT"));
    } else {
      turnRight();
      lastDirection = "RIGHT";
      sendText(F("RIGHT"));
    }

    manualStart = millis();
    manualMoving = true;
  }
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

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

  sendText(F("ROVER_READY"));
  sendText(F("MODE:MANUAL"));
}

void loop() {
  readPort(BT, btInput);
  readPort(Serial, serialInput);

  // Manual safety stop after 500 ms.
  if (mode == MANUAL && manualMoving &&
      millis() - manualStart >= MANUAL_TIME) {
    stopMotors();
    manualMoving = false;
    BT.print(lastDirection);
    BT.println(F("_STOP"));
    Serial.print(lastDirection);
    Serial.println(F("_STOP"));
  }

  if (mode == OBSTACLE) {
    obstacleMode();
  } else if (mode == PATH) {
    updatePath();
  }
}
