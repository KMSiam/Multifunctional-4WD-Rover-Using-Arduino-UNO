#include <Servo.h>
#include <SoftwareSerial.h>

// ==========================================
// HC-05 Bluetooth Classic Pin Configuration
// ==========================================
const int BT_RX = 2; // Connect to HC-05 TX
const int BT_TX = 3; // Connect to HC-05 RX (through 1k/2k voltage divider recommended)
SoftwareSerial BT(BT_RX, BT_TX);

// ==========================================
// L298N Dual H-Bridge Motor Driver Pins
// ==========================================
const int ENA = 11; // Left Motor Speed (PWM)
const int IN1 = 9;  // Left Motor Direction 1
const int IN2 = 8;  // Left Motor Direction 2
const int IN3 = 7;  // Right Motor Direction 1
const int IN4 = 6;  // Right Motor Direction 2
const int ENB = 5;  // Right Motor Speed (PWM)

// ==========================================
// HC-SR04 Ultrasonic Distance Sensor Pins
// ==========================================
const int TRIG = A3;
const int ECHO = A2;

// ==========================================
// SG90 Servo Pin
// ==========================================
const int SERVO_PIN = A4;
Servo servo;

// ==========================================
// Rover Operating States & Settings
// ==========================================
enum Mode { MANUAL, OBSTACLE, PATH };
Mode mode = MANUAL;

int leftSpeed = 120;
int rightSpeed = 120;

const int SAFE_DISTANCE = 15; // Centimeters threshold

const int CENTER = 90;
const int LEFT_ANGLE = 150;
const int RIGHT_ANGLE = 30;

const unsigned long MANUAL_MOVE_TIME = 500;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  // CRITICAL: Reduce SoftwareSerial timeout from default 1000ms to 30ms for zero lag!
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
}

void loop() {
  // Check for incoming Bluetooth commands
  if (BT.available()) {
    String cmd = BT.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {
      handleCommand(cmd);
    }
  }

  // Execute active mode
  if (mode == OBSTACLE) {
    obstacleMode();
  }
}

// ==========================================
// Non-Blocking Smart Delay Helper
// Checks BT commands while waiting so Emergency
// STOP and Mode Switches respond in < 5ms!
// ==========================================
bool smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (BT.available()) {
      String cmd = BT.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        handleCommand(cmd);
        // If mode changed away from OBSTACLE, abort immediately
        if (mode != OBSTACLE) {
          stopMotors();
          return false;
        }
      }
    }
  }
  return true;
}

// ==========================================
// Bluetooth Command Handler
// ==========================================
void handleCommand(String cmd) {
  // Mode selection
  if (cmd == "M") {
    mode = MANUAL;
    stopMotors();
    servo.write(CENTER);
    BT.println("MODE:MANUAL");
    return;
  }

  if (cmd == "O") {
    mode = OBSTACLE;
    stopMotors();
    servo.write(CENTER);
    BT.println("MODE:OBSTACLE");
    return;
  }

  if (cmd == "P") {
    mode = PATH;
    stopMotors();
    servo.write(CENTER);
    BT.println("MODE:PATH");
    return;
  }

  // CRITICAL FIX: Immediate emergency stop MUST switch mode to MANUAL
  // so that obstacleMode() in loop() does not immediately resume driving!
  if (cmd == "S") {
    mode = MANUAL;
    stopMotors();
    servo.write(CENTER);
    BT.println("STOP");
    BT.println("MODE:MANUAL");
    return;
  }

  // Set speed (V:0 to V:255)
  if (cmd.startsWith("V:")) {
    int speed = cmd.substring(2).toInt();
    if (speed >= 0 && speed <= 255) {
      leftSpeed = speed;
      rightSpeed = speed;
      BT.print("SPEED:");
      BT.println(speed);
    }
    return;
  }

  // Set separate left/right speed (e.g. L:100,R:120)
  if (cmd.startsWith("L:")) {
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      int newLeft = cmd.substring(2, comma).toInt();
      int newRight = cmd.substring(comma + 3).toInt();

      if (newLeft >= 0 && newLeft <= 255 && newRight >= 0 && newRight <= 255) {
        leftSpeed = newLeft;
        rightSpeed = newRight;

        BT.print("SPEED:L");
        BT.print(leftSpeed);
        BT.print(",R");
        BT.println(rightSpeed);
      }
    }
    return;
  }

  // Manual Control Mode (500ms pulses with abortable wait)
  if (mode == MANUAL) {
    if (cmd == "F") {
      forward();
      smartDelay(MANUAL_MOVE_TIME);
      stopMotors();
      BT.println("FORWARD_STOP");
    } else if (cmd == "B") {
      backward();
      smartDelay(MANUAL_MOVE_TIME);
      stopMotors();
      BT.println("BACKWARD_STOP");
    } else if (cmd == "L") {
      turnLeft();
      smartDelay(MANUAL_MOVE_TIME);
      stopMotors();
      BT.println("LEFT_STOP");
    } else if (cmd == "R") {
      turnRight();
      smartDelay(MANUAL_MOVE_TIME);
      stopMotors();
      BT.println("RIGHT_STOP");
    }
    return;
  }

  // Path mode
  if (mode == PATH) {
    executePath(cmd);
  }
}

// ==========================================
// Motor Control Functions
// ==========================================
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

// ==========================================
// Ultrasonic Distance Measurement
// ==========================================
long distanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 25000); // 25ms timeout (~4.2m)

  if (duration == 0)
    return 400;

  return duration * 0.0343 / 2;
}

// ==========================================
// Autonomous Obstacle Avoidance (Non-Blocking)
// ==========================================
void obstacleMode() {
  long dist = distanceCM();
  
  // Stream live distance metric to app screen
  BT.print("DIST:");
  BT.println(dist);

  if (dist > SAFE_DISTANCE) {
    forward();
    if (!smartDelay(100)) return; // Brief move tick, checks BT continually
    return;
  }

  // Obstacle detected!
  stopMotors();
  BT.println("OBSTACLE");

  if (!smartDelay(200)) return;

  // Scan Left
  servo.write(LEFT_ANGLE);
  if (!smartDelay(400)) return;
  long leftDist = distanceCM();

  // Scan Right
  servo.write(RIGHT_ANGLE);
  if (!smartDelay(400)) return;
  long rightDist = distanceCM();

  // Re-center
  servo.write(CENTER);
  if (!smartDelay(150)) return;

  if (leftDist > SAFE_DISTANCE && leftDist > rightDist) {
    BT.println("TURN_LEFT");
    turnLeft();
    if (!smartDelay(600)) return;
  } else if (rightDist > SAFE_DISTANCE) {
    BT.println("TURN_RIGHT");
    turnRight();
    if (!smartDelay(600)) return;
  } else {
    BT.println("BOTH_BLOCKED");
    backward();
    if (!smartDelay(500)) return;
    turnRight();
    if (!smartDelay(900)) return;
  }

  stopMotors();
}

// ==========================================
// Path Execution Engine (Non-Blocking)
// ==========================================
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

  // Non-blocking step delay: abort immediately if emergency stop arrived
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
  }

  stopMotors();
}
