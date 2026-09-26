# Multifunctional-4WD-Rover-Using-Arduino-UNO

A complete, end-to-end robotics control system featuring a native **Android Ground Station App** (Bluetooth Classic SPP/RFCOMM) paired with an **Arduino Uno 4WD Autonomous & Remote-Controlled Rover**.

Designed with a high-contrast **Light Technical Aerospace Ground Station** interface, robust fault-tolerant communication pipelines, and real-time telemetry streaming.

---

## 🌟 Key Features

### 🎮 1. Manual Control Mode
- **Tactile D-Pad Interface**: Ergonomic, responsive forward, backward, left, and right directional control buttons.
- **Tactile 500ms Safety Auto-Brake**: Each directional tap drives the rover for exactly 500ms before safely auto-braking, preventing runaway accidents. Rapid tapping extends smooth continuous drive.
- **PWM Speed Regulation Slider**: Continuous motor speed modulation from `0` to `255` PWM (`V:<speed>`), with quick-preset chips for **Slow (80)**, **Normal (120)**, **Fast (180)**, and **Max (255)**.
- **Centralized Emergency Brake**: Instantaneous red stop button executing redundant stop bursts (`S\n`).

### 🛡️ 2. Autonomous Obstacle Avoidance Mode
- **Dual-Control Safety Interlock**: Switching to this mode puts the rover in a safe **STANDBY** state. Autonomous navigation requires an explicit tap on **▶ START MODE (O)**, preventing accidental rover movement on screen transition.
- **SG90 Servo + HC-SR04 Ultrasonic Telemetry**: Sweeps ultrasonic radar forward (90°), left (150°), and right (30°), avoiding obstacles within 15 cm.
- **Dedicated Live Telemetry Screen**: High-contrast instrument LCD viewport streaming real-time distance measurements, sweep angles, and navigation decisions directly onto the phone screen.

### ✏️ 3. Draw-a-Path Navigation Mode
- **Interactive Touch Canvas**: Real-time engineering blueprint drafting board with dashed coordinate grid lines.
- **Differential-Drive Command Generator**: Converts finger-drawn paths into sequential Arduino differential movement commands (`F:<ms>,R:<ms>,F:<ms>,L:<ms>,S\n`).
- **Visual Trajectory Markers**: Start (Emerald green), End (Crimson red), and waypoints (Amber) with live command string preview before transmission.

### ⚡ 4. Robust Bluetooth Classic SPP Engine
- Built specifically for the **HC-05** module using standard RFCOMM SPP UUID (`00001101-0000-1000-8000-00805F9B34FB`) with multi-step socket reflection fallback.
- **Dedicated Single-Thread Send Executor**: Prevents socket write collisions and eliminates packet corruption during rapid tapping.
- **Hardware Disconnect Detection**: Listens to Android OS `ACTION_ACL_DISCONNECTED` broadcasts and triggers a 3-attempt auto-reconnect cycle upon sudden voltage drop or range loss.
- **Collapsible Serial Terminal Monitor**: Expandable bottom monitor displaying incoming (`RX`), outgoing (`TX`), and system status logs with custom newline termination (`LF`, `CRLF`, `NONE`).

---

## 📐 Hardware Architecture & Pinout

### 🔌 Arduino Uno Connections

| Component | Pin Label | Arduino Uno Pin | Function / Description |
|---|---|---|---|
| **HC-05 Bluetooth** | TXD | `D2` (Arduino RX) | SoftwareSerial RX (via voltage divider) |
| **HC-05 Bluetooth** | RXD | `D3` (Arduino TX) | SoftwareSerial TX |
| **L298N Motor Driver** | ENA | `D11` (~PWM) | Left Motors Speed Enable |
| **L298N Motor Driver** | IN1 | `D9` | Left Motors Direction 1 |
| **L298N Motor Driver** | IN2 | `D8` | Left Motors Direction 2 |
| **L298N Motor Driver** | IN3 | `D7` | Right Motors Direction 1 |
| **L298N Motor Driver** | IN4 | `D6` | Right Motors Direction 2 |
| **L298N Motor Driver** | ENB | `D5` (~PWM) | Right Motors Speed Enable |
| **HC-SR04 Ultrasonic** | TRIG | `A3` | Ultrasonic Trigger Output Pulse |
| **HC-SR04 Ultrasonic** | ECHO | `A2` | Ultrasonic Echo Input Pulse |
| **SG90 Micro Servo** | Signal | `A4` | Pan-tilt radar sweep angle control |

> **⚠️ Note on HC-05 Logic Levels**:
> The HC-05 RX pin operates at 3.3V logic. Use a 1kΩ / 2kΩ voltage divider between Arduino `D3` (TX) and HC-05 `RX` to ensure longevity.

---

## 📡 Serial Command Protocol

All commands are transmitted over Bluetooth Classic SPP terminated with `\n` (LF):

| Command | Action | Description |
|---|---|---|
| `F` | Forward | Drives forward for 500ms safety pulse, then auto-brakes |
| `B` | Backward | Drives backward for 500ms safety pulse, then auto-brakes |
| `L` | Turn Left | Spins left for 500ms safety pulse, then auto-brakes |
| `R` | Turn Right | Spins right for 500ms safety pulse, then auto-brakes |
| `S` | Emergency Stop | Halts all motors immediately and resets state |
| `M` | Manual Mode | Sets rover to manual command listening mode |
| `O` | Obstacle Mode | Activates autonomous ultrasonic obstacle avoidance |
| `P` | Path Mode | Prepares Arduino for differential path execution |
| `V:<0-255>` | Set PWM Speed | Sets motor PWM speed (e.g. `V:120`, `V:255`) |
| `F:<ms>,R:<ms>...` | Execute Path | Sequential differential drive instructions |

---

## 🛠️ Software Setup & Installation

### 1. Arduino Firmware
1. Open [`ardunio_code/all_in_one_rover/all_in_one_rover.ino`](ardunio_code/all_in_one_rover/all_in_one_rover.ino) in the Arduino IDE.
2. Install the **Servo** library (built-in) and **SoftwareSerial** library (built-in).
3. Select board **Arduino Uno** and your corresponding COM port.
4. Upload the sketch to the Arduino Uno.
5. Once uploaded, the rover will output `ROVER_READY` over the HC-05 Bluetooth serial.

### 2. Android Controller App
1. Open the project root in **Android Studio** (or build via command line).
2. Build the debug APK using Gradle:
   ```bash
   ./gradlew assembleDebug
   ```
   *(On Windows PowerShell: `.\gradlew.bat assembleDebug`)*
3. The compiled APK will be located at:
   `app/build/outputs/apk/debug/app-debug.apk`
4. Install on your Android smartphone (Android 8.0 Oreo to Android 15 supported):
   ```bash
   adb install -r app/build/outputs/apk/debug/app-debug.apk
   ```
5. Pair your phone with **HC-05** in Android Bluetooth Settings (Default PIN: `1234` or `0000`).
6. Launch the app, select `HC-05` from the top device dropdown, and tap **Connect**.

---

## 📂 Repository Structure

```
├── app/                                 # Android Controller application
│   ├── src/main/
│   │   ├── java/com/example/rover_app/
│   │   │   ├── MainActivity.java        # Bluetooth Classic SPP engine & UI controller
│   │   │   └── PathDrawingView.java     # Interactive touch canvas & path resampling
│   │   ├── res/
│   │   │   ├── drawable/                # Technical styled UI shapes, HUD bezels & icons
│   │   │   ├── layout/
│   │   │   │   ├── activity_main.xml    # 3-mode interface & telemetry monitor layout
│   │   │   │   └── item_spinner.xml     # Custom device picker dropdown
│   │   │   └── values/
│   │   │       ├── colors.xml           # Light Technical Aerospace ground station palette
│   │   │       └── themes.xml           # Light window system bars & styles
│   │   └── AndroidManifest.xml          # Bluetooth & Bluetooth Connect runtime permissions
│   └── build.gradle.kts
├── ardunio_code/
│   └── all_in_one_rover/
│       └── all_in_one_rover.ino         # Arduino Uno firmware sketch
├── .gitignore                           # Excludes Gradle cache, build files, and local properties
├── build.gradle.kts
├── settings.gradle.kts
└── README.md
```

---

## 📜 License
This project is open-source and available under the [MIT License](LICENSE).
