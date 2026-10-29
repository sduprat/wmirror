# 🪞 wmirror - Smart Orientable Mirror

An open-source low-tech system for automatically orienting a dual-axis mirror to redirect natural sunlight to targeted locations within a room.

---

## 📋 Overview

**wmirror** controls a dual-axis mirror driven by two servo motors. Unlike standard solar trackers that point directly at the sun, `wmirror` calculates the required mirror normal to reflect sunlight toward specific 3D targets in a room (e.g., dining table, desk, living room area) while continuously adjusting as the sun moves across the sky.

### Key Features

| Feature | Description |
|---------|-------------|
| **Smart Sun Reflection** | Calculates bisector normal angles to redirect solar light to room targets |
| **Room Targeting & Presets** | Target specific 3D coordinates (X, Y) or jump to saved room presets |
| **Dual Control Options** | Infrared (IR) remote control and full Serial ASCII interface |
| **Smooth Motion** | Gradual servo step interpolation prevents abrupt mirror movements |
| **3D Web Visualizer** | Interactive web tool to inspect reachability, light paths, and mirror limits |
| **Robust & Tested** | Core geometry tested with C unit tests (CMocka) |

---

## 📦 Hardware & Wiring

### Hardware Requirements
- **Microcontroller**: Arduino Uno (or compatible ATmega328P board)
- **Servos**: 2 × 180° Servo motors (e.g., SG90 or MG996R)
- **Clock Module**: DS3231 or DS1307 Real-Time Clock (I2C RTC)
- **IR Receiver**: VS1838B or compatible 38kHz IR receiver + standard IR remote
- **Mirror**: Lightweight mirror mounted on the vertical servo axis
- **Power Supply**: External 5V DC power supply (minimum 2A recommended for servos)

### Wiring Diagram

```
Arduino Uno
├── Pin 9   ── Servo Vertical (PWM) - Elevation / Pitch
├── Pin 10  ── Servo Horizontal (PWM) - Azimuth / Yaw
├── Pin 7   ── IR Receiver DATA
├── Pin A4  ── RTC SDA (I2C)
├── Pin A5  ── RTC SCL (I2C)
├── 5V      ── RTC, IR Receiver (External 5V supply for Servos recommended)
└── GND     ── Common Ground
```

> ⚠️ **Important**: Always use a separate 5V power supply for the servo motors with a shared ground line to the Arduino.

---

## 🚀 Installation & Setup

### Software Requirements
- [PlatformIO](https://platformio.org/) (Recommended) or [Arduino IDE](https://www.arduino.cc/)

### Option A: PlatformIO
1. Clone the repository:
   ```bash
   git clone https://github.com/username/wmirror.git
   cd wmirror
   ```
2. Build and upload directly:
   ```bash
   pio run -t upload
   ```

### Option B: Arduino IDE
1. Install required libraries via Library Manager:
   - `TimeLib`
   - `SunPosition` (by Andrew Moore)
   - `Servo`
   - `IRremote`
   - `RTClib` (by Adafruit)
2. Open `wmirror.ino` and select **Arduino Uno**.
3. Compile and upload to your board.

---

## ⚙️ Configuration

Set your geographic location and room dimensions in `wmirror.ino`:

```cpp
// Geographic Location (Decimal Degrees)
#define LATITUDE  43.6   // e.g. Toulouse / Lavaur, France
#define LONGITUDE 1.8

// Room Dimensions (Ceiling height in meters)
#define ROOM_CEILING_Z_M 2.5

// Preset Room Targets (X, Y in meters relative to mirror)
#define ROOM_POINT_SALON_X_M           (1.5f)
#define ROOM_POINT_SALON_Y_M          (-0.5f)
#define ROOM_POINT_SALLE_A_MANGER_X_M (3.5f)
#define ROOM_POINT_SALLE_A_MANGER_Y_M  (0.5f)
#define ROOM_POINT_CUISINE_X_M        (6.0f)
#define ROOM_POINT_CUISINE_Y_M        (3.0f)
```

---

## 🎮 User Guide

### 1. Infrared Remote Control

| Button | Action |
|--------|--------|
| **Up (↑)** / **Down (↓)** | Move target along X-axis (±0.20 m) |
| **Left (←)** / **Right (→)** | Move target along Y-axis (±0.20 m) |
| **Home** | Reset servos to default initial position |
| **Play** | Start automatic sun tracking mode |
| **Stop** | Pause automatic tracking mode |
| **1 / 2 / 3** | Quick test positions / sun alignment test |
| **4** | Jump to **Salon** preset |
| **5** | Jump to **Dining Room** preset |
| **6** | Jump to **Kitchen** preset |

### 2. Serial Console Interface (9600 baud)

Connect via Serial Monitor to issue text commands:

#### Target & Movement Commands
- `X+` / `X-` : Nudge room target X coordinate by ±0.20 m
- `Y+` / `Y-` : Nudge room target Y coordinate by ±0.20 m
- `POS x y` : Set target room coordinates directly (e.g. `POS 2.0 -1.0`)
- `A` / `E` : Adjust target azimuth by ±5°
- `Z` / `S` : Adjust target altitude by ±4° / -5°

#### Presets & Operating Modes
- `SALON` / `SALLE` / `CUISINE` : Move mirror target to preset room locations
- `LIST` : Print available preset positions
- `P` : Start auto-tracking mode (`PLAY`)
- `O` : Stop auto-tracking mode (`STOP`)
- `HOME` : Reset to initial position
- `?` : Display current date/time, sun angles, target angles, and calculated mirror normal
- `R` : Redo last command

---

## 🎯 3D Web Visualizer

`wmirror` includes an interactive 3D web tool to inspect physical coverage, visualize beam reflections, and check target reachability.

```bash
cd web3d
python3 -m http.server 8000
```

Open `http://localhost:8000` in your web browser to view:
- 🔵 **Blue Vector**: Sun direction
- 🟡 **Yellow Vector**: Target direction / ceiling points
- 🟢 **Green Vector**: Calculated mirror normal
- **Yellow / Red Points**: Reachable vs unreachable physical ceiling positions

---

## 🛠️ Developers & Architecture Notes

For deep-dive developer documentation, geometric mathematical models, CMocka unit testing, and firmware loop architectures, please consult [DEV_GUIDE.md](DEV_GUIDE.md).

---

## 📜 License

Distributed under the MIT License.
