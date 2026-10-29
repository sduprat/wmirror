# wmirror — Developer & Validation Guide

This document provides developer notes for the `wmirror` project: geometric logic, CMocka unit testing, serial debugging commands, and the 3D web visualizer.

---

## 1. Project Architecture & Objectives

The goal of `wmirror` is to orient a dual-axis mirror to reflect sunlight toward a target point in a room. The system calculates a mirror normal vector as the angle bisector between the sun direction vector (incident) and the target vector (reflected).

The codebase is structured into clear modules:
- **`wmirror_logic.c` / `wmirror_logic.h`**: Core C geometry routines, angle normalization, coordinate transformations (spherical $\leftrightarrow$ Cartesian room space), and mirror normal calculation.
- **`wmirror.ino`**: Arduino firmware managing real-time loops, RTC time syncing, IR remote decoding, serial commands, and servo hardware outputs.
- **`tests/`**: Unit tests written in C using the CMocka framework (`test_basic.c`, `test_wmirror_logic.c`, `test_limits_bounds.c`).
- **`web3d/`**: Browser-based Three.js 3D tool for visually verifying reachable ceiling/room zones without hardware.

---

## 2. Geometric Logic & Servo Mapping

### 2.1 Mirror Normal Calculation
The reflection normal $\vec{N}$ is derived from:
- Sun position: Azimuth $\text{Az}_{\text{sun}}$ and Altitude $\text{Alt}_{\text{sun}}$
- Room target position: Azimuth $\text{Az}_{\text{target}}$ and Altitude $\text{Alt}_{\text{target}}$

The unit vectors for incident sunlight $\vec{V}_{\text{sun}}$ and target beam $\vec{V}_{\text{target}}$ are summed to obtain the bisector vector $\vec{N} = \frac{\vec{V}_{\text{sun}} + \vec{V}_{\text{target}}}{\|\vec{V}_{\text{sun}} + \vec{V}_{\text{target}}\|}$, which is converted back to spherical coordinates (`norm_azimuth`, `norm_altitude`).

### 2.2 Mapping Normal to Servos
The mechanical servo angles are mapped as follows:
- **Horizontal Servo (Azimuth / Yaw)**: $H = 270^\circ - \text{norm\_azimuth}$
- **Vertical Servo (Altitude / Pitch)**: $V = 90^\circ - \text{norm\_altitude}$

Mechanical Clamping Limits:
- $H \in [0^\circ, 180^\circ]$
- $V \in [0^\circ, 90^\circ]$

### 2.3 Reachability & Boundary Rules
When determining if a room target is reachable:
- Raw un-clamped servo values must be evaluated against physical limits ($[0^\circ, 180^\circ]$ and $[0^\circ, 90^\circ]$).
- A point is marked **reachable** only if the raw computed servo position falls strictly within mechanical bounds. Clamping a saturated position (e.g. forcing $195^\circ \to 180^\circ$) produces an invalid physical reflection angle and is marked **unreachable**.

---

## 3. Firmware Execution Loops

The firmware runs two distinct execution loops in `wmirror.ino`:

1. **Slow Tracking Loop (~30 seconds)**:
   - Queries RTC for current timestamp.
   - Computes solar position ($\text{Az}_{\text{sun}}, \text{Alt}_{\text{sun}}$) using `SunPosition`.
   - Recalculates mirror normal and desired target servo positions.

2. **Fast Servo Loop (~30 ms)**:
   - Interpolates servo positions step-by-step (`SERVO_STEP_H`, `SERVO_STEP_V`).
   - Ensures smooth mechanical movement without sudden jumps or mechanical chatter.

---

## 4. Building & Running Unit Tests (CMocka)

Core geometry logic in `wmirror_logic.c` is tested using CMocka unit tests in `tests/`.

### 4.1 Prerequisites
On Debian/Ubuntu-based Linux systems:
```bash
sudo apt-get install -y libcmocka-dev gcc make
```

### 4.2 Building and Running Tests
At the root of the repository:
```bash
# Build and execute unit tests
make test

# Build and validate all targets
make all

# Clean build artifacts
make clean
```

### 4.3 Covered Test Cases
- Angle normalization across $360^\circ$ and negative values
- Servo angle clamping and boundary checks
- Reflection normal vector calculation accuracy
- Spherical to Cartesian room coordinate conversions
- Edge cases around $0^\circ / 360^\circ$ discontinuities

---

## 5. 3D Web Visualizer

Located in `web3d/`, this tool simulates the 3D room environment and visualizes light vectors without physical hardware.

### 5.1 Running the Visualizer
```bash
cd web3d
python3 -m http.server 8000
```
Navigate to `http://localhost:8000` in your web browser.

### 5.2 Visualizer Components
- **Floor Grid**: Scan area of room targets.
- **Yellow Points**: Reachable target points on ceiling grid.
- **Red Points**: Unreachable target points exceeding servo limits.
- **Vectors**:
  - 🔵 Blue: Direction to sun
  - 🟡 Yellow: Direction to room target
  - 🟢 Green: Calculated mirror normal vector

---

## 6. Key Project Files

| File | Purpose |
|------|---------|
| `wmirror.ino` | Arduino main firmware entry point |
| `wmirror_logic.c` / `.h` | Geometry and vector calculation library |
| `Makefile` | Build system for CMocka unit tests |
| `tests/test_basic.c` | Basic boundary test suite |
| `tests/test_wmirror_logic.c` | Core geometry test suite |
| `tests/test_limits_bounds.c` | Edge cases and servo limits test suite |
| `web3d/index.html` | 3D visualizer web page |
| `web3d/logic.js` | JavaScript implementation of mirror geometry |
| `web3d/app.js` | Three.js rendering and UI controls |
