# 🏓 Ping Pong Ball Balancer

> A PID-controlled system that autonomously balances a ping pong ball at the centre of a 420 mm bar using an ESP32-S3, dual VL53L0X time-of-flight sensors, and an MG995 servo motor.

```
[LEFT TOF]  <── dL ──── ● ──── dR ──>  [RIGHT TOF]
                    ball position = L − dR  (primary)

Servo tilts bar ──> ball rolls toward centre (210 mm)
```

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware](#2-hardware)
3. [Hardware Pivot: Stepper → Servo](#3-hardware-pivot-stepper--servo)
4. [Wiring](#4-wiring)
5. [Mechanical Design (CAD)](#5-mechanical-design-cad)
6. [Sensor Reliability & Fusion](#6-sensor-reliability--fusion)
7. [PID Controller](#7-pid-controller)
8. [Hyperparameters](#8-hyperparameters)
9. [PID Tuning Procedure](#9-pid-tuning-procedure)
10. [Arduino IDE Setup](#10-arduino-ide-setup)
11. [Serial Monitor Output](#11-serial-monitor-output)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Project Overview

This project implements a closed-loop PID control system that balances a ping pong ball at the midpoint of a 420 mm aluminium bar. Two VL53L0X time-of-flight sensors measure the ball's position from each end; an ESP32-S3 microcontroller runs the control loop at 30 ms intervals; and an MG995 metal-gear servo physically tilts the bar to drive the ball back to the 210 mm setpoint.

The design went through one major hardware pivot — an initial attempt using a stepper motor was abandoned in favour of the MG995 servo. That journey is documented in [Section 3](#3-hardware-pivot-stepper--servo).

---

## 2. Hardware

| Component | Model / Spec | Notes |
|---|---|---|
| Microcontroller | ESP32-S3 N8R8 | Dual-core, 8 MB flash, 8 MB PSRAM |
| ToF Sensors ×2 | Adafruit VL53L0X | I2C, up to 2 m range; default addr `0x29` |
| Servo Motor | MG995 (metal gear) | 4.8–7.2 V, stall torque 13 kg·cm |
| Bar | 420 mm aluminium channel | Sensing distance between sensors |
| Power Supply | 5 V / 3 A dedicated | For servo only — never use ESP32 rail |

---

## 3. Hardware Pivot: Stepper → Servo

### 3.1 Initial Design — Stepper Motor Approach

The first iteration used a stepper motor for bar actuation, on the premise that steppers offer precise, repeatable positioning without a motor-side feedback sensor. Components used:

| Part | Model |
|---|---|
| Motor | NEMA 17 bipolar stepper — 17HS4401, 1.7 A/phase, 1.8°/step |
| Driver | A4988 stepper driver module (Pololu-style pinout) |
| Encoder | 600 PPR incremental rotary encoder (A/B quadrature) mounted on bar pivot shaft |

The encoder was added to provide absolute angle feedback, since the stepper could lose steps under rapid reversals.

### 3.2 Why It Didn't Work

Several fundamental problems emerged during testing:

- **Resonance and mid-speed torque drop** — The NEMA 17 exhibits a well-known torque null around 200–400 steps/sec, which coincided exactly with the bar oscillation frequency the PID loop was trying to command. The motor would stall or miss steps mid-correction.

- **Latency** — Step-pulse generation within a tight 30 ms PID loop created timing jitter. Achieving smooth motion required interrupt-driven step/dir code that conflicted with the I2C polling of both VL53L0X sensors.

- **Holding current noise** — At standstill, the stepper's holding current injected electrical noise into the 3.3 V rail, causing intermittent I2C bus errors on both sensors.

- **Encoder complexity** — Adding a quadrature encoder for actual angle tracking introduced interrupt-driven counter code and debouncing without resolving the core torque and latency problems.

- **Physical bulk** — The NEMA 17 + A4988 + encoder assembly was mechanically oversized for a lightweight ping pong bar, introducing unwanted rotational inertia at the pivot.

### 3.3 The Pivot — MG995 RC Servo

The system was redesigned around an MG995 metal-gear RC servo, which solved every listed problem:

- **Direct angle command** — A PWM pulse (500–2400 µs) maps directly to a shaft angle. No step counting, no lost-step risk, no encoder required.
- **Software speed limiting** — The `SERVO_SPEED` hyperparameter (max degrees per loop cycle) replaces mechanical damping with smooth, controllable motion.
- **Electrical isolation** — The servo runs from a dedicated 5 V/3 A rail; its signal line on GPIO 3 keeps the I2C bus clean.
- **Compact and lightweight** — At 55 g, the MG995 mounts directly under the bar pivot with a standard servo horn.

> **Trade-off:** The MG995 has a rated positioning accuracy of ~±1° and a dead-band of ~5 µs, which sets a practical lower bound on `DEADBAND_MM` (~25 mm). This is acceptable for the project goals.

---

## 4. Wiring

### 4.1 ToF Sensors — Dual I2C Buses

Both VL53L0X sensors share the default I2C address `0x29` and cannot share a bus without address remapping. Instead, two hardware I2C peripherals on the ESP32-S3 are used — one per sensor — eliminating any need to toggle XSHUT lines. **Leave XSHUT unconnected on both sensors.**

| ESP32-S3 Pin | Left VL53L0X | Right VL53L0X |
|---|---|---|
| GPIO 8 (SDA0) | SDA | — |
| GPIO 9 (SCL0) | SCL | — |
| GPIO 6 (SDA1) | — | SDA |
| GPIO 7 (SCL1) | — | SCL |
| 3.3V | VIN | VIN |
| GND | GND | GND |

### 4.2 MG995 Servo

| ESP32-S3 Pin | MG995 Wire | Notes |
|---|---|---|
| GPIO 3 | Signal (orange/yellow) | PWM via ESP32Servo library |
| GND (shared) | GND (black) | Must share with ESP32 GND |
| External 5V / 3A | VCC (red) | **Never** use ESP32 3.3V rail |

> ⚠️ **Warning:** The MG995 draws up to 2.5 A under stall load. Powering it from the ESP32's 3.3 V rail will brown-out the board. Always use a dedicated 5 V/3 A supply with its GND tied to the ESP32 GND.

---

## 5. Mechanical Design (CAD)

The bar pivots at its midpoint on a 3D-printed mount. The MG995 servo horn connects to a linkage arm that converts rotational motion into bar tilt. VL53L0X sensor brackets at each end hold the modules pointing inward along the bar axis.

<!-- CAD IMAGE 1: Replace the path below with your actual image -->
**Figure 1 — Isometric View **
![CAD Isometric View](CAD/Screenshot%202026-06-11%20112109.png)

**Figure 2 — Exploded View (Servo Mount & Pivot Detail)**
![CAD Exploded View](CAD/Screenshot%202026-06-11%20111944.png)

---

## 6. Sensor Reliability & Fusion

### 6.1 Trusted Ranges

The two sensors have different reliable operating ranges due to reflectivity and angle-of-incidence differences at extreme bar positions:

| Sensor | SDA / SCL | Trusted Range | Role |
|---|---|---|---|
| Left | GPIO 8 / 9 | 0 – 150 mm | Fallback (ball near left end) |
| Right | GPIO 6 / 7 | 0 – 300 mm | Primary |

### 6.2 Fusion Logic

| Condition | Position Used |
|---|---|
| Both sensors valid | Average of both estimates |
| Right only valid | `L − dR` |
| Left only valid | `dL` |
| Neither valid | No ball detected |

> **Blind spot:** There is a region approximately 120–150 mm from the left end where neither sensor is trusted. If the ball is consistently lost there, increase the left sensor threshold from `150` mm to `180` mm inside `fusePosition()` and verify readings remain stable.

---

## 7. PID Controller

### 7.1 What is a PID Controller?

A PID (Proportional-Integral-Derivative) controller is a feedback algorithm that computes a corrective output from three terms, each responding to a different characteristic of the error signal. Here the *error* is the deviation of the ball's measured position from the 210 mm setpoint.

| Term | Role | Effect if too high |
|---|---|---|
| **P** — Proportional | Correction ∝ current error. Large error → aggressive tilt; small error → gentle tilt. | Sustained oscillation around setpoint |
| **I** — Integral | Accumulates error over time, eliminating steady-state offset. | Slow, undamped oscillation that never settles |
| **D** — Derivative | Acts on the *rate of change* of error, braking fast approaches to prevent overshoot. | Noisy, jittery servo motion |

### 7.2 Error Function

```
e(t) = setpoint − position(t)
     = 210 mm − fused_ball_position(t)
```

A positive error (ball left of centre) should tilt the bar so the left side drops, rolling the ball right. A negative error does the opposite. The sign convention is verified during tuning Step 2.

### 7.3 Discrete PID Implementation

The controller runs on a fixed **30 ms** loop period. The continuous equations are discretised as:

```
P_term  = Kp × e[n]

I_term  = I_term + Ki × e[n]          // accumulate each cycle
          (reset if |e[n]| < DEADBAND_MM)  // anti-windup

D_term  = Kd × (e[n] − e[n-1])        // backward difference

PID_out = P_term + I_term + D_term

servo_angle = NEUTRAL_DEG − (int)PID_out  // swap sign if direction is inverted
servo_angle = clamp(servo_angle, SERVO_MIN_DEG, SERVO_MAX_DEG)
servo_angle = rate_limit(servo_angle, SERVO_SPEED)
```

The integral accumulator resets to zero whenever the error falls within `DEADBAND_MM`. This prevents **integrator windup** during the final approach to centre, where mechanical friction and sensor noise would otherwise cause the bar to oscillate with a growing integral component.

---

## 8. Hyperparameters

All tunable values are at the top of the sketch under the `// HYPERPARAMETERS` block.

| Parameter | Default | Description |
|---|---|---|
| `NEUTRAL_DEG` | `97` | Servo angle when bar is physically horizontal. **Set this first** before any gain tuning. |
| `SERVO_MIN_DEG` | `75` | Hard lower clamp on servo angle. |
| `SERVO_MAX_DEG` | `120` | Hard upper clamp on servo angle. |
| `SERVO_SPEED` | `3` | Max degrees the servo moves per 30 ms cycle. Higher = faster, lower = smoother. |
| `Kp` | `0.06` | Proportional gain. |
| `Ki` | `0.0004` | Integral gain. Keep very small (`0.0001`–`0.001`). |
| `Kd` | `0.45` | Derivative gain. Primary damping term. Start at `5 × Kp`. |
| `DEADBAND_MM` | `25` | Error within this window → servo holds neutral, integrator resets. |
| `NO_BALL_SLACK` | `50` | Tolerance for no-ball detection (mm). Raise if ball is ignored mid-bar. |
| `MIN_DIST_MM` | `20` | Readings below this are treated as noise / wall reflections. |

---

## 9. PID Tuning Procedure

Tune gains in strict order. Jumping ahead gives misleading results.

**Step 1 — Set `NEUTRAL_DEG`**
Zero all gains. Adjust `NEUTRAL_DEG` until the bar sits physically horizontal (use a spirit level). This is your mechanical baseline.

**Step 2 — Check correction direction**
Place the ball near one end. The bar should tilt to push the ball toward 210 mm. If it tilts the wrong way, swap the sign in the servo command line:
```cpp
// Change this:
NEUTRAL_DEG - (int)pidOut
// To this:
NEUTRAL_DEG + (int)pidOut
```

**Step 3 — Tune `Kp`** (Ki = 0, Kd = 0)
Increase `Kp` until the ball oscillates steadily around 210 mm. Back off ~30% from that value.

**Step 4 — Tune `Kd`**
Increase `Kd` to damp overshoot. Good starting point: `Kd = 5 × Kp`. Keep raising until oscillations settle without sluggishness.

**Step 5 — Tune `Ki` last**
Only add `Ki` if the ball consistently settles slightly off-centre. Start at `0.0001` and increase slowly. Too much causes a slow oscillation that never damps out.

---

## 10. Arduino IDE Setup

### Board Setup

1. **File → Preferences** → add to *Additional Boards Manager URLs*:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager** → search `esp32` → install **ESP32 by Espressif Systems**
3. **Tools → Board** → select **ESP32S3 Dev Module**
4. **Tools → Port** → select your COM port

### Required Libraries

Install all three via Arduino IDE Library Manager:

| Library | Search Term |
|---|---|
| ESP32Servo | `ESP32Servo` |
| Adafruit VL53L0X | `Adafruit VL53L0X` |
| Wire | Built-in (no install needed) |

---

## 11. Serial Monitor Output

Set baud rate to **115200**. Each line prints:

```
L:183mm   R:112mm   pos:308.0mm   err:-98.0   PID:-4.41   target:101   servo:100
```

| Column | Meaning |
|---|---|
| `L` | Raw left sensor reading (mm) |
| `R` | Raw right sensor reading (mm) |
| `pos` | Fused ball position from left end (mm) |
| `err` | Position error = `210 − pos` |
| `PID` | PID controller output (degrees offset) |
| `target` | Servo angle requested this cycle (post-PID) |
| `servo` | Actual servo angle after speed-limiting clamp |

---

## 12. Troubleshooting

**Servo not moving**
- Verify GND is shared between ESP32 and the external 5V supply.
- Flash a servo debug sketch and confirm pulses appear on GPIO 3 in Serial Monitor.

**Sensor `FAILED` on boot**
- Check VIN is connected to 3.3V, not 5V.
- Verify SDA/SCL pin assignments match the `#define` values in the sketch.
- Try reducing I2C clock: change `400000` → `100000` in `I2C_left.begin()`.

**Ball rolls to one end and stays**
- Correction direction is inverted — swap `+` and `−` in the servo command line.
- `Kp` is too low — raise it until the bar reacts visibly.

**Ball oscillates and won't settle**
- Raise `Kd` to damp overshoot.
- Increase `DEADBAND_MM` to widen the neutral zone.
- Reduce `SERVO_SPEED` for smoother motion.

**Ball gets lost mid-bar**
- Left sensor blind spot (120–150 mm region) — increase left sensor threshold from `150` to `180` mm in `fusePosition()`.
- Check `NO_BALL_SLACK` is large enough; raise from `50` mm if needed.

**Slow oscillation that never damps**
- `Ki` is too large — reduce or set to zero and retune from Step 5.
