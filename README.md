# Ping Pong Ball Balancer

A PID-controlled system that balances a ping pong ball at the centre of a 420mm bar using an ESP32-S3, two VL53L0X time-of-flight sensors, and an MG995 servo motor.

---

## How It Works

Two ToF sensors are mounted at each end of the bar, pointing inward. They measure the distance to the ball. The PID loop computes the error between the ball's position and the centre (210mm), and tilts the bar via the servo to push the ball back to centre.

```
[LEFT TOF] ←── dL ──── ● ──── dR ──→ [RIGHT TOF]
                   ball position = L - dR  (primary)
```

---

## Hardware

| Component | Model |
|---|---|
| Microcontroller | ESP32-S3 N8R8 |
| ToF Sensors (×2) | Adafruit VL53L0X |
| Servo | MG995 |
| Bar length | 420mm (sensing distance between sensors) |

---

## Wiring

### ToF Sensors — Two separate I2C buses (no XSHUT needed)

| ESP32-S3 | Left VL53L0X | Right VL53L0X |
|---|---|---|
| GPIO 8 (SDA) | SDA | — |
| GPIO 9 (SCL) | SCL | — |
| GPIO 6 (SDA) | — | SDA |
| GPIO 7 (SCL) | — | SCL |
| 3.3V | VIN | VIN |
| GND | GND | GND |

> Both sensors keep their default I2C address `0x29` — they are on physically separate buses so there is no conflict. Leave XSHUT unconnected on both.

### Servo MG995

| ESP32-S3 | MG995 |
|---|---|
| GPIO 3 | Signal (orange/yellow) |
| GND | GND (black) — must share with ESP32 GND |
| External 5V | VCC (red) |

> **Never power the MG995 from the ESP32 3.3V rail.** It draws up to 2.5A under load and will brown-out the board. Use a dedicated 5V/3A supply with its GND tied to ESP32 GND.

---

## Sensor Reliability

The two sensors have different reliable ranges:

| Sensor | GPIO (SDA/SCL) | Trusted range | Role |
|---|---|---|---|
| Left  | GPIO 8 / 9 | 0 – 150mm | Fallback only (ball near left end) |
| Right | GPIO 6 / 7 | 0 – 300mm | Primary |

**Fusion logic:**

| Condition | Position used |
|---|---|
| Both sensors valid | Average of both estimates |
| Right only valid | `L − dR` |
| Left only valid | `dL` |
| Neither valid | No ball detected |

> There is a blind spot approximately 120–150mm from the left end where neither sensor is trusted. If the ball consistently gets lost there, increase the left sensor threshold from 150mm to 180mm and check if readings are still stable.

## Libraries


| Library | Search term |
| ESP32Servo | `ESP32Servo` |
| Wire | Built-in |


   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
4. Tools → Port → select your COM port

---
## Hyperparameters


| Parameter | Default | Description |
| `SERVO_MIN_DEG` | 75 | Hard lower bound on servo angle |
| `SERVO_MAX_DEG` | 120 | Hard upper bound on servo angle |
| `NO_BALL_SLACK` | 50mm | Tolerance for no-ball detection. Raise if ball gets ignored |
| `DEADBAND_MM` | 25mm | Error within this range → servo holds neutral, integrator resets |
| `Kp` | 0.06 | Proportional gain |
| `Ki` | 0.0004 | Integral gain |

---

## PID Tuning — Step by Step

**Step 1 — Set `NEUTRAL_DEG`**
Set all gains to 0. Adjust `NEUTRAL_DEG` until the bar sits physically horizontal.

**Step 2 — Check direction**
Place ball near one end. Bar should tilt to push ball toward centre (210mm).
If it tilts the wrong way, change in the code:
```cpp
NEUTRAL_DEG - (int)pidOut   →   NEUTRAL_DEG + (int)pidOut
```

**Step 3 — Tune `Kp`** (Ki=0, Kd=0)
Raise until ball oscillates steadily around 210mm. Back off ~30%.

**Step 4 — Tune `Kd`**
Raise to damp overshoot. A good starting point is `Kd = 5 × Kp`.

**Step 5 — Tune `Ki` last**
Only add if ball settles slightly off-centre. Keep it very small (0.0001–0.001).
Too much causes a slow oscillation that never damps out.

---

## Serial Monitor Output

Set baud rate to **115200**. Each line prints:

```
L:183mm   R:112mm   pos:308.0mm   err:-98.0   PID:-4.41   target:101   servo:100
```

| Column | Meaning |
|---|---|
| `L` | Raw left sensor reading (mm) |
| `R` | Raw right sensor reading (mm) |
| `pos` | Fused ball position from left end (mm) |
| `err` | Position error = 210 − pos |
| `PID` | PID controller output (degrees offset) |
| `target` | Servo angle requested this cycle |
| `servo` | Actual servo angle after speed limiting |

---

## Troubleshooting

**Servo not moving**
- Check GND is shared between ESP32 and external 5V supply
- Flash the servo debug sketch and check Serial Monitor output

**Sensor FAILED on boot**
- Check VIN → 3.3V (not 5V)
- Verify SDA/SCL pin connections match the defines in the sketch
- Try reducing I2C clock: change `400000` to `100000` in `I2C_left.begin()`

**Ball rolls to one end and stays**
- Direction is inverted — swap `+` to `−` (or vice versa) in the servo command line
- Or `Kp` is too low — raise it

**Ball oscillates and won't settle**
- Raise `Kd`
- Raise `DEADBAND_MM`
- Lower `SERVO_SPEED`

**Ball gets lost mid-bar**
- Left sensor blind spot — increase left threshold from 150mm to 180mm in `fusePosition()`| `Kd` | 0.45 | Derivative gain |
| `MIN_DIST_MM` | 20mm | Readings below this are treated as noise |
| `NEUTRAL_DEG` | 97 | Servo angle when bar is level — adjust until bar is physically flat |
| `SERVO_SPEED` | 3 | Max degrees servo moves per loop cycle (30ms). Higher = faster, lower = smoother |
|---|---|---|
All tunable values are at the top of the sketch under the `HYPERPARAMETERS` block.

3. Select: Tools → Board → **ESP32S3 Dev Module**
2. Tools → Board → Boards Manager → search `esp32` → install **ESP32 by Espressif Systems**
   ```
1. File → Preferences → add to *Additional Boards Manager URLs*:
## Board Setup (Arduino IDE)
---

| Adafruit VL53L0X | `Adafruit VL53L0X` |
|---|---|
Install all three via Arduino IDE Library Manager:
---

