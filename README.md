# ESP32 WiFi RC Car

A browser-controlled RC car built on an ESP32 + L298N motor driver. The ESP32
hosts its own WiFi hotspot and a game-style control page — no router, no app,
no internet required. Open a browser, connect, and drive.

---

## Features

- **Virtual joystick** — drag anywhere on the circle to steer and throttle simultaneously
- **Steering Wheel Mode** — rotate an on-screen wheel to steer, hold Gas / Brake pedals (like Dr. Driving)
- **Reverse Mode** — flips all directions at once; a red banner shows when active
- **Rainbow Mode** — cycles the control buttons through colours (cosmetic only)
- **Fail-safe** — motors stop automatically within 400 ms if the browser disconnects or the phone locks
- **Status LED** — built-in LED blinks fast (no client) or slow (client connected)
- **Single-file sketch** — the full web app is embedded in `ESP32_RC_Car.ino`; no second file needed

---

## Hardware

| Component | Detail |
|---|---|
| ESP32 Dev Board | WROOM-32, 38-pin |
| L298N Motor Driver | Standard breakout board |
| DC Gear Motors × 2 | TT motors, 3–6 V |
| Battery | 2× 18650 in series (~7.4 V) |
| Chassis | 2WD acrylic kit with casters |
| Wiring | Dupont male-to-female jumper wires |

### GPIO assignments

| L298N pin | ESP32 GPIO | Purpose |
|---|:---:|---|
| IN1 | 27 | Left motor — forward |
| IN2 | 26 | Left motor — backward |
| IN3 | 25 | Right motor — forward |
| IN4 | 33 | Right motor — backward |
| ENA | 14 | Left motor speed (PWM) |
| ENB | 32 | Right motor speed (PWM) |
| GND | GND | Common ground — required |

> **Before wiring:** remove the ENA and ENB jumpers from the L298N board.
> These jumpers force full speed at all times — removing them lets PWM control work.

> **Common ground:** connect a wire from any ESP32 GND pin to the L298N GND terminal.
> This is the most commonly missed step and causes motors to not respond.

> **Powering the ESP32:** the L298N's onboard 5 V regulator output can power the ESP32
> via its VIN pin when the battery is connected. During programming, use USB as normal.

---

## Libraries

Install both via **Tools > Manage Libraries** in Arduino IDE. The author must be **ESP32Async**:

1. **ESPAsyncWebServer** — [github.com/ESP32Async/ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer)
2. **AsyncTCP** — [github.com/ESP32Async/AsyncTCP](https://github.com/ESP32Async/AsyncTCP)

If they don't appear in search, download the ZIP and install via
**Sketch > Include Library > Add .ZIP Library**.

> Older forks (`me-no-dev`, `lacamera`) are outdated and will cause compile errors
> if installed alongside these. Remove them if present.

---

## Board setup

- **Tools > Board:** ESP32 Arduino > **ESP32 Dev Module**
- **Arduino-ESP32 core:** v3.x required (Boards Manager > esp32 > update if on 2.x)
- **Boards Manager URL:** `https://dl.espressif.com/dl/package_esp32_index.json`

The sketch uses the v3.x LEDC API (`ledcAttach` / `ledcWrite`). A note in the `.ino`
shows the one-line change needed if you are stuck on core 2.x.

---

## Flashing & first run

1. Open `ESP32_RC_Car.ino` in Arduino IDE 2.
2. Select **ESP32 Dev Module** and the correct COM port.
3. Click **Upload**. Hold the BOOT button on some boards if upload times out.
4. Open **Serial Monitor** at **115200 baud** — you should see:
   ```
   ========================================
     ESP32 WiFi RC Car — starting up
   ========================================
   Hotspot SSID : ESP32-RC-Car
   Password     : rccar123
   Hotspot IP   : 192.168.4.1
   ```
5. On your phone, connect to **ESP32-RC-Car** (password: `rccar123`).
6. Open `http://192.168.4.1` in a browser. The connection overlay clears when the car is reachable.
7. Drag the joystick to drive.

---

## Controls

| Control | How |
|---|---|
| Joystick | Drag the circle — angle = turn, distance = speed |
| Arrow keys | Keyboard fallback for desktop testing |
| STOP button | Immediate full stop |
| Reverse Mode | Hamburger menu (top right) → toggle |
| Rainbow Mode | Hamburger menu → toggle |
| Steering Wheel Mode | Hamburger menu → toggle, then rotate wheel + hold pedals |

All three modes are saved in the browser and remembered between visits.

---

## Tuning

- **Speed:** PWM duty runs 0–255 from joystick distance. Scale the value in `driveSide()`
  to cap top speed if motors are too fast.
- **Steering feel:** multiply the `x` turn term in `driveFromJoystick()` by e.g. `0.7`
  to soften turning.
- **Motor runs backward:** swap the two wires at the L298N terminal for that motor.
  Do not change the code for this.
- **WiFi credentials:** change `AP_SSID` and `AP_PASSWORD` in the `.ino`.
  Password must be 8+ characters for WPA2.

---

## How it works

The ESP32 runs a WiFi access point, a web server (serving the embedded HTML/CSS/JS
page), and a WebSocket endpoint at `/ws`. The browser sends joystick position as a
plain text `"x,y"` string (values –1 to 1) roughly 16 times per second. The ESP32
mixes x (turn) and y (throttle) into left/right motor speeds using arcade-drive
math, then drives the L298N with PWM signals.

The steady stream of messages also acts as a heartbeat — if nothing arrives for
400 ms, the fail-safe fires and motors stop.

Reverse Mode and Steering Wheel Mode are handled entirely in the browser's JavaScript.
The ESP32 firmware receives the same `x,y` format regardless of which control mode
is active and does not need to know about modes at all.

---

## Troubleshooting

**Page won't load / connection overlay stays up**
- Confirm phone is on ESP32-RC-Car, not home WiFi
- Try `192.168.4.1` directly instead of `esp32car.local`
- Check Serial Monitor shows "Server started"

**Motors don't move**
- Check common GND wire between ESP32 and L298N
- Confirm ENA and ENB jumpers are removed
- Verify battery is on and voltage is 7 V or above

**Only one motor moves**
- Re-tighten the terminal screw for the non-moving motor
- Check IN3/IN4 and ENB wiring matches the GPIO table above

**Board crashes or reboots**
- Confirm Arduino-ESP32 core is v3.x, not 2.x
- Ensure no boot-strapping pin (0, 2, 5, 12, 15) is driven externally at boot
