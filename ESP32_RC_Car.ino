# ESP32 WiFi RC Car

Revamped from your **043 – Obstacle-Avoiding Robot with L298N** sketch. Instead
of driving itself around with an ultrasonic sensor, the car is now driven live
from a phone or laptop over WiFi, through a game-style web app the ESP32
hosts itself.

## What you get

- **Clean control** — a drag-anywhere virtual joystick (steering + speed in
  one motion, like a mobile game control stick), plus a big STOP button.
- **Connectivity** — the ESP32 creates its own WiFi hotspot and web server.
  No home router, no app store, no internet needed: connect and open a page.
- **Fail-safe** — if the phone disconnects, locks, or the page is closed, the
  car stops itself automatically within ~0.4 s.
- **Fun modes** (hamburger menu, top right):
  1. **Reverse Mode** — flips every direction: forward↔backward and
     left↔right, all at once. A red banner and a rotated direction ring make
     it obvious the mode is active so nobody's confused mid-demo.
  2. **Rainbow Mode** — the joystick, STOP button, and menu accents cycle
     through soft rainbow colors continuously, purely for show. It never
     touches how the car actually drives.

Both toggles are remembered (saved in the browser) between visits.

## Hardware

ESP32 Dev Board (WROOM-32) + L298N dual motor driver + 2 DC gear motors,
same chassis as your original build. The ultrasonic sensor + servo from the
original project are **not wired up** in this version — see
[Bonus: bring back the sensor](#bonus-bring-back-the-sensor) if you'd like to
keep it as an extra feature.

| L298N pin        | ESP32 GPIO | Purpose                     |
|-------------------|:----------:|------------------------------|
| IN1               | 27         | Left motor direction         |
| IN2               | 26         | Left motor direction         |
| IN3               | 25         | Right motor direction        |
| IN4               | 33         | Right motor direction        |
| ENA               | 14         | Left motor speed (PWM)       |
| ENB               | 32         | Right motor speed (PWM)      |
| GND               | GND        | Common ground — see below    |

These GPIOs were deliberately picked to avoid every ESP32 boot-strapping pin
(0, 2, 5, 12, 15), the flash pins (6–11), and the input-only pins (34–39), so
there's no risk of a boot-mode conflict or bricking the board.

**Important wiring notes:**
- **Remove the ENA and ENB jumpers** on the L298N board. Those jumpers tie
  the enable pins permanently HIGH (always full speed); removing them lets
  the ESP32's PWM signals actually control speed.
- **Common ground**: ESP32 GND, L298N GND, and the motor battery's negative
  terminal must all be tied together, even though the ESP32 is powered
  separately (e.g. USB) from the motors.
- **Logic power**: if your motor supply is 12 V or less, the L298N's onboard
  5 V regulator (jumper in place) can power its own logic — you don't need
  to feed it from the ESP32. If your motor supply is above 12V, remove that
  regulator jumper and give the L298N's 5V logic pin its own 5V source.
- ESP32 GPIOs output 3.3V logic. This reliably registers as HIGH on standard
  L298N breakout boards (a very common pairing) — no level shifter needed.

## Libraries to install

In the Arduino IDE: **Tools > Manage Libraries...**

1. **ESPAsyncWebServer** — search for it, but confirm the listing is the
   **ESP32Async** fork before installing. If it's not indexed on your IDE
   yet, install it manually: download the ZIP from
   https://github.com/ESP32Async/ESPAsyncWebServer and use
   **Sketch > Include Library > Add .ZIP Library...**
2. **AsyncTCP** — same author (ESP32Async). ZIP fallback:
   https://github.com/ESP32Async/AsyncTCP

Both are actively maintained and Arduino-ESP32 core v3-compatible as of 2026;
older forks (e.g. `me-no-dev`, `lacamera`) are outdated and can cause
compile conflicts if installed alongside these — remove them if present.

## Board setup

- **Tools > Board**: "ESP32 Dev Module" (or your specific WROOM-32 board)
- **Arduino-ESP32 core**: v3.x required (Boards Manager > esp32 > update if
  you're on 2.x). The sketch uses the newer `ledcAttach()` / `ledcWrite()`
  PWM API — a one-line swap is noted in a comment in the `.ino` if you're
  stuck on core 2.x.

## Flashing & first run

1. Open `ESP32_RC_Car.ino` in the Arduino IDE. The web app's HTML/CSS/JS is
   embedded directly in this one file (as `PAGE_HTML`) — no second tab or
   extra file needed.
2. Select your board + port, then **Upload**.
3. Open the **Serial Monitor** at 115200 baud. You should see the hotspot IP
   (normally `192.168.4.1`) print out.
4. On your phone or laptop, connect to WiFi network **`ESP32-RC-Car`**
   (password `rccar123` — change both in the `.ino` before your event if
   you'd like something unique).
5. Open a browser to **`http://192.168.4.1`** (or **`http://esp32car.local`**
   on phones/computers that support mDNS/Bonjour — most modern iOS, macOS,
   and Android devices do).
6. Drag the joystick to drive. Tap **STOP** any time for an immediate halt.

## Tuning

- **Top speed** feels off? The PWM duty is derived from the joystick's
  distance from center (0–255). If your motors are very high-torque/fast,
  you can cap it by scaling the computed PWM in `driveSide()` before calling
  `ledcWrite()`.
- **Car turns too sharply/gently**: adjust the mixing formula in
  `driveFromJoystick()` — e.g. multiply the turn term (`x`) by 0.7 before
  mixing to soften steering.
- **Motors run backward from what you'd expect**: swap that side's two
  direction wires at the L298N terminal (easier than touching code).
- **AP password**: change `AP_PASSWORD` in the `.ino` — must be 8+ characters
  for WPA2, or set it to `""` for an open network.

## Bonus: bring back the sensor

Your original build's ultrasonic sensor + servo aren't wired into this
sketch, but they'd make a great extra "impress the judges" feature layered
on top of manual control — e.g. an **auto-brake** that stops forward motion
if something is closer than ~15 cm, without taking over steering. If you'd
like that added, it's a small addition (NewPing + Servo libraries, one
sensor read per loop, and a single check before `driveFromJoystick()` is
allowed to drive forward) — just ask.

## How it works, briefly

- The ESP32 runs both a small web server (serves the one HTML/CSS/JS page,
  embedded as `PAGE_HTML` in the `.ino`) and a WebSocket endpoint (`/ws`)
  for realtime control.
- The web page sends its joystick position as a plain text `"x,y"` string
  (each -1..1) about 16 times a second, whether or not the stick has moved —
  that steady stream doubles as the connection heartbeat the fail-safe
  relies on.
- The ESP32 mixes `x` (turn) and `y` (throttle) into independent left/right
  motor speeds (classic arcade-drive mixing), and drives the L298N directly.
- **Reverse Mode** and **Rainbow Mode** are both handled entirely in the web
  page's JavaScript — Reverse Mode just negates `x` and `y` before they're
  sent, so the ESP32 firmware never needs to know a mode exists; Rainbow
  Mode never talks to the ESP32 at all.
