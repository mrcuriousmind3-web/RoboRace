/* =====================================================================
   ESP32 WiFi RC Car  —  v2  (4-wheel drive, dual L298N)
   =====================================================================

   HARDWARE OVERVIEW
   -----------------
   ESP32 dev board (WROOM-32 / "ISM24G" label = same chip, 2.4GHz band)
   2× L298N dual H-bridge motor driver boards
   4× 12W 200RPM DC gear motors
   3× 18650 Li-Ion in SERIES → ~11.1V nominal
   2× LM2596 buck converter (step 11.1V → 5V each)
   1× 25W bulk capacitor across the 11.1V battery rail

   POWER ARCHITECTURE
   ------------------

     [3S 18650 pack: ~11.1V]
           │
           ├──► [25W cap across rail]  ← absorbs motor inrush, stops ESP32 brownouts
           │
           ├──► [LM2596 #1 → 5V] ──► ESP32 VIN pin
           │
           ├──► [L298N #1: 12V terminal]  ← Left motors power
           │         Motor A → Front-Left motor (both wires in parallel)
           │         Motor B → Rear-Left  motor (both wires in parallel)
           │
           └──► [L298N #2: 12V terminal]  ← Right motors power
                     Motor A → Front-Right motor
                     Motor B → Rear-Right  motor

   ALL GROUNDS MUST SHARE A COMMON RAIL:
     Battery (–) → L298N #1 GND → L298N #2 GND → ESP32 GND

   NOTE: The L298N's onboard 5V regulator is NOT used here because
   the 11.1V pack exceeds its reliable input range under load.
   The LM2596 is a proper switching regulator — use it instead.

   MOTOR WIRING  (parallel pairs per channel for 4WD)
   ---------------------------------------------------
   Each L298N channel drives two motors wired in PARALLEL.
   Parallel wiring doubles current draw but keeps voltage the same,
   so the L298N's 2A peak per channel is shared by both motors.
   For 12W motors at 11.1V → each draws ~1.08A → pair = ~2.16A.
   This is right at the L298N limit. If the driver gets very hot,
   add heatsinks (stick-on aluminium) to both L298N chips.

   L298N #1 — LEFT SIDE
   ┌──────────────────────────────────────────────────────────┐
   │  ENA  ← GPIO 16  (left speed PWM)   — remove ENA jumper │
   │  IN1  ← GPIO 17  (left forward)                         │
   │  IN2  ← GPIO 5   (left backward)                        │
   │  IN3  ← GPIO 18  (left forward — B channel mirrors A)   │
   │  IN4  ← GPIO 19  (left backward — B channel mirrors A)  │
   │  ENB  ← GPIO 21  (left speed PWM)   — remove ENB jumper │
   │  Motor A → Front-Left motor                             │
   │  Motor B → Rear-Left  motor                             │
   └──────────────────────────────────────────────────────────┘

   L298N #2 — RIGHT SIDE
   ┌──────────────────────────────────────────────────────────┐
   │  ENA  ← GPIO 22  (right speed PWM)  — remove ENA jumper │
   │  IN1  ← GPIO 23  (right forward)                        │
   │  IN2  ← GPIO 25  (right backward)                       │
   │  IN3  ← GPIO 26  (right forward — B channel mirrors A)  │
   │  IN4  ← GPIO 27  (right backward — B channel mirrors A) │
   │  ENB  ← GPIO 33  (right speed PWM)  — remove ENB jumper │
   │  Motor A → Front-Right motor                            │
   │  Motor B → Rear-Right  motor                            │
   └──────────────────────────────────────────────────────────┘

   GPIO SAFETY NOTES (ESP32 WROOM-32)
   ------------------------------------
   Avoided GPIO 12  → strapping pin (MTDI): HIGH at boot bricks flash
   Avoided GPIO 0   → strapping pin: must float/HIGH at boot
   Avoided GPIO 2   → strapping pin: must be LOW or float at boot
   Avoided GPIO 15  → strapping pin: affects SDIO slave / boot log
   Avoided GPIO 6–11→ connected to internal flash SPI — NEVER USE
   Avoided GPIO 34–39 → input-only, no internal pull, unusable as output
   GPIO 2 is used for the status LED (built-in on most dev boards);
   it is only driven AFTER boot is complete, which is safe.

   LIBRARIES  (Tools > Manage Libraries in Arduino IDE 2)
   -------------------------------------------------------
   1. ESPAsyncWebServer  — author: ESP32Async
      https://github.com/ESP32Async/ESPAsyncWebServer
   2. AsyncTCP           — author: ESP32Async
      https://github.com/ESP32Async/AsyncTCP

   BOARD SETTINGS
   --------------
   Tools > Board   : ESP32 Dev Module
   Core version    : arduino-esp32 v3.x  (uses ledcAttach/ledcWrite API)
   Partition scheme: Default 4MB with spiffs (or any 4MB scheme)
   ===================================================================== */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ── WiFi hotspot ──────────────────────────────────────────────────────
const char *AP_SSID     = "ESP32-RC-Car";
const char *AP_PASSWORD = "rccar123";      // min 8 chars (WPA2)
const char *HOSTNAME    = "esp32car";      // http://esp32car.local

// ── L298N #1 — LEFT SIDE ─────────────────────────────────────────────
#define L_ENA   16   // Front-Left  speed  (PWM)
#define L_IN1   17   // Front-Left  forward
#define L_IN2    5   // Front-Left  backward
#define L_ENB   21   // Rear-Left   speed  (PWM)
#define L_IN3   18   // Rear-Left   forward
#define L_IN4   19   // Rear-Left   backward

// ── L298N #2 — RIGHT SIDE ────────────────────────────────────────────
#define R_ENA   22   // Front-Right speed  (PWM)
#define R_IN1   23   // Front-Right forward
#define R_IN2   25   // Front-Right backward
#define R_ENB   33   // Rear-Right  speed  (PWM)
#define R_IN3   26   // Rear-Right  forward
#define R_IN4   27   // Rear-Right  backward

// ── Status LED ───────────────────────────────────────────────────────
// GPIO 2 is the built-in LED on most ESP32 dev boards.
// It is only driven after boot — safe despite being a strapping pin.
#define LED_PIN  2

// ── PWM settings ─────────────────────────────────────────────────────
const int PWM_FREQ_HZ  = 5000;   // above audible whine range
const int PWM_RES_BITS = 8;      // 0–255 duty

// ── Fail-safe ────────────────────────────────────────────────────────
// Motors stop if no command received for this many milliseconds.
// The web page sends at ~16 Hz so a WiFi drop is caught quickly.
const unsigned long FAILSAFE_MS = 400;
unsigned long lastCommandAt = 0;
bool motorsStopped = true;

// ── WebSocket / server ───────────────────────────────────────────────
int clientCount = 0;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── LED blink ────────────────────────────────────────────────────────
unsigned long lastLedToggle = 0;
bool ledState = false;

// =====================================================================
//  LOW-LEVEL MOTOR DRIVE
//  speed: -255 (full reverse) … 255 (full forward)
// =====================================================================
void driveMotor(int pinFwd, int pinBwd, int enPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinFwd, HIGH);
    digitalWrite(pinBwd, LOW);
  } else if (speed < 0) {
    digitalWrite(pinFwd, LOW);
    digitalWrite(pinBwd, HIGH);
  } else {
    digitalWrite(pinFwd, LOW);
    digitalWrite(pinBwd, LOW);
  }
  ledcWrite(enPin, abs(speed));
}

// Drive both channels of one L298N at the same speed (parallel pair)
void driveSide(int in1, int in2, int ena, int in3, int in4, int enb, int speed) {
  driveMotor(in1, in2, ena, speed);
  driveMotor(in3, in4, enb, speed);
}

void stopAll() {
  driveSide(L_IN1, L_IN2, L_ENA, L_IN3, L_IN4, L_ENB, 0);
  driveSide(R_IN1, R_IN2, R_ENA, R_IN3, R_IN4, R_ENB, 0);
}

// =====================================================================
//  ARCADE-DRIVE MIXING
//  x: turn  (-1 = full left … 1 = full right)
//  y: throttle (-1 = full reverse … 1 = full forward)
//
//  Left wheels  = y + x
//  Right wheels = y - x
//  Both terms are normalised so neither exceeds ±1.
//  Dead-zone of ±4% filters joystick centering noise.
// =====================================================================
void driveFromJoystick(float x, float y) {
  x = constrain(x, -1.0f, 1.0f);
  y = constrain(y, -1.0f, 1.0f);

  const float DEAD = 0.04f;
  if (fabsf(x) < DEAD) x = 0.0f;
  if (fabsf(y) < DEAD) y = 0.0f;

  float left  = y + x;
  float right = y - x;

  float peak = max(fabsf(left), fabsf(right));
  if (peak > 1.0f) { left /= peak; right /= peak; }

  int leftPWM  = (int)(left  * 255.0f);
  int rightPWM = (int)(right * 255.0f);

  driveSide(L_IN1, L_IN2, L_ENA, L_IN3, L_IN4, L_ENB, leftPWM);
  driveSide(R_IN1, R_IN2, R_ENA, R_IN3, R_IN4, R_ENB, rightPWM);
}

// =====================================================================
//  WEBSOCKET EVENT HANDLER
//  Text frames: "x,y"  e.g. "0.532,-0.812"
// =====================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {

    case WS_EVT_CONNECT:
      clientCount++;
      Serial.printf("[ws] client #%u connected  (total: %d)\n",
                    client->id(), clientCount);
      lastCommandAt = millis();
      break;

    case WS_EVT_DISCONNECT:
      clientCount = max(0, clientCount - 1);
      Serial.printf("[ws] client #%u disconnected  (total: %d)\n",
                    client->id(), clientCount);
      stopAll();
      motorsStopped = true;
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        char buf[32];
        size_t n = min(len, sizeof(buf) - 1);
        memcpy(buf, data, n);
        buf[n] = '\0';

        float x = 0.0f, y = 0.0f;
        if (sscanf(buf, "%f,%f", &x, &y) == 2) {
          lastCommandAt = millis();
          motorsStopped = false;
          driveFromJoystick(x, y);
        }
      }
      break;
    }

    default:
      break;
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n========================================");
  Serial.println("  ESP32 4WD WiFi RC Car  —  v2");
  Serial.println("========================================");

  // Direction pins
  int dirPins[] = {L_IN1, L_IN2, L_IN3, L_IN4, R_IN1, R_IN2, R_IN3, R_IN4};
  for (int p : dirPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // PWM on enable pins (arduino-esp32 core v3.x API)
  // On core v2.x replace with:
  //   ledcSetup(ch, freq, res); ledcAttachPin(pin, ch);
  //   and use ledcWrite(channel, duty) — not ledcWrite(pin, duty)
  ledcAttach(L_ENA, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(L_ENB, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(R_ENA, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(R_ENB, PWM_FREQ_HZ, PWM_RES_BITS);

  // Status LED — safe to drive GPIO 2 after boot is done
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  stopAll();

  // WiFi access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("SSID     : %s\n", AP_SSID);
  Serial.printf("Password : %s\n", AP_PASSWORD);
  Serial.printf("IP       : %s\n", WiFi.softAPIP().toString().c_str());

  if (MDNS.begin(HOSTNAME)) {
    Serial.printf("mDNS     : http://%s.local\n", HOSTNAME);
  }

  // Serve the embedded web app
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", PAGE_HTML);
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Server   : started");
  Serial.println("========================================");
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  ws.cleanupClients();

  // Fail-safe: stop motors if no command received recently
  if (!motorsStopped && (millis() - lastCommandAt > FAILSAFE_MS)) {
    stopAll();
    motorsStopped = true;
    Serial.println("[failsafe] timeout — motors stopped");
  }

  // Status LED blink:
  //   200 ms period = no client connected
  //  1000 ms period = client connected and driving
  unsigned long blinkInterval = (clientCount > 0) ? 1000UL : 200UL;
  if (millis() - lastLedToggle >= blinkInterval) {
    lastLedToggle = millis();
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}

// =====================================================================
//  EMBEDDED WEB APP
//  Self-contained HTML/CSS/JS — no CDN, no internet needed.
// =====================================================================
const char PAGE_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<title>ESP32 RC Car</title>
<style>
  :root{
    --bg1:#0f1120; --bg2:#1b1e33;
    --accent:#00e5ff; --accent2:#7c4dff; --danger:#ff5252; --go:#33e08a;
    --panel:#181b30; --line:#2a2e4d;
    --rb1:var(--accent); --rb2:var(--danger); --rb3:var(--accent2);
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent;margin:0;padding:0;}
  html,body{width:100%;height:100%;background:linear-gradient(160deg,var(--bg1),var(--bg2));
    color:#f0f2ff;font-family:'Segoe UI',Roboto,-apple-system,sans-serif;
    overflow:hidden;user-select:none;touch-action:none;}

  /* CONNECTION OVERLAY */
  #connOverlay{position:fixed;inset:0;z-index:200;
    background:linear-gradient(160deg,#09091a,#111428);
    display:flex;flex-direction:column;align-items:center;justify-content:center;gap:20px;
    transition:opacity .5s;}
  #connOverlay.hidden{opacity:0;pointer-events:none;}
  .conn-icon{font-size:3rem;animation:pulse 1.4s infinite;}
  @keyframes pulse{0%,100%{transform:scale(1);}50%{transform:scale(1.2);}}
  .conn-title{font-size:1.2rem;font-weight:800;}
  .conn-sub{color:#7b81ac;font-size:.85rem;text-align:center;max-width:260px;line-height:1.6;}
  .conn-spinner{width:44px;height:44px;border:3px solid #2a2e4d;
    border-top-color:var(--accent);border-radius:50%;animation:spin 1s linear infinite;}
  @keyframes spin{to{transform:rotate(360deg);}}
  .conn-error{color:var(--danger);font-size:.85rem;text-align:center;display:none;}
  #retryBtn{display:none;padding:10px 28px;border-radius:12px;border:none;
    background:var(--accent2);color:#fff;font-weight:700;font-size:.9rem;cursor:pointer;}

  /* REVERSE BANNER */
  #reverseBanner{display:none;position:fixed;top:0;left:0;right:0;z-index:50;
    background:var(--danger);color:#fff;text-align:center;
    font-weight:700;letter-spacing:1px;padding:8px;font-size:.85rem;
    animation:pulseBg 1s infinite;}
  @keyframes pulseBg{0%,100%{opacity:1;}50%{opacity:.55;}}
  .reverse-active #reverseBanner{display:block;}

  /* TOP BAR */
  .topbar{display:flex;align-items:center;justify-content:space-between;
    padding:14px 16px;position:relative;z-index:10;}
  .brand{font-weight:800;font-size:1.05rem;letter-spacing:.5px;}
  .status{display:flex;align-items:center;gap:6px;font-size:.8rem;color:#aab0d6;}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--danger);
    box-shadow:0 0 6px var(--danger);transition:.3s;}
  .dot.online{background:var(--go);box-shadow:0 0 8px var(--go);}
  .hamburger{width:42px;height:42px;border:none;border-radius:10px;
    background:#20233d;display:flex;flex-direction:column;
    align-items:center;justify-content:center;gap:5px;cursor:pointer;}
  .hamburger span{width:20px;height:2px;background:var(--rb3);border-radius:2px;}

  /* DRAWER */
  .drawer{position:fixed;top:0;right:-300px;width:280px;max-width:82%;height:100%;
    background:var(--panel);box-shadow:-8px 0 24px #0008;
    padding:24px 20px;z-index:40;transition:right .3s ease;touch-action:auto;overflow-y:auto;}
  .drawer.open{right:0;}
  .drawer h2{margin:0 0 6px;font-size:.85rem;color:#aab0d6;
    text-transform:uppercase;letter-spacing:1.5px;}
  .toggle-row{display:flex;align-items:center;justify-content:space-between;
    padding:16px 0;border-bottom:1px solid var(--line);}
  .toggle-row span.label{font-size:.95rem;}
  .hint{color:#7b81ac;font-size:.78rem;line-height:1.5;margin-top:20px;}
  #closeDrawer{position:absolute;top:16px;right:16px;background:none;
    border:none;color:#7b81ac;font-size:1.3rem;cursor:pointer;}
  .switch{position:relative;width:46px;height:26px;display:inline-block;flex:none;}
  .switch input{opacity:0;width:0;height:0;}
  .switch .slider{position:absolute;inset:0;background:#33375c;
    border-radius:26px;transition:.3s;cursor:pointer;}
  .switch .slider::before{content:'';position:absolute;width:20px;height:20px;
    left:3px;top:3px;background:#fff;border-radius:50%;transition:.3s;}
  .switch input:checked + .slider{background:var(--accent2);}
  .switch input:checked + .slider::before{transform:translateX(20px);}
  #drawerOverlay{position:fixed;inset:0;background:#0009;opacity:0;
    pointer-events:none;transition:.3s;z-index:30;}
  #drawerOverlay.show{opacity:1;pointer-events:auto;}

  /* MAIN */
  main{display:flex;flex-direction:column;align-items:center;justify-content:center;
    height:calc(100% - 120px);gap:22px;}

  /* JOYSTICK */
  #joystickView{display:flex;flex-direction:column;align-items:center;gap:22px;}
  .ring{position:relative;width:230px;height:230px;
    display:flex;align-items:center;justify-content:center;transition:transform .3s;}
  .reverse-active .ring{transform:rotate(180deg);}
  .arrow{position:absolute;color:#4c5180;font-size:1.1rem;}
  .arrow.up{top:0;}.arrow.down{bottom:0;}.arrow.left{left:0;}.arrow.right{right:0;}
  .reverse-active .arrow{color:var(--danger);}
  .joy-base{width:185px;height:185px;border-radius:50%;touch-action:none;
    background:radial-gradient(circle at 35% 30%,#262a4a,#14162a);
    border:2px solid #2f3358;display:flex;align-items:center;justify-content:center;}
  .joy-knob{width:74px;height:74px;border-radius:50%;
    background:radial-gradient(circle at 35% 30%,#4fd8ff,var(--accent));
    box-shadow:0 6px 18px #0007,0 0 24px #00e5ff55;}
  .rainbow-active .joy-knob{background:radial-gradient(circle at 35% 30%,#ffffffaa,var(--rb1));
    box-shadow:0 0 26px var(--rb1);}
  #stopBtn{width:130px;height:54px;border-radius:16px;border:none;
    background:var(--danger);color:#fff;font-weight:800;letter-spacing:1.5px;
    font-size:1rem;box-shadow:0 6px 18px #ff525255;cursor:pointer;}
  .rainbow-active #stopBtn{background:var(--rb2);box-shadow:0 0 22px var(--rb2);}

  /* STEERING WHEEL */
  #steeringView{display:none;flex-direction:column;align-items:center;gap:14px;width:100%;}
  #steeringView.active{display:flex;}
  #joystickView.hidden{display:none;}
  .wheel-wrap{position:relative;width:180px;height:180px;touch-action:none;}
  .wheel-svg{width:180px;height:180px;cursor:grab;}
  .wheel-svg:active{cursor:grabbing;}
  .steer-label{font-size:.75rem;color:#7b81ac;letter-spacing:1px;text-transform:uppercase;}
  .pedals{display:flex;gap:20px;align-items:flex-end;}
  .pedal-col{display:flex;flex-direction:column;align-items:center;gap:8px;}
  .pedal-label{font-size:.7rem;color:#7b81ac;text-transform:uppercase;letter-spacing:.5px;}
  .pedal{width:80px;height:96px;border-radius:14px;border:none;cursor:pointer;
    display:flex;align-items:center;justify-content:center;font-size:1.5rem;
    touch-action:none;transition:transform .1s,box-shadow .1s;}
  .pedal.pressed{transform:scale(.93);}
  #gasBtn{background:linear-gradient(160deg,#1a4d2e,#33e08a55);border:2px solid #33e08a66;
    box-shadow:0 6px 20px #33e08a33;}
  #gasBtn.pressed{box-shadow:0 0 28px #33e08a88;border-color:#33e08a;}
  #brakeBtn{background:linear-gradient(160deg,#4d1a1a,#ff525255);border:2px solid #ff525266;
    box-shadow:0 6px 20px #ff525233;}
  #brakeBtn.pressed{box-shadow:0 0 28px #ff525288;border-color:#ff5252;}
  .speed-bar-wrap,.brake-bar-wrap{width:180px;height:7px;background:#1e2240;
    border-radius:4px;overflow:hidden;}
  .speed-bar{height:100%;width:0%;border-radius:4px;
    background:linear-gradient(90deg,#33e08a,#00e5ff);transition:width .05s;}
  .brake-bar{height:100%;width:0%;border-radius:4px;
    background:linear-gradient(90deg,#ff8a00,#ff5252);transition:width .05s;}

  footer{text-align:center;color:#5a5f8a;font-size:.72rem;padding:8px;}

  @media(max-height:620px){
    main{gap:10px;}
    .ring,.wheel-wrap{width:160px;height:160px;}
    .joy-base{width:130px;height:130px;}.joy-knob{width:52px;height:52px;}
    .wheel-svg{width:160px;height:160px;}
    .pedal{width:68px;height:78px;}
  }
</style>
</head>
<body>

<div id="connOverlay">
  <div class="conn-icon">🚗</div>
  <div class="conn-title">RC CAR</div>
  <div class="conn-spinner" id="connSpinner"></div>
  <div class="conn-sub" id="connSub">Connecting to car&hellip;<br>Make sure you&rsquo;re on the <strong>ESP32-RC-Car</strong> WiFi.</div>
  <div class="conn-error" id="connError">Could not reach the car.<br>Check your WiFi and try again.</div>
  <button id="retryBtn">Retry</button>
</div>

<div id="reverseBanner">&#9888; REVERSE MODE ACTIVE</div>

<header class="topbar">
  <div class="brand">4WD RC CAR</div>
  <div class="status"><span id="statusDot" class="dot"></span><span id="statusText">Connecting&hellip;</span></div>
  <button id="menuBtn" class="hamburger" aria-label="Menu"><span></span><span></span><span></span></button>
</header>

<nav id="drawer" class="drawer">
  <button id="closeDrawer" aria-label="Close">&times;</button>
  <h2>Fun Features</h2>
  <div class="toggle-row">
    <span class="label">Reverse Mode</span>
    <label class="switch"><input type="checkbox" id="reverseToggle"><span class="slider"></span></label>
  </div>
  <div class="toggle-row">
    <span class="label">Rainbow Mode</span>
    <label class="switch"><input type="checkbox" id="rainbowToggle"><span class="slider"></span></label>
  </div>
  <div class="toggle-row">
    <span class="label">Steering Wheel Mode</span>
    <label class="switch"><input type="checkbox" id="wheelToggle"><span class="slider"></span></label>
  </div>
  <p class="hint">
    <strong>Reverse Mode</strong> flips every direction.<br><br>
    <strong>Rainbow Mode</strong> cycles the controls through colours.<br><br>
    <strong>Steering Wheel</strong> gives you a rotatable wheel + gas &amp; brake pedals.
  </p>
</nav>
<div id="drawerOverlay"></div>

<main>
  <div id="joystickView">
    <div class="ring">
      <span class="arrow up">&#9650;</span>
      <span class="arrow down">&#9660;</span>
      <span class="arrow left">&#9664;</span>
      <span class="arrow right">&#9654;</span>
      <div id="joyBase" class="joy-base">
        <div id="joyKnob" class="joy-knob"></div>
      </div>
    </div>
    <button id="stopBtn">STOP</button>
  </div>

  <div id="steeringView">
    <div class="steer-label">Steering</div>
    <div class="wheel-wrap">
      <svg class="wheel-svg" id="wheelSvg" viewBox="0 0 180 180" xmlns="http://www.w3.org/2000/svg">
        <circle cx="90" cy="90" r="82" fill="none" stroke="#2f3358" stroke-width="14"/>
        <path d="M 90 8 A 82 82 0 0 1 172 90" fill="none" stroke="#00e5ff" stroke-width="14" stroke-linecap="round"/>
        <path d="M 172 90 A 82 82 0 0 1 90 172" fill="none" stroke="#2f3358" stroke-width="14" stroke-linecap="round"/>
        <path d="M 90 172 A 82 82 0 0 1 8 90" fill="none" stroke="#00e5ff" stroke-width="14" stroke-linecap="round"/>
        <path d="M 8 90 A 82 82 0 0 1 90 8" fill="none" stroke="#2f3358" stroke-width="14" stroke-linecap="round"/>
        <line x1="90" y1="8"   x2="90"  y2="55"  stroke="#7c4dff" stroke-width="6" stroke-linecap="round"/>
        <line x1="157" y1="136" x2="118" y2="113" stroke="#7c4dff" stroke-width="6" stroke-linecap="round"/>
        <line x1="23"  y1="136" x2="62"  y2="113" stroke="#7c4dff" stroke-width="6" stroke-linecap="round"/>
        <circle cx="90" cy="90" r="26" fill="#1b1e33" stroke="#2f3358" stroke-width="2"/>
        <circle cx="90" cy="90" r="18" fill="#20233d"/>
        <text x="90" y="95" text-anchor="middle" font-size="10" fill="#00e5ff" font-family="sans-serif" font-weight="bold">4WD</text>
      </svg>
    </div>
    <div class="speed-bar-wrap"><div class="speed-bar" id="speedBar"></div></div>
    <div class="brake-bar-wrap"><div class="brake-bar" id="brakeBar"></div></div>
    <div class="pedals">
      <div class="pedal-col">
        <div id="brakeBtn" class="pedal">🛑</div>
        <div class="pedal-label">Brake</div>
      </div>
      <div class="pedal-col">
        <div id="gasBtn" class="pedal">⚡</div>
        <div class="pedal-label">Gas</div>
      </div>
    </div>
  </div>
</main>

<footer id="footer">Drag the stick to drive &bull; &#9776; for fun modes</footer>

<script>
// ── STATE ──────────────────────────────────────────────────────────────
var pendingX = 0, pendingY = 0;
var reverseMode     = localStorage.getItem('reverseMode')  === '1';
var rainbowMode     = localStorage.getItem('rainbowMode')  === '1';
var wheelModeActive = localStorage.getItem('wheelMode')    === '1';
var steerAngle = 0, MAX_STEER = 120;
var wheelDown = false, wheelStartAngle = 0, wheelStartSteer = 0;
var gasHeld = false, brakeHeld = false, gasLevel = 0, brakeLevel = 0;
var originX = 0, originY = 0, maxRadius = 60, dragging = false;
var hue = 0, rainbowRAF = null;
var ws, wsConnected = false, connAttempts = 0, overlayGone = false;

// ── DOM REFS (set in load) ─────────────────────────────────────────────
var connOverlay, connSpinner, connSub, connError, retryBtn;
var statusDot, statusText;
var joyBase, joyKnob, drawer, drawerOverlay;
var reverseToggle, rainbowToggle, wheelToggle;
var joystickView, steeringView, footerEl;
var wheelSvg, gasBtn, brakeBtn, speedBar, brakeBar;

window.addEventListener('load', function(){
  connOverlay   = document.getElementById('connOverlay');
  connSpinner   = document.getElementById('connSpinner');
  connSub       = document.getElementById('connSub');
  connError     = document.getElementById('connError');
  retryBtn      = document.getElementById('retryBtn');
  statusDot     = document.getElementById('statusDot');
  statusText    = document.getElementById('statusText');
  joyBase       = document.getElementById('joyBase');
  joyKnob       = document.getElementById('joyKnob');
  drawer        = document.getElementById('drawer');
  drawerOverlay = document.getElementById('drawerOverlay');
  reverseToggle = document.getElementById('reverseToggle');
  rainbowToggle = document.getElementById('rainbowToggle');
  wheelToggle   = document.getElementById('wheelToggle');
  joystickView  = document.getElementById('joystickView');
  steeringView  = document.getElementById('steeringView');
  footerEl      = document.getElementById('footer');
  wheelSvg      = document.getElementById('wheelSvg');
  gasBtn        = document.getElementById('gasBtn');
  brakeBtn      = document.getElementById('brakeBtn');
  speedBar      = document.getElementById('speedBar');
  brakeBar      = document.getElementById('brakeBar');

  retryBtn.addEventListener('click', function(){
    connError.style.display   = 'none';
    retryBtn.style.display    = 'none';
    connSpinner.style.display = 'block';
    connSub.style.display     = 'block';
    connectWS();
  });

  document.getElementById('menuBtn').addEventListener('click', openDrawer);
  document.getElementById('closeDrawer').addEventListener('click', closeDrawerFn);
  drawerOverlay.addEventListener('click', closeDrawerFn);

  reverseToggle.checked = reverseMode;
  document.body.classList.toggle('reverse-active', reverseMode);
  reverseToggle.addEventListener('change', function(){
    reverseMode = reverseToggle.checked;
    localStorage.setItem('reverseMode', reverseMode ? '1' : '0');
    document.body.classList.toggle('reverse-active', reverseMode);
  });

  rainbowToggle.checked = rainbowMode;
  applyRainbow(rainbowMode);
  rainbowToggle.addEventListener('change', function(){
    rainbowMode = rainbowToggle.checked;
    localStorage.setItem('rainbowMode', rainbowMode ? '1' : '0');
    applyRainbow(rainbowMode);
  });

  wheelToggle.checked = wheelModeActive;
  applyWheelMode(wheelModeActive);
  wheelToggle.addEventListener('change', function(){
    wheelModeActive = wheelToggle.checked;
    localStorage.setItem('wheelMode', wheelModeActive ? '1' : '0');
    applyWheelMode(wheelModeActive);
    closeDrawerFn();
  });

  // Joystick events
  joyCenter();
  window.addEventListener('resize', joyCenter);
  joyBase.addEventListener('pointerdown', function(e){
    joyCenter(); dragging = true;
    joyBase.setPointerCapture(e.pointerId);
    updateJoy(e.clientX, e.clientY);
  });
  joyBase.addEventListener('pointermove',   function(e){ if(!dragging) return; updateJoy(e.clientX, e.clientY); });
  joyBase.addEventListener('pointerup',     function(){ dragging = false; resetJoy(); });
  joyBase.addEventListener('pointercancel', function(){ dragging = false; resetJoy(); });
  document.getElementById('stopBtn').addEventListener('click', resetJoy);

  // Keyboard
  var ks = {ArrowUp:0,ArrowDown:0,ArrowLeft:0,ArrowRight:0};
  window.addEventListener('keydown', function(e){ if(e.key in ks){ ks[e.key]=1; fromKeys(); e.preventDefault(); }});
  window.addEventListener('keyup',   function(e){ if(e.key in ks){ ks[e.key]=0; fromKeys(); e.preventDefault(); }});
  function fromKeys(){
    if(dragging||wheelModeActive) return;
    var ky=ks.ArrowUp-ks.ArrowDown, kx=ks.ArrowRight-ks.ArrowLeft;
    setKnob(kx*maxRadius,-ky*maxRadius);
    pendingX=reverseMode?-kx:kx; pendingY=reverseMode?-ky:ky;
  }

  // Wheel events
  wheelSvg.addEventListener('pointerdown', function(e){
    wheelDown=true; wheelSvg.setPointerCapture(e.pointerId);
    wheelStartAngle=pointerAngle(e); wheelStartSteer=steerAngle;
    e.preventDefault();
  });
  wheelSvg.addEventListener('pointermove', function(e){
    if(!wheelDown) return;
    var d=pointerAngle(e)-wheelStartAngle;
    while(d>180) d-=360; while(d<-180) d+=360;
    steerAngle=Math.max(-MAX_STEER,Math.min(MAX_STEER,wheelStartSteer+d));
    if(wheelSvg) wheelSvg.style.transform='rotate('+steerAngle+'deg)';
    computeWheelOutput(); e.preventDefault();
  });
  wheelSvg.addEventListener('pointerup',    function(){ wheelDown=false; steerAngle=0; if(wheelSvg) wheelSvg.style.transform=''; computeWheelOutput(); });
  wheelSvg.addEventListener('pointercancel',function(){ wheelDown=false; steerAngle=0; if(wheelSvg) wheelSvg.style.transform=''; computeWheelOutput(); });

  // Pedal events
  gasBtn.addEventListener('pointerdown',   function(e){ gasHeld=true;   gasBtn.classList.add('pressed');   e.preventDefault(); });
  gasBtn.addEventListener('pointerup',     function(e){ gasHeld=false;  gasBtn.classList.remove('pressed'); e.preventDefault(); });
  gasBtn.addEventListener('pointercancel', function(e){ gasHeld=false;  gasBtn.classList.remove('pressed'); e.preventDefault(); });
  brakeBtn.addEventListener('pointerdown',   function(e){ brakeHeld=true;  brakeBtn.classList.add('pressed');   e.preventDefault(); });
  brakeBtn.addEventListener('pointerup',     function(e){ brakeHeld=false; brakeBtn.classList.remove('pressed'); e.preventDefault(); });
  brakeBtn.addEventListener('pointercancel', function(e){ brakeHeld=false; brakeBtn.classList.remove('pressed'); e.preventDefault(); });

  requestAnimationFrame(rampLoop);
  connectWS();
});

// ── WEBSOCKET ──────────────────────────────────────────────────────────
function connectWS(){
  connAttempts++;
  var proto=location.protocol==='https:'?'wss://':'ws://';
  ws=new WebSocket(proto+location.host+'/ws');
  var t=setTimeout(function(){ if(!wsConnected){ ws.close(); if(!overlayGone) showConnError(); } },5000);
  ws.onopen =function(){ clearTimeout(t); wsConnected=true;  updateStatus(); showConnected(); };
  ws.onclose=function(){ clearTimeout(t); wsConnected=false; updateStatus();
    if(!overlayGone&&connAttempts>=3){ showConnError(); return; }
    setTimeout(connectWS,1500); };
  ws.onerror=function(){ ws.close(); };
}
function updateStatus(){
  if(!statusDot) return;
  statusDot.classList.toggle('online',wsConnected);
  statusText.textContent=wsConnected?'Connected':'Reconnecting\u2026';
}
function showConnected(){
  if(overlayGone) return; overlayGone=true;
  connOverlay.classList.add('hidden');
  setTimeout(function(){ connOverlay.style.display='none'; },550);
}
function showConnError(){
  connSpinner.style.display='none'; connSub.style.display='none';
  connError.style.display='block'; retryBtn.style.display='block';
}
setInterval(function(){
  if(wsConnected&&ws&&ws.readyState===WebSocket.OPEN)
    ws.send(pendingX.toFixed(3)+','+pendingY.toFixed(3));
},62);

// ── JOYSTICK ───────────────────────────────────────────────────────────
function joyCenter(){
  if(!joyBase) return;
  var r=joyBase.getBoundingClientRect();
  originX=r.left+r.width/2; originY=r.top+r.height/2;
  maxRadius=r.width/2-(joyKnob?joyKnob.offsetWidth/2:30);
}
function setKnob(dx,dy){ if(joyKnob) joyKnob.style.transform='translate('+dx+'px,'+dy+'px)'; }
function updateJoy(cx,cy){
  var dx=cx-originX,dy=cy-originY,dist=Math.hypot(dx,dy);
  if(dist>maxRadius&&dist>0){dx=dx/dist*maxRadius;dy=dy/dist*maxRadius;}
  setKnob(dx,dy);
  var x=maxRadius>0?dx/maxRadius:0, y=maxRadius>0?-dy/maxRadius:0;
  if(reverseMode){x=-x;y=-y;}
  pendingX=x; pendingY=y;
}
function resetJoy(){ setKnob(0,0); pendingX=0; pendingY=0; }

// ── RAINBOW ────────────────────────────────────────────────────────────
function applyRainbow(on){
  document.body.classList.toggle('rainbow-active',on);
  if(on) animRainbow(); else if(rainbowRAF){ cancelAnimationFrame(rainbowRAF); rainbowRAF=null; }
}
function animRainbow(){
  hue=(hue+2)%360;
  var s=document.documentElement.style;
  s.setProperty('--rb1','hsl('+hue+',85%,72%)');
  s.setProperty('--rb2','hsl('+((hue+120)%360)+',85%,72%)');
  s.setProperty('--rb3','hsl('+((hue+240)%360)+',85%,72%)');
  if(rainbowMode) rainbowRAF=requestAnimationFrame(animRainbow);
}

// ── WHEEL MODE ─────────────────────────────────────────────────────────
function applyWheelMode(on){
  if(!joystickView||!steeringView) return;
  if(on){
    joystickView.classList.add('hidden'); steeringView.classList.add('active');
    if(footerEl) footerEl.textContent='Rotate wheel \u2022 Hold Gas / Brake \u2022 \u2630 for modes';
  } else {
    joystickView.classList.remove('hidden'); steeringView.classList.remove('active');
    if(footerEl) footerEl.textContent='Drag the stick to drive \u2022 \u2630 for fun modes';
    steerAngle=0; gasHeld=false; brakeHeld=false;
    if(wheelSvg) wheelSvg.style.transform='';
  }
  pendingX=0; pendingY=0;
}
function pointerAngle(e){
  var r=wheelSvg.getBoundingClientRect();
  return Math.atan2(e.clientY-(r.top+r.height/2),e.clientX-(r.left+r.width/2))*180/Math.PI;
}
function computeWheelOutput(){
  if(!wheelModeActive) return;
  var tx=steerAngle/MAX_STEER;
  var ty=(gasHeld&&!brakeHeld)?gasLevel:(brakeHeld&&!gasHeld)?-brakeLevel:0;
  if(reverseMode){tx=-tx;ty=-ty;}
  pendingX=tx; pendingY=ty;
}

// ── PEDAL RAMP ─────────────────────────────────────────────────────────
function rampLoop(){
  if(gasHeld)   gasLevel=Math.min(1,gasLevel+0.04);   else gasLevel=Math.max(0,gasLevel-0.06);
  if(brakeHeld) brakeLevel=Math.min(1,brakeLevel+0.05); else brakeLevel=Math.max(0,brakeLevel-0.06);
  if(speedBar) speedBar.style.width=(gasLevel*100)+'%';
  if(brakeBar) brakeBar.style.width=(brakeLevel*100)+'%';
  if(wheelModeActive) computeWheelOutput();
  requestAnimationFrame(rampLoop);
}

// ── DRAWER ─────────────────────────────────────────────────────────────
function openDrawer(){ if(drawer) drawer.classList.add('open'); if(drawerOverlay) drawerOverlay.classList.add('show'); }
function closeDrawerFn(){ if(drawer) drawer.classList.remove('open'); if(drawerOverlay) drawerOverlay.classList.remove('show'); }
</script>
</body>
</html>
)HTMLPAGE";
