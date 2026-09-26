/* =====================================================================
   ESP32 WiFi RC Car  —  revamped from "Obstacle-Avoiding Robot w/ L298N"
   =====================================================================

   WHAT CHANGED FROM THE ORIGINAL ARDUINO CODE
   --------------------------------------------
   The original sketch drove an L298N with plain digitalWrite() HIGH/LOW
   (full speed only, no steering feel) and steered itself using an
   ultrasonic sensor on a servo. This version keeps the same L298N wiring
   idea (2 motors, 4 direction pins) but:
     - Replaces autonomous obstacle avoidance with real-time WiFi control
       from a phone/laptop browser (the servo + ultrasonic sensor are not
       used by this sketch — see README "Bonus: bring back the sensor").
     - Adds PWM speed control (ENA/ENB) so the car has variable speed
       instead of just on/off.
     - Hosts its own WiFi hotspot + a game-style single-page web app
       (embedded below as PAGE_HTML) with a virtual joystick, a hamburger
       menu with "Reverse Mode" and "Rainbow Mode", and a big STOP button.
     - Adds a connection fail-safe: if the browser stops sending
       commands (WiFi drops, phone locks, tab closes) the car
       auto-stops within ~400 ms.

   HARDWARE
   --------
   ESP32 Dev Board (WROOM-32) + L298N dual motor driver + 2x DC gear motors.

     L298N pin      ESP32 GPIO      Notes
     ------------   ----------      -----------------------------------
     IN1 (Left +)   GPIO 27         Left motor direction
     IN2 (Left -)   GPIO 26         Left motor direction
     IN3 (Right +)  GPIO 25         Right motor direction
     IN4 (Right -)  GPIO 33         Right motor direction
     ENA (Left PWM) GPIO 14         Left motor speed (remove ENA jumper!)
     ENB (Right PWM)GPIO 32         Right motor speed (remove ENB jumper!)
     GND            GND             Common ground: ESP32 <-> L298N <-> battery
     5V (logic)     —               From L298N onboard reg. (see README)

   These 6 GPIOs were chosen to avoid every ESP32 boot-strapping pin
   (0, 2, 5, 12, 15), the flash-SPI pins (6-11), and the input-only
   pins (34-39) — so there's no boot-mode or flashing conflict.

   LIBRARIES  (Arduino IDE > Tools > Manage Libraries...)
     - "ESPAsyncWebServer" (ESP32Async fork — search ESPAsyncWebServer and
        confirm the author is "ESP32Async"; if it's not in your Library
        Manager index, install the ZIP from
        https://github.com/ESP32Async/ESPAsyncWebServer)
     - "AsyncTCP" (ESP32Async fork —
        https://github.com/ESP32Async/AsyncTCP)

   BOARD
     Tools > Board > "ESP32 Dev Module"
     Requires Arduino-ESP32 core v3.x (for the ledcAttach/ledcWrite API
     used below). Update via Boards Manager if you're on core v2.x.
   ===================================================================== */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

/* =====================================================================
   Embedded web app (served by the ESP32 itself).
   No external CDNs/fonts/scripts — the AP hotspot has no internet, so
   everything the page needs (HTML/CSS/JS) lives in this one string.
   ===================================================================== */

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
    --accent:#00e5ff; --accent2:#7c4dff; --danger:#ff5252;
    --panel:#181b30; --line:#2a2e4d;
    --rb1:var(--accent); --rb2:var(--danger); --rb3:var(--accent2);
  }
  *{box-sizing:border-box; -webkit-tap-highlight-color:transparent;}
  html,body{
    margin:0; padding:0; width:100%; height:100%;
    background:linear-gradient(160deg,var(--bg1),var(--bg2));
    color:#f0f2ff; font-family:'Segoe UI',Roboto,-apple-system,sans-serif;
    overflow:hidden; user-select:none; touch-action:none;
  }
  #reverseBanner{
    display:none; position:fixed; top:0; left:0; right:0; z-index:50;
    background:var(--danger); color:#fff; text-align:center;
    font-weight:700; letter-spacing:1px; padding:8px; font-size:.85rem;
    animation:pulseBg 1s infinite;
  }
  @keyframes pulseBg{0%,100%{opacity:1;}50%{opacity:.55;}}
  .reverse-active #reverseBanner{display:block;}

  .topbar{
    display:flex; align-items:center; justify-content:space-between;
    padding:14px 16px; position:relative; z-index:10;
  }
  .brand{font-weight:800; font-size:1.1rem; letter-spacing:.5px;}
  .status{display:flex; align-items:center; gap:6px; font-size:.8rem; color:#aab0d6;}
  .dot{width:9px; height:9px; border-radius:50%; background:var(--danger);
    box-shadow:0 0 6px var(--danger); transition:.3s;}
  .dot.online{background:#33e08a; box-shadow:0 0 8px #33e08a;}

  .hamburger{
    width:42px; height:42px; border:none; border-radius:10px;
    background:#20233d; display:flex; flex-direction:column;
    align-items:center; justify-content:center; gap:5px; cursor:pointer;
  }
  .hamburger span{width:20px; height:2px; background:var(--rb3); border-radius:2px;}

  .drawer{
    position:fixed; top:0; right:-300px; width:280px; max-width:82%; height:100%;
    background:var(--panel); box-shadow:-8px 0 24px #0008;
    padding:24px 20px; z-index:40; transition:right .3s ease; touch-action:auto;
  }
  .drawer.open{right:0;}
  .drawer h2{margin:0 0 6px; font-size:.85rem; color:#aab0d6;
    text-transform:uppercase; letter-spacing:1.5px;}
  .toggle-row{display:flex; align-items:center; justify-content:space-between;
    padding:16px 0; border-bottom:1px solid var(--line);}
  .toggle-row span.label{font-size:.95rem;}
  .hint{color:#7b81ac; font-size:.78rem; line-height:1.5; margin-top:20px;}
  #closeDrawer{position:absolute; top:16px; right:16px; background:none;
    border:none; color:#7b81ac; font-size:1.3rem; cursor:pointer;}

  .switch{position:relative; width:46px; height:26px; display:inline-block; flex:none;}
  .switch input{opacity:0; width:0; height:0;}
  .switch .slider{position:absolute; inset:0; background:#33375c;
    border-radius:26px; transition:.3s; cursor:pointer;}
  .switch .slider::before{content:''; position:absolute; width:20px; height:20px;
    left:3px; top:3px; background:#fff; border-radius:50%; transition:.3s;}
  .switch input:checked + .slider{background:var(--accent2);}
  .switch input:checked + .slider::before{transform:translateX(20px);}

  #drawerOverlay{position:fixed; inset:0; background:#0009; opacity:0;
    pointer-events:none; transition:.3s; z-index:30;}
  #drawerOverlay.show{opacity:1; pointer-events:auto;}

  main{
    display:flex; flex-direction:column; align-items:center; justify-content:center;
    height:calc(100% - 120px); gap:30px;
  }

  .ring{position:relative; width:240px; height:240px;
    display:flex; align-items:center; justify-content:center; transition:transform .3s;}
  .reverse-active .ring{transform:rotate(180deg);}
  .arrow{position:absolute; color:#4c5180; font-size:1.1rem; transition:color .3s;}
  .arrow.up{top:0;} .arrow.down{bottom:0;} .arrow.left{left:0;} .arrow.right{right:0;}
  .reverse-active .arrow{color:var(--danger);}

  .joy-base{
    width:190px; height:190px; border-radius:50%; touch-action:none;
    background:radial-gradient(circle at 35% 30%,#262a4a,#14162a);
    border:2px solid #2f3358; display:flex; align-items:center; justify-content:center;
  }
  .joy-knob{
    width:76px; height:76px; border-radius:50%;
    background:radial-gradient(circle at 35% 30%,#4fd8ff,var(--accent));
    box-shadow:0 6px 18px #0007, 0 0 24px #00e5ff55;
    transition:background .15s, box-shadow .15s;
  }
  .rainbow-active .joy-knob{
    background:radial-gradient(circle at 35% 30%,#ffffffaa,var(--rb1));
    box-shadow:0 0 26px var(--rb1);
  }

  #stopBtn{
    width:130px; height:58px; border-radius:16px; border:none;
    background:var(--danger); color:#fff; font-weight:800; letter-spacing:1.5px;
    font-size:1rem; box-shadow:0 6px 18px #ff525255; cursor:pointer;
  }
  .rainbow-active #stopBtn{background:var(--rb2); box-shadow:0 0 22px var(--rb2);}

  footer{text-align:center; color:#5a5f8a; font-size:.72rem; padding:8px;}

  @media (max-height:600px){
    main{gap:14px;}
    .ring{width:190px; height:190px;}
    .joy-base{width:150px; height:150px;}
    .joy-knob{width:60px; height:60px;}
  }
</style>
</head>
<body>

  <div id="reverseBanner">&#9888; REVERSE MODE ACTIVE</div>

  <header class="topbar">
    <div class="brand">RC CAR</div>
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
    <p class="hint">Reverse Mode flips every direction — forward becomes backward, left becomes right, and vice versa.<br><br>
    Rainbow Mode is just for show — the controls cycle color. It doesn't change how the car drives.</p>
  </nav>
  <div id="drawerOverlay"></div>

  <main>
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
  </main>

  <footer>Drag the stick to drive &bull; arrow keys work too &bull; &#9776; for fun modes</footer>

<script>
  // ---------------- elements ----------------
  var joyBase = document.getElementById('joyBase');
  var joyKnob = document.getElementById('joyKnob');
  var menuBtn = document.getElementById('menuBtn');
  var closeDrawer = document.getElementById('closeDrawer');
  var drawer = document.getElementById('drawer');
  var drawerOverlay = document.getElementById('drawerOverlay');
  var reverseToggle = document.getElementById('reverseToggle');
  var rainbowToggle = document.getElementById('rainbowToggle');
  var stopBtn = document.getElementById('stopBtn');
  var statusDot = document.getElementById('statusDot');
  var statusText = document.getElementById('statusText');

  // ---------------- joystick geometry ----------------
  var originX = 0, originY = 0, maxRadius = 60;
  var dragging = false;

  function joyCenter(){
    var rect = joyBase.getBoundingClientRect();
    originX = rect.left + rect.width / 2;
    originY = rect.top + rect.height / 2;
    maxRadius = rect.width / 2 - joyKnob.offsetWidth / 2;
  }
  window.addEventListener('load', joyCenter);
  window.addEventListener('resize', joyCenter);

  function setKnob(dx, dy){
    joyKnob.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
  }

  function updateVector(clientX, clientY){
    var dx = clientX - originX;
    var dy = clientY - originY;
    var dist = Math.hypot(dx, dy);
    if (dist > maxRadius && dist > 0){
      dx = dx / dist * maxRadius;
      dy = dy / dist * maxRadius;
    }
    setKnob(dx, dy);

    var x = maxRadius > 0 ? dx / maxRadius : 0;   // -1..1, right positive
    var y = maxRadius > 0 ? -dy / maxRadius : 0;  // -1..1, forward positive

    if (reverseMode){ x = -x; y = -y; }
    pendingX = x; pendingY = y;
  }

  function resetKnob(){
    setKnob(0, 0);
    pendingX = 0; pendingY = 0;
  }

  joyBase.addEventListener('pointerdown', function(e){
    joyCenter();
    dragging = true;
    joyBase.setPointerCapture(e.pointerId);
    updateVector(e.clientX, e.clientY);
  });
  joyBase.addEventListener('pointermove', function(e){
    if (!dragging) return;
    updateVector(e.clientX, e.clientY);
  });
  joyBase.addEventListener('pointerup', function(){ dragging = false; resetKnob(); });
  joyBase.addEventListener('pointercancel', function(){ dragging = false; resetKnob(); });

  // ---------------- keyboard fallback (desktop testing) ----------------
  var keyState = {ArrowUp:0, ArrowDown:0, ArrowLeft:0, ArrowRight:0};
  window.addEventListener('keydown', function(e){
    if (e.key in keyState){ keyState[e.key] = 1; updateFromKeys(); e.preventDefault(); }
  });
  window.addEventListener('keyup', function(e){
    if (e.key in keyState){ keyState[e.key] = 0; updateFromKeys(); e.preventDefault(); }
  });
  function updateFromKeys(){
    if (dragging) return;
    var ky = keyState.ArrowUp - keyState.ArrowDown;
    var kx = keyState.ArrowRight - keyState.ArrowLeft;
    setKnob(kx * maxRadius, -ky * maxRadius);
    var x = kx, y = ky;
    if (reverseMode){ x = -x; y = -y; }
    pendingX = x; pendingY = y;
  }

  // ---------------- websocket ----------------
  var ws, wsConnected = false;
  var pendingX = 0, pendingY = 0;

  function connectWS(){
    var proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
    ws = new WebSocket(proto + location.host + '/ws');
    ws.onopen = function(){ wsConnected = true; updateStatus(); };
    ws.onclose = function(){ wsConnected = false; updateStatus(); setTimeout(connectWS, 1500); };
    ws.onerror = function(){ ws.close(); };
  }
  function updateStatus(){
    statusDot.classList.toggle('online', wsConnected);
    statusText.textContent = wsConnected ? 'Connected' : 'Reconnecting\u2026';
  }
  setInterval(function(){
    if (wsConnected && ws.readyState === WebSocket.OPEN){
      ws.send(pendingX.toFixed(3) + ',' + pendingY.toFixed(3));
    }
  }, 60);
  connectWS();

  stopBtn.addEventListener('click', resetKnob);

  // ---------------- hamburger drawer ----------------
  function openDrawer(){ drawer.classList.add('open'); drawerOverlay.classList.add('show'); }
  function closeDrawerFn(){ drawer.classList.remove('open'); drawerOverlay.classList.remove('show'); }
  menuBtn.addEventListener('click', openDrawer);
  closeDrawer.addEventListener('click', closeDrawerFn);
  drawerOverlay.addEventListener('click', closeDrawerFn);

  // ---------------- Reverse Mode (fun feature #1) ----------------
  var reverseMode = localStorage.getItem('reverseMode') === '1';
  reverseToggle.checked = reverseMode;
  document.body.classList.toggle('reverse-active', reverseMode);
  reverseToggle.addEventListener('change', function(){
    reverseMode = reverseToggle.checked;
    localStorage.setItem('reverseMode', reverseMode ? '1' : '0');
    document.body.classList.toggle('reverse-active', reverseMode);
  });

  // ---------------- Rainbow Mode (fun feature #2) ----------------
  var rainbowMode = localStorage.getItem('rainbowMode') === '1';
  var hue = 0, rainbowRAF = null;
  rainbowToggle.checked = rainbowMode;
  function applyRainbow(on){
    document.body.classList.toggle('rainbow-active', on);
    if (on){
      animateRainbow();
    } else if (rainbowRAF){
      cancelAnimationFrame(rainbowRAF);
      rainbowRAF = null;
    }
  }
  function animateRainbow(){
    hue = (hue + 2) % 360;
    var root = document.documentElement.style;
    root.setProperty('--rb1', 'hsl(' + hue + ',85%,72%)');
    root.setProperty('--rb2', 'hsl(' + ((hue + 120) % 360) + ',85%,72%)');
    root.setProperty('--rb3', 'hsl(' + ((hue + 240) % 360) + ',85%,72%)');
    if (rainbowMode) rainbowRAF = requestAnimationFrame(animateRainbow);
  }
  applyRainbow(rainbowMode);
  rainbowToggle.addEventListener('change', function(){
    rainbowMode = rainbowToggle.checked;
    localStorage.setItem('rainbowMode', rainbowMode ? '1' : '0');
    applyRainbow(rainbowMode);
  });
</script>
</body>
</html>
)HTMLPAGE";

// ---------------- Wi-Fi hotspot settings ----------------
// Your phone/laptop connects DIRECTLY to this network — no home router
// or internet needed, which is ideal for a demo table.
const char *AP_SSID     = "ESP32-RC-Car";
const char *AP_PASSWORD = "rccar123";   // 8+ chars (WPA2). Use "" for open.
const char *HOSTNAME    = "esp32car";   // -> http://esp32car.local

// ---------------- L298N motor pins ----------------
#define IN1 27   // Left motor  - forward
#define IN2 26   // Left motor  - backward
#define IN3 25   // Right motor - forward
#define IN4 33   // Right motor - backward
#define ENA 14   // Left motor  - PWM speed
#define ENB 32   // Right motor - PWM speed

const int PWM_FREQ_HZ = 5000;   // above audible whine range
const int PWM_RES_BITS = 8;     // 0-255 duty

// ---------------- Safety fail-safe ----------------
// If no drive command arrives for this long, stop the motors. The web
// page pings a command at a steady rate whenever it's connected (even
// "stop" while the stick is centered), so a WiFi drop or closed tab
// is caught almost immediately.
const unsigned long FAILSAFE_MS = 400;
unsigned long lastCommandAt = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------------------------------------------------------------
// Low-level motor drive. speed: -255 (full reverse) .. 255 (full fwd)
// ---------------------------------------------------------------------
void driveSide(int pinFwd, int pinBwd, int enPin, int speed) {
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

void stopCar() {
  driveSide(IN1, IN2, ENA, 0);
  driveSide(IN3, IN4, ENB, 0);
}

// ---------------------------------------------------------------------
// Arcade-style differential mixing.
//   x = turn, -1 (full left) .. 1 (full right)
//   y = throttle, -1 (full reverse) .. 1 (full forward)
// Reverse Mode is handled entirely on the web page (it just flips the
// x/y it sends), so this firmware never needs to know about it.
// ---------------------------------------------------------------------
void driveFromJoystick(float x, float y) {
  x = constrain(x, -1.0f, 1.0f);
  y = constrain(y, -1.0f, 1.0f);

  float left  = y + x;
  float right = y - x;

  float maxMag = max(abs(left), abs(right));
  if (maxMag > 1.0f) {
    left  /= maxMag;
    right /= maxMag;
  }

  driveSide(IN1, IN2, ENA, (int)(left  * 255));
  driveSide(IN3, IN4, ENB, (int)(right * 255));
}

// ---------------------------------------------------------------------
// WebSocket event handler. Text frames look like "x,y", e.g. "0.532,-0.812"
// ---------------------------------------------------------------------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[ws] client #%u connected from %s\n",
                    client->id(), client->remoteIP().toString().c_str());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("[ws] client #%u disconnected\n", client->id());
      stopCar();   // safety: stop the instant the controlling page goes away
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        char buf[32];
        size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
        memcpy(buf, data, n);
        buf[n] = '\0';

        float x = 0, y = 0;
        if (sscanf(buf, "%f,%f", &x, &y) == 2) {
          lastCommandAt = millis();
          driveFromJoystick(x, y);
        }
      }
      break;
    }

    default:
      break;
  }
}

// =======================================================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // core v3.x LEDC API: attach a pin directly, no manual channel number.
  // --- On arduino-esp32 core v2.x instead use:
  //   ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(ENA, 0);
  //   ledcSetup(1, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(ENB, 1);
  //   ...and call ledcWrite(0, duty) / ledcWrite(1, duty) instead of
  //   ledcWrite(ENA, duty) / ledcWrite(ENB, duty) in driveSide().
  ledcAttach(ENA, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(ENB, PWM_FREQ_HZ, PWM_RES_BITS);

  stopCar();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("Hotspot IP: ");
  Serial.println(WiFi.softAPIP());   // normally 192.168.4.1

  if (MDNS.begin(HOSTNAME)) {
    Serial.printf("Open: http://%s.local  (or the IP above)\n", HOSTNAME);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", PAGE_HTML);
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();
  Serial.println("Server started.");
}

void loop() {
  ws.cleanupClients();

  if (millis() - lastCommandAt > FAILSAFE_MS) {
    stopCar();
  }
}
