// ================================================================
//  ESP32 4WD ROVER — WiFi OTA + Web Control + PID
//
//  FIRST TIME: upload this via USB cable.
//  AFTER THAT: Arduino IDE → Tools → Port → Network Ports
//              → esp32-rover at [IP address shown in Serial]
//
//  WEB CONTROL: open browser on same WiFi network
//              → http://[IP shown in Serial Monitor]
//
//  Libraries needed (all built into ESP32 Arduino Core):
//    WiFi.h, ArduinoOTA.h, WebServer.h
// ================================================================

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

// ================================================================
//  WiFi CREDENTIALS — change before uploading
// ================================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* OTA_HOSTNAME  = "esp32-rover";
const char* OTA_PASSWORD  = "YOUR_OTA_PASSWORD";    // required when uploading OTA

// ================================================================
//  PINS — unchanged from your hardware
// ================================================================
#define FL_PWM   27
#define FL_IN1   25
#define FL_IN2   26

#define FR_PWM   13
#define FR_IN1   14
#define FR_IN2   12

#define STBY_FRONT  33

#define RL_PWM   21
#define RL_IN1   18
#define RL_IN2   19

#define RR_PWM    5
#define RR_IN1   22
#define RR_IN2   23

#define STBY_REAR   32

// ---- Encoder pins ----
#define FL_ENC_A   34
#define FL_ENC_B   35
#define FR_ENC_A   36   // SVP
#define FR_ENC_B   39   // SVN
#define RL_ENC_A    4
#define RL_ENC_B   16
#define RR_ENC_A   17
#define RR_ENC_B   15

// ================================================================
//  MOTOR POLARITY — flip to -1 if wheel spins wrong way
// ================================================================
#define DIR_FL  -1
#define DIR_FR   1
#define DIR_RL  -1
#define DIR_RR   1

// ================================================================
//  CALIBRATION
//
//  MAX_SPEED_TICKS = encoder ticks per second at full PWM (255).
//
//  HOW TO MEASURE:
//    1. Upload this code
//    2. Open Serial Monitor at 115200
//    3. Connect battery, type M and press enter
//    4. It runs RL motor at full speed for 3 seconds and prints
//       average ticks/sec. Use that number here, then re-upload.
//
//  Start with 400 — PID will partially compensate even if wrong,
//  but calibrating this makes PID converge much faster.
// ================================================================
float MAX_SPEED_TICKS = 2865.0f;

// ================================================================
//  PID SETTINGS
//
//  TUNING PROCEDURE (do this after calibrating MAX_SPEED_TICKS):
//    Step 1: Set Ki=0, Kd=0. Increase Kp until motor responds
//            quickly without oscillating. Start at 1.0, try 2.0.
//    Step 2: Add small Ki (try 0.5) to eliminate steady-state
//            speed error at constant target.
//    Step 3: If overshooting on direction change, add small Kd
//            (try 0.05). Increase carefully — too much = instability.
//
//  These can also be changed live via the web interface.
// ================================================================
float Kp = 0.15f;
float Ki = 0.23f;
float Kd = 0.00f;

#define PID_INTERVAL_MS  50    // PID runs every 50ms = 20Hz update rate

// ================================================================
//  ENCODER COUNTERS — written by ISR, read by PID
// ================================================================
volatile long ticksFL = 0, ticksFR = 0, ticksRL = 0, ticksRR = 0;
long prevTicksFL = 0, prevTicksFR = 0, prevTicksRL = 0, prevTicksRR = 0;

// ================================================================
//  MOTOR STATE — visible to web /status endpoint
// ================================================================
float targetFL = 0, targetFR = 0, targetRL = 0, targetRR = 0;
float actualFL = 0, actualFR = 0, actualRL = 0, actualRR = 0;
float pwmFL    = 0, pwmFR    = 0, pwmRL    = 0, pwmRR    = 0;

// ================================================================
//  PID INTEGRATORS AND PREVIOUS ERROR — one set per motor
// ================================================================
float intFL = 0, intFR = 0, intRL = 0, intRR = 0;
float preFL = 0, preFR = 0, preRL = 0, preRR = 0;

bool driversEnabled = false;

WebServer server(80);

// ================================================================
//  ENCODER ISRs
//  IRAM_ATTR: forces function into fast IRAM so interrupt
//  executes reliably even during WiFi/flash operations.
//  Counting CHANGE (both edges) doubles encoder resolution.
// ================================================================
void IRAM_ATTR isrFL() { ticksFL++; }
void IRAM_ATTR isrFR() { ticksFR++; }
void IRAM_ATTR isrRL() { ticksRL++; }
void IRAM_ATTR isrRR() { ticksRR++; }

// ================================================================
//  STANDBY CONTROL
// ================================================================
void enableDrivers() {
  digitalWrite(STBY_FRONT, HIGH);
  digitalWrite(STBY_REAR,  HIGH);
  driversEnabled = true;
}

void disableDrivers() {
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW); ledcWrite(FL_PWM, 0);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW); ledcWrite(FR_PWM, 0);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW); ledcWrite(RL_PWM, 0);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW); ledcWrite(RR_PWM, 0);
  digitalWrite(STBY_FRONT, LOW);
  digitalWrite(STBY_REAR,  LOW);
  driversEnabled = false;
}

// ================================================================
//  RAW MOTOR WRITE — applies signed PWM directly to H-bridge
// ================================================================
void writeMotor(int in1, int in2, int pwmPin, float speed, int dir) {
  int s = constrain((int)(speed * dir), -255, 255);
  if (s > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    ledcWrite(pwmPin, s);
  } else if (s < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    ledcWrite(pwmPin, -s);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    ledcWrite(pwmPin, 0);
  }
}

// ================================================================
//  PID COMPUTE — positional form, one call per motor per interval
//
//  target  : desired ticks/second
//  actual  : measured ticks/second
//  integral: running sum of error × dt (modified in-place)
//  prevErr : error from last call (modified in-place)
//  dt      : time step in seconds
//
//  Anti-windup: integral is clamped so the I-term alone
//  can never exceed full PWM range. Prevents runaway
//  accumulation when motor is stalled or at limits.
// ================================================================
float computePID(float target, float actual,
                 float &integral, float &prevErr, float dt) {
  if (target == 0.0f) {          // target zero — reset state cleanly
    integral = 0.0f;
    prevErr  = 0.0f;
    return 0.0f;
  }

  float error   = target - actual;
  integral     += error * dt;

  float awLimit = (Ki > 0.001f) ? (255.0f / Ki) : 1000.0f;
  integral      = constrain(integral, -awLimit, awLimit);

  float deriv   = (dt > 0.0f) ? (error - prevErr) / dt : 0.0f;
  prevErr       = error;

  return constrain(Kp * error + Ki * integral + Kd * deriv, -255.0f, 255.0f);
}

// ================================================================
//  PID UPDATE — called every PID_INTERVAL_MS from loop()
//
//  1. Atomically snapshot encoder tick counters
//  2. Compute ticks since last call → actual speed (ticks/sec)
//  3. Sign actual speed from current PWM direction
//  4. Run PID for each motor → new PWM value
//  5. Write PWM to drivers
// ================================================================
void updatePID() {
  if (!driversEnabled) return;

  float dt = PID_INTERVAL_MS / 1000.0f;

  // Atomic snapshot — prevents half-updated reads
  noInterrupts();
  long nowFL = ticksFL, nowFR = ticksFR;
  long nowRL = ticksRL, nowRR = ticksRR;
  interrupts();

  long dFL = nowFL - prevTicksFL;  prevTicksFL = nowFL;
  long dFR = nowFR - prevTicksFR;  prevTicksFR = nowFR;
  long dRL = nowRL - prevTicksRL;  prevTicksRL = nowRL;
  long dRR = nowRR - prevTicksRR;  prevTicksRR = nowRR;

  // Signed actual speed: magnitude from encoder, sign from PWM direction
  actualFL = (pwmFL >= 0 ? 1.0f : -1.0f) * (dFL / dt);
  actualFR = (pwmFR >= 0 ? 1.0f : -1.0f) * (dFR / dt);
  actualRL = (pwmRL >= 0 ? 1.0f : -1.0f) * (dRL / dt);
  actualRR = (pwmRR >= 0 ? 1.0f : -1.0f) * (dRR / dt);

  pwmFL = computePID(targetFL, actualFL, intFL, preFL, dt);
  pwmFR = computePID(targetFR, actualFR, intFR, preFR, dt);
  pwmRL = computePID(targetRL, actualRL, intRL, preRL, dt);
  pwmRR = computePID(targetRR, actualRR, intRR, preRR, dt);

  writeMotor(FL_IN1, FL_IN2, FL_PWM, pwmFL, DIR_FL);
  writeMotor(FR_IN1, FR_IN2, FR_PWM, pwmFR, DIR_FR);
  writeMotor(RL_IN1, RL_IN2, RL_PWM, pwmRL, DIR_RL);
  writeMotor(RR_IN1, RR_IN2, RR_PWM, pwmRR, DIR_RR);
}

// ================================================================
//  DIFFERENTIAL DRIVE MIXING
//
//  linear  : -255 to +255  (+ve = forward, -ve = backward)
//  angular : -255 to +255  (-ve = turn left, +ve = turn right)
//
//  Left  motors  = linear - angular
//  Right motors  = linear + angular
//
//  If either side exceeds 255, both are scaled down proportionally
//  so the turn ratio is preserved at maximum speed.
//
//  Examples at speed 200:
//    Forward:        linear=200, angular=0   → L=200  R=200
//    Turn left:      linear=0,   angular=200 → L=-200 R=200
//    Fwd+left curve: linear=200, angular=100 → L=100  R=255 (scaled)
//    Backward:       linear=-200,angular=0   → L=-200 R=-200
// ================================================================
void setDrive(float linear, float angular) {
  float left  = linear - angular;
  float right = linear + angular;

  // Scale both proportionally if either exceeds max
  float maxMag = max(abs(left), abs(right));
  if (maxMag > 255.0f) {
    float scale = 255.0f / maxMag;
    left  *= scale;
    right *= scale;
  }

  if (linear == 0.0f && angular == 0.0f) {
    stopAll();
    return;
  }

  // Convert PWM fraction to ticks/second target
  targetFL = (left  / 255.0f) * MAX_SPEED_TICKS;
  targetFR = (right / 255.0f) * MAX_SPEED_TICKS;
  targetRL = (left  / 255.0f) * MAX_SPEED_TICKS;
  targetRR = (right / 255.0f) * MAX_SPEED_TICKS;

  enableDrivers();
}

void stopAll() {
  targetFL = targetFR = targetRL = targetRR = 0.0f;
  pwmFL    = pwmFR    = pwmRL    = pwmRR    = 0.0f;
  intFL    = intFR    = intRL    = intRR    = 0.0f;
  preFL    = preFR    = preRL    = preRR    = 0.0f;
  disableDrivers();
}

// ================================================================
//  CALIBRATION ROUTINE — run via Serial command 'M'
//  Drives RL motor at full speed for 3 seconds, prints ticks/sec.
//  Copy the printed value into MAX_SPEED_TICKS above.
// ================================================================
void runCalibration() {
  Serial.println("=== CALIBRATION: RL motor at full speed for 3s ===");
  Serial.println("Make sure wheels are off the ground!");
  delay(2000);

  noInterrupts();
  ticksRL = 0;
  interrupts();
  prevTicksRL = 0;

  enableDrivers();
  writeMotor(RL_IN1, RL_IN2, RL_PWM, 255, DIR_RL);

  unsigned long start = millis();
  long totalTicks = 0;
  int samples = 0;

  while (millis() - start < 3000) {
    delay(500);
    noInterrupts();
    long now = ticksRL;
    interrupts();
    long delta = now - prevTicksRL;
    prevTicksRL = now;
    float ticksPerSec = delta / 0.5f;
    Serial.print("  ticks/sec sample: ");
    Serial.println(ticksPerSec);
    totalTicks += delta;
    samples++;
  }

  writeMotor(RL_IN1, RL_IN2, RL_PWM, 0, DIR_RL);
  disableDrivers();

  float avg = (totalTicks / 3.0f);
  Serial.print(">>> Average ticks/sec at PWM=255: ");
  Serial.println(avg);
  Serial.println(">>> Set MAX_SPEED_TICKS to this value and re-upload.");
}

// ================================================================
//  HTML WEB INTERFACE
//  Embedded as raw string — stored in flash
// ================================================================
const char HTML_PAGE[] = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Rover</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#12122b;color:#ddd;font-family:sans-serif;padding:12px;-webkit-user-select:none;user-select:none}
h2{text-align:center;color:#e94560;margin-bottom:14px;font-size:22px;letter-spacing:1px}
.card{background:#1a1a3e;border-radius:14px;padding:14px;margin-bottom:12px;border:1px solid #2a2a5e}
.card h3{font-size:11px;color:#556;text-transform:uppercase;letter-spacing:2px;margin-bottom:10px}
.dpad{display:grid;grid-template-columns:repeat(3,80px);grid-template-rows:repeat(3,80px);gap:8px;margin:0 auto;width:fit-content}
.btn{background:#1e2060;border:2px solid #3a3a8e;color:#ccd;font-size:30px;border-radius:12px;cursor:pointer;width:80px;height:80px;-webkit-tap-highlight-color:transparent;transition:background .1s}
.btn:active,.btn.act{background:#e94560;border-color:#ff6080}
.stopbtn{background:#8b0000;border-color:#cc2020;font-size:13px;font-weight:bold;letter-spacing:1px;color:#fff}
.stopbtn:active{background:#ff2020}
.empty{width:80px;height:80px;visibility:hidden}
.sliderRow{display:flex;align-items:center;gap:10px;margin:4px 0}
.sliderRow span{font-size:13px;min-width:60px}
.sliderRow b{font-size:14px;min-width:28px;text-align:right;color:#e94560}
input[type=range]{flex:1;accent-color:#e94560;height:6px}
.tbl{width:100%;font-family:monospace;font-size:12px;border-collapse:collapse}
.tbl th{color:#556;font-size:10px;text-transform:uppercase;padding:2px 6px;text-align:right}
.tbl th:first-child{text-align:left}
.tbl td{padding:3px 6px;text-align:right;color:#aef}
.tbl td:first-child{color:#eee;text-align:left}
.tbl tr:nth-child(even){background:#1e1e3a}
.pg{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:8px}
.pf label{font-size:11px;color:#667;display:block;margin-bottom:3px}
.pf input{width:100%;background:#12122b;border:1px solid #334;color:#eee;padding:6px;border-radius:8px;font-size:14px;text-align:center}
.abtn{width:100%;background:#e94560;border:none;color:#fff;padding:9px;border-radius:10px;cursor:pointer;font-size:14px;font-weight:bold;letter-spacing:1px}
.abtn:active{background:#c0304f}
.conn{display:flex;align-items:center;gap:6px;font-size:12px;color:#667;margin-top:6px}
.dot{width:8px;height:8px;border-radius:50%;background:#2ecc71}
.dot.off{background:#e74c3c}
</style>
</head>
<body>
<h2>&#x1F916; ESP32 Rover</h2>

<div class="card">
<h3>Speed</h3>
<div class="sliderRow">
  <span>Drive speed</span>
  <input type="range" id="spd" min="60" max="255" value="180"
    oninput="document.getElementById('sv').textContent=this.value">
  <b id="sv">180</b>
</div>
<div class="sliderRow">
  <span>Turn speed</span>
  <input type="range" id="tspd" min="40" max="255" value="160"
    oninput="document.getElementById('tv').textContent=this.value">
  <b id="tv">160</b>
</div>
</div>

<div class="card">
<h3>Drive Control</h3>
<div class="dpad">
  <div class="empty"></div>
  <button class="btn" id="bF"
    ontouchstart="go(1,0,this)" ontouchend="halt(this)"
    onmousedown="go(1,0,this)" onmouseup="halt(this)" onmouseleave="halt(this)">&#9650;</button>
  <div class="empty"></div>

  <button class="btn" id="bFL"
    ontouchstart="go(1,-1,this)" ontouchend="halt(this)"
    onmousedown="go(1,-1,this)" onmouseup="halt(this)" onmouseleave="halt(this)">&#8598;</button>
  <button class="btn stopbtn" id="bS"
    ontouchstart="halt(this)" onmousedown="halt(this)">STOP</button>
  <button class="btn" id="bFR"
    ontouchstart="go(1,1,this)" ontouchend="halt(this)"
    onmousedown="go(1,1,this)" onmouseup="halt(this)" onmouseleave="halt(this)">&#8599;</button>

  <button class="btn" id="bL"
    ontouchstart="go(0,-1,this)" ontouchend="halt(this)"
    onmousedown="go(0,-1,this)" onmouseup="halt(this)" onmouseleave="halt(this)">&#9664;</button>
  <button class="btn" id="bB"
    ontouchstart="go(-1,0,this)" ontouchend="halt(this)"
    onmousedown="go(-1,0,this)" onmouseup="halt(this)" onmouseleave="halt(this)">&#9660;</button>
  <button class="btn" id="bR"
    ontouchstart="go(0,1,this)" ontouchend="halt(this)"
    onmousedown="go(0,1,this)" onmouseup="halt(this)" onmouseleave="halt(this)">&#9654;</button>
</div>
</div>

<div class="card">
<h3>Live Telemetry</h3>
<table class="tbl">
<tr><th>Motor</th><th>Target t/s</th><th>Actual t/s</th><th>PWM</th></tr>
<tr><td>FL</td><td id="tFL">-</td><td id="aFL">-</td><td id="pFL">-</td></tr>
<tr><td>FR</td><td id="tFR">-</td><td id="aFR">-</td><td id="pFR">-</td></tr>
<tr><td>RL</td><td id="tRL">-</td><td id="aRL">-</td><td id="pRL">-</td></tr>
<tr><td>RR</td><td id="tRR">-</td><td id="aRR">-</td><td id="pRR">-</td></tr>
</table>
<div class="conn"><div class="dot" id="connDot"></div><span id="connTxt">Connecting...</span></div>
</div>

<div class="card">
<h3>PID Tuning — Live</h3>
<div class="pg">
  <div class="pf"><label>Kp (response)</label>
    <input type="number" id="kp" value="2.0" step="0.1" min="0"></div>
  <div class="pf"><label>Ki (steady error)</label>
    <input type="number" id="ki" value="0.8" step="0.1" min="0"></div>
  <div class="pf"><label>Kd (damping)</label>
    <input type="number" id="kd" value="0.05" step="0.01" min="0"></div>
</div>
<button class="abtn" id="applyBtn" onclick="applyPID()">Apply PID Gains</button>
</div>

<script>
let iv = null;
const spd  = () => parseInt(document.getElementById('spd').value);
const tspd = () => parseInt(document.getElementById('tspd').value);

function go(lin, ang, btn) {
  clearInterval(iv);
  const s = spd(), t = tspd();
  const L = Math.round(lin * s - ang * t);
  const A = Math.round(ang * t);
  const send = () => fetch('/drive?linear=' + L + '&angular=' + A).catch(() => {});
  send();
  iv = setInterval(send, 150);
}

function halt(btn) {
  clearInterval(iv);
  iv = null;
  fetch('/stop').catch(() => {});
}

function applyPID() {
  const btn = document.getElementById('applyBtn');
  fetch('/pid?kp=' + document.getElementById('kp').value +
              '&ki=' + document.getElementById('ki').value +
              '&kd=' + document.getElementById('kd').value)
    .then(() => { btn.textContent = 'Applied ✓'; setTimeout(() => btn.textContent = 'Apply PID Gains', 1500); })
    .catch(() => {});
}

function f(v) { return (v === undefined ? '-' : Math.round(v)); }

setInterval(() => {
  fetch('/status')
    .then(r => r.json())
    .then(d => {
      document.getElementById('tFL').textContent = f(d.tFL);
      document.getElementById('aFL').textContent = f(d.aFL);
      document.getElementById('pFL').textContent = f(d.pFL);
      document.getElementById('tFR').textContent = f(d.tFR);
      document.getElementById('aFR').textContent = f(d.aFR);
      document.getElementById('pFR').textContent = f(d.pFR);
      document.getElementById('tRL').textContent = f(d.tRL);
      document.getElementById('aRL').textContent = f(d.aRL);
      document.getElementById('pRL').textContent = f(d.pRL);
      document.getElementById('tRR').textContent = f(d.tRR);
      document.getElementById('aRR').textContent = f(d.aRR);
      document.getElementById('pRR').textContent = f(d.pRR);
      const kpEl = document.getElementById('kp');
      const kiEl = document.getElementById('ki');
      const kdEl = document.getElementById('kd');
      if (document.activeElement !== kpEl) kpEl.value = d.kp;
      if (document.activeElement !== kiEl) kiEl.value = d.ki;
      if (document.activeElement !== kdEl) kdEl.value = d.kd;
      document.getElementById('connDot').className = 'dot';
      document.getElementById('connTxt').textContent = 'Connected — ' + d.ip;
    })
    .catch(() => {
      document.getElementById('connDot').className = 'dot off';
      document.getElementById('connTxt').textContent = 'Connection lost';
    });
}, 400);
</script>
</body>
</html>
)rawhtml";

// ================================================================
//  WEB SERVER HANDLERS
// ================================================================
void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

void handleDrive() {
  float lin = server.hasArg("linear")  ? server.arg("linear").toFloat()  : 0;
  float ang = server.hasArg("angular") ? server.arg("angular").toFloat() : 0;
  setDrive(constrain(lin, -255, 255), constrain(ang, -255, 255));
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopAll();
  server.send(200, "text/plain", "Stopped");
}

void handlePID() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  intFL = intFR = intRL = intRR = 0.0f;
  preFL = preFR = preRL = preRR = 0.0f;
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  char json[512];
  snprintf(json, sizeof(json),
    "{\"tFL\":%.1f,\"aFL\":%.1f,\"pFL\":%.0f,"
     "\"tFR\":%.1f,\"aFR\":%.1f,\"pFR\":%.0f,"
     "\"tRL\":%.1f,\"aRL\":%.1f,\"pRL\":%.0f,"
     "\"tRR\":%.1f,\"aRR\":%.1f,\"pRR\":%.0f,"
     "\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.3f,"
     "\"ip\":\"%s\"}",
    targetFL, actualFL, pwmFL,
    targetFR, actualFR, pwmFR,
    targetRL, actualRL, pwmRL,
    targetRR, actualRR, pwmRR,
    Kp, Ki, Kd,
    WiFi.localIP().toString().c_str());
  server.send(200, "application/json", json);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 Rover — WiFi + PID Mode");

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  pinMode(STBY_FRONT, OUTPUT); digitalWrite(STBY_FRONT, LOW);
  pinMode(STBY_REAR,  OUTPUT); digitalWrite(STBY_REAR,  LOW);

  pinMode(FL_ENC_A, INPUT);        pinMode(FL_ENC_B, INPUT);
  pinMode(FR_ENC_A, INPUT);        pinMode(FR_ENC_B, INPUT);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR, CHANGE);

  ledcAttach(FL_PWM, 1000, 8); ledcAttach(FR_PWM, 1000, 8);
  ledcAttach(RL_PWM, 1000, 8); ledcAttach(RR_PWM, 1000, 8);
  ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0);
  ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);

  Serial.print("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    if (++attempts > 30) {
      Serial.println("\nWiFi failed — restarting");
      ESP.restart();
    }
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Open browser: http://");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    stopAll();
    Serial.println("OTA upload starting...");
  });
  ArduinoOTA.onEnd([]()   { Serial.println("\nOTA done. Rebooting..."); });
  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
    Serial.printf("OTA: %u%%\r", prog * 100 / total);
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("OTA error [%u]\n", err);
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready. Subsequent uploads: Tools > Port > Network Ports");

  server.on("/",       HTTP_GET, handleRoot);
  server.on("/drive",  HTTP_GET, handleDrive);
  server.on("/stop",   HTTP_GET, handleStop);
  server.on("/pid",    HTTP_GET, handlePID);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
  Serial.println("Web server started.");
  Serial.println("Serial commands: M=calibrate  S=stop");
}

// ================================================================
//  LOOP — three things run here, all non-blocking
// ================================================================
void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  static unsigned long lastPID = 0;
  unsigned long now = millis();
  if (now - lastPID >= PID_INTERVAL_MS) {
    lastPID = now;
    updatePID();
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'M' || c == 'm') runCalibration();
    if (c == 'S' || c == 's') { stopAll(); Serial.println("Stopped."); }
  }
}
