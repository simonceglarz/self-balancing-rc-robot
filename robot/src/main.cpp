#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>

// =================== Pin map ===================
// GPIO19 = SDA, GPIO18 = SCL (confirmed against physical wiring).
#define SDA_PIN 19
#define SCL_PIN 18

// Motor driver — AIN1/AIN2 control Motor A, BIN1/BIN2 control Motor B.
#define AIN1 26
#define AIN2 32
#define BIN1 25
#define BIN2 33

// NOTE: arduino-esp32 core 3.0+ removed manual channel numbers —
// ledcAttach() below assigns a channel automatically per pin.

// Encoders — A-channel pins are the interrupt pins.
// GPIO 34/35 are input-only (no internal pull-up) — fine here,
// encoders are pure inputs. Can't be used for motor drive.
#define ENC_A1 27   // Motor A channel A
#define ENC_B1 14   // Motor A channel B
#define ENC_A2 34   // Motor B channel A
#define ENC_B2 35   // Motor B channel B

#define ENABLE_BATTERY_MONITOR 1
#if ENABLE_BATTERY_MONITOR
#define VBAT_PIN 36   // ADC1 — required, ADC2 dies when WiFi is active
const float VBAT_DIVIDER_RATIO = (100000.0 + 47000.0) / 47000.0; // 3.128
const float VBAT_CUTOFF = 6.6;
float lastVbat = 0.0;   // updated once/sec in loop(), read by the OLED block
#endif

// =================== Direction flags ===================
// Unknowns per project notes: mirror-mounted motors likely need one
// side's wires (or this flag) flipped; encoder sign is unverified
// until wheels actually turn. Flip these booleans after your first
// power-on test rather than rewiring or rewriting logic.
const bool INVERT_MOTOR_A = false;
const bool INVERT_MOTOR_B = true;   // confirmed via motor/encoder test — wheels now spin correctly
const bool INVERT_ENC_A   = false;
const bool INVERT_ENC_B   = false;

// Set to 1, flash, watch which physical wheel moves during each phase.
// Set back to 0 once you've identified A vs B and set the INVERT flags
// above — then reflash for real balancing.
#define RUN_MOTOR_ENCODER_TEST 0

// Set to 1 for a dead-simple sanity check: both motors spin together
// whenever the IMU detects real rotation, no matter which way the
// board is physically mounted. Good first test when you're not sure
// if motors OR the gyro are the actual problem.
#define RUN_BASIC_MOTOR_GYRO_TEST 0

// Set to 1 to run a bare I2C bus scan instead of normal setup — useful
// for diagnosing wiring without touching lib_deps/includes at all,
// since this reuses the exact dependency set that already builds fine.
#define RUN_I2C_SCANNER 0

#if RUN_I2C_SCANNER
void runI2CScanner() {
    Serial.println("I2C Scanner starting...");
    while (1) {
        byte count = 0;
        for (byte addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            byte error = Wire.endTransmission();
            if (error == 0) {
                Serial.print("Device found at address 0x");
                if (addr < 16) Serial.print("0");
                Serial.println(addr, HEX);
                count++;
            }
        }
        if (count == 0) Serial.println("No I2C devices found");
        else { Serial.print(count); Serial.println(" device(s) found"); }
        delay(3000);
    }
}
#endif

// =================== Objects ===================
Adafruit_LSM6DSOX sox;                          // IMU driver instance — talks to the
                                                 // physical sensor over I2C once Wire.begin() runs
Adafruit_SSD1306 display(128, 64, &Wire, -1);   // OLED driver — 128x64 px, shares the I2C bus
                                                 // with the IMU (-1 = no dedicated reset pin wired)

// Network name/password the ESP32 broadcasts. Since we're in AP mode,
// the ESP32 *is* the WiFi network — your phone connects to "BalanceBot"
// directly rather than both devices joining your home router.
const char* AP_SSID = "BalanceBot";
const char* AP_PASS = "tuning123";   // WPA2 requires 8+ characters minimum

// ESP-NOW (the physical joystick controller) and the AP (the tuning
// webpage) share the same radio, so they must agree on a channel — the
// controller firmware hardcodes this same value. If ESP-NOW packets
// silently never arrive, mismatched channel is the first thing to check.
const int WIFI_CHANNEL = 1;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Must match the controller's JoyPacket struct byte-for-byte — ESP-NOW
// has no schema, it just copies raw bytes.
typedef struct {
    float x;   // -1 (left) .. +1 (right)
    float y;   // -1 (back) .. +1 (forward)
} JoyPacket;

// Shared with the webpage's 'J' WebSocket handler below, so both input
// sources command the same maximum lean/turn.
const float JOY_MAX_TRIM_DEG = 3.0;   // matches old FWD/BACK magnitude
const float JOY_MAX_STEER    = 30.0;  // matches old L/R magnitude

// =================== Live-tunable gains (WiFi) ===================
volatile float Kp = 38.0;
volatile float Ki = 0.7;
volatile float Kd = 0.4;
volatile float targetAngleTrim = 0.0;   // "goal" value set instantly by F/B commands —
                                         // the loop ramps toward this via
                                         // targetAngleTrimSmoothed rather than using
                                         // it directly, to avoid a derivative-kick step.
volatile float steerBias = 0.0;         // "goal" value set instantly by L/R commands —
                                         // same ramping treatment via steerBiasSmoothed.

// =================== Outer loop gains (WiFi-tunable via V/W) ===================
volatile float Kp_vel = 0.12;
volatile float Ki_vel = 0.003;

// =================== Filter / control state ===================
const float alpha = 0.98;

// Corrects for the IMU's zero-angle not lining up with the robot's true
// mechanical balance point (from mounting tilt, CoG offset, or both).
// Determine by trial and error: nudge this value up/down and re-test
// forward vs backward reaction thresholds until they're symmetric.
// Sign convention: if forward tilts react at SMALLER angles than
// backward tilts (as observed), try a small POSITIVE value first.
volatile float ANGLE_TRIM_DEG = -2.2;  // Tune live via the WiFi slider.
                             // The outer velocity loop can mask a wide
                             // range of trim error by continuously
                             // creeping to hold position, so "it balances"
                             // isn't proof a given value is correct —
                             // watch for steady creep in one direction
                             // at rest as the real symmetry check.

float angle = 0.0;
float lastAngle = 0.0;      // for derivative-on-measurement — see loop()
float gyroBias = 0.0;
float angleErrorIntegral = 0.0;
const float INTEGRAL_CLAMP = 100.0;
float velIntegral = 0.0;
const int MAX_OUTPUT = 255;

// Per-motor stiction floors — NOT assumed equal. Testing found motor A's
// physical wheel needs more duty to start turning than motor B's at all
// (at MIN=147, only B moved) — a single shared floor can't be correct for
// both without either stalling A or overdriving B. Tune live via WiFi.
volatile int MIN_OUTPUT_A = 146;
volatile int MIN_OUTPUT_B = 146;

// Per-motor output scale, applied across the WHOLE output range (not just
// near the floor) — corrects one motor consistently out-turning the other
// even away from the stiction region (observed: countB always ends up
// higher than countA over a run). Start both at 1.0 and nudge the FASTER
// motor's scale down while watching countA/-countB converge on the OLED.
volatile float MOTOR_A_SCALE = 1.0;
volatile float MOTOR_B_SCALE = 1.0;

const float FALL_CUTOFF_DEG = 45.0;

// Exponential-moving-average state for the encoder-derived velocity —
// see avgVelocityFiltered below. 0.1 is a starting guess, not a derived
// value: raise it (closer to 1.0) for less smoothing/more responsiveness,
// lower it for more smoothing if the outer loop still looks jumpy.
const float VEL_FILTER_ALPHA = 0.1;

// Max age, in seconds, that a single loop iteration's dt is allowed to
// have before being clamped. Without this, a loop iteration delayed by
// WiFi/WebSocket servicing (a genuinely occasional ESP32 async-server
// hiccup) produces one abnormally large dt, which inflates the integral
// term (error * dt) and shrinks the derivative denominator in the same
// tick — a plausible cause of the brief "controls give out" moments.
const float MAX_DT = 0.02;   // 20ms — 2x the normal ~10ms tick

// How fast targetAngleTrim/steerBias ramp toward their commanded value
// per second, instead of jumping instantly. Prevents the FWD/BACK/LEFT/
// RIGHT buttons from causing a one-tick derivative-kick-style spike and
// the overshoot/tipping that comes with it. Starting guesses — raise for
// snappier response, lower if it still feels like it overshoots.
const float TRIM_SLEW_PER_SEC  = 12.0;  // deg/sec
const float STEER_SLEW_PER_SEC = 80.0;  // units/sec (same units as steerBias)

// =================== Encoder state ===================
volatile long countA = 0;
volatile long countB = 0;
long lastCountA = 0, lastCountB = 0;

void IRAM_ATTR encA_ISR() {
    bool state = digitalRead(ENC_A1) == digitalRead(ENC_B1);
    if (INVERT_ENC_A) state = !state;
    if (state) countA++; else countA--;
}
void IRAM_ATTR encB_ISR() {
    bool state = digitalRead(ENC_A2) == digitalRead(ENC_B2);
    if (INVERT_ENC_B) state = !state;
    if (state) countB++; else countB--;
}

// =================== Motor drive ===================
void motorA(int speed) {
    if (INVERT_MOTOR_A) speed = -speed;
    speed = constrain(speed, -255, 255);
    if (speed >= 0) { ledcWrite(AIN1, speed); ledcWrite(AIN2, 0); }
    else            { ledcWrite(AIN1, 0); ledcWrite(AIN2, -speed); }
}
void motorB(int speed) {
    if (INVERT_MOTOR_B) speed = -speed;
    speed = constrain(speed, -255, 255);
    if (speed >= 0) { ledcWrite(BIN1, speed); ledcWrite(BIN2, 0); }
    else            { ledcWrite(BIN1, 0); ledcWrite(BIN2, -speed); }
}

// =================== Motor/Encoder identification test ===================
// Defined HERE (after motorA/motorB/countA/countB), not earlier — it
// references all of them. The actual CALL that runs this lives in
// setup(), further down.
#if RUN_MOTOR_ENCODER_TEST
void runMotorEncoderTest() {
    Serial.println("=== MOTOR/ENCODER TEST ===");
    Serial.println("Watch which physical wheel moves in each phase.");
    delay(2000);

    Serial.println("Testing Motor A (forward command)...");
    motorA(150);
    for (int i = 0; i < 15; i++) {
        Serial.print("  countA: "); Serial.println(countA);
        delay(100);
    }
    motorA(0);
    delay(1000);

    Serial.println("Testing Motor B (forward command)...");
    motorB(150);
    for (int i = 0; i < 15; i++) {
        Serial.print("  countB: "); Serial.println(countB);
        delay(100);
    }
    motorB(0);

    Serial.println("=== TEST COMPLETE ===");
    Serial.println("Whichever wheel moved during 'Motor A' phase IS Motor A.");
    Serial.println("If countA/countB didn't increase during its own phase,");
    Serial.println("that encoder's INVERT flag or wiring needs checking.");
    while (1) delay(1000);  // halt here — never falls through to balancing
}
#endif

// =================== Basic motor + gyro sanity test ===================
// Orientation-independent by design: uses the MAGNITUDE of the full
// 3-axis rotation vector, not a single axis. Rotating a physical
// object has a total "how fast" that doesn't change based on which
// way you're holding it — only which axis that rotation happens to
// load onto changes. So this test can't be fooled by the IMU being
// mounted sideways, upside down, or at any odd angle — real rotation
// shows up as a magnitude spike regardless of orientation.
#if RUN_BASIC_MOTOR_GYRO_TEST
void runBasicMotorGyroTest() {
    Serial.println("=== BASIC MOTOR + GYRO TEST ===");
    Serial.println("Tilt or rotate the IMU in ANY direction.");
    Serial.println("Both motors should spin together the moment real");
    Serial.println("rotation is detected — orientation doesn't matter.");

    // Retry instead of halting — lets you fix a loose I2C wire live,
    // without needing to reflash, since the board stays powered and
    // just keeps trying every second until it succeeds.
    bool imuOk = false;
    while (!imuOk) {
        imuOk = sox.begin_I2C();
        if (!imuOk) {
            Serial.println("IMU not found, retrying in 1s...");
            delay(1000);
        }
    }
    Serial.println("IMU found!");
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

    ledcAttach(AIN1, 20000, 8);
    ledcAttach(AIN2, 20000, 8);
    ledcAttach(BIN1, 20000, 8);
    ledcAttach(BIN2, 20000, 8);
    motorA(0); motorB(0);

    const float SPIN_THRESHOLD_DPS = 20.0;  // above typical at-rest noise
    const int TEST_SPEED = 150;

    while (1) {
        sensors_event_t accel, gyro, temp;
        sox.getEvent(&accel, &gyro, &temp);

        float gx = gyro.gyro.x * 180.0 / PI;
        float gy = gyro.gyro.y * 180.0 / PI;
        float gz = gyro.gyro.z * 180.0 / PI;

        // Vector magnitude — orientation-independent, see note above.
        float magnitude = sqrt(gx * gx + gy * gy + gz * gz);

        Serial.print("Rotation rate: ");
        Serial.print(magnitude, 1);
        Serial.print(" deg/s   ");

        if (magnitude > SPIN_THRESHOLD_DPS) {
            Serial.println("-> MOTORS ON (both same direction)");
            motorA(TEST_SPEED);
            motorB(TEST_SPEED);
        } else {
            Serial.println("-> motors off");
            motorA(0);
            motorB(0);
        }

        delay(100);
    }
}
#endif
// Needs targetAngleTrim/steerBias/JoyPacket/JOY_MAX_* declared above —
// all of them are, so its exact position in the file doesn't matter
// beyond that.
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    // Signature takes esp_now_recv_info_t* (not a bare MAC pointer) on
    // arduino-esp32 core 3.0.3, which this project is pinned to — an
    // older core (2.x) used a different callback signature and this
    // won't compile against it unchanged.
    if (len != sizeof(JoyPacket)) return;
    JoyPacket packet;
    memcpy(&packet, incomingData, sizeof(packet));
    targetAngleTrim = packet.y * JOY_MAX_TRIM_DEG;
    steerBias       = -packet.x * JOY_MAX_STEER;   // sign matches existing L/R convention
}

// Small "artificial horizon" style circle: a line through the center
// rotates with the robot's tilt angle, giving an at-a-glance visual
// independent of reading the numeric angle value next to it.
void drawTiltIndicator(int cx, int cy, int r, float angleDeg) {
    // Clamp only for the indicator's own geometry — doesn't touch the
    // real angle value used anywhere else.
    float clamped = constrain(angleDeg, -45.0, 45.0);
    float rad = clamped * PI / 180.0;

    display.drawCircle(cx, cy, r, SSD1306_WHITE);

    int dx = (int)(r * cos(rad));
    int dy = (int)(r * sin(rad));
    display.drawLine(cx - dx, cy - dy, cx + dx, cy + dy, SSD1306_WHITE);

    // Fixed center tick — represents the robot's own body axis, so the
    // rotating line's angle relative to this mark shows the tilt.
    display.drawFastVLine(cx, cy - 2, 5, SSD1306_WHITE);
}

// =================== WiFi tuning page ===================
// NOTE: the value="..." attributes and the initial <span> text below
// are just the PAGE'S OWN static fallback (what shows for an instant
// before the WebSocket connects). They do NOT drive the control loop
// and don't need to be kept in sync with the real Kp/Ki/Kd/etc. — the
// 'Y:' sync message sent from onWsEvent()'s WS_EVT_CONNECT handler
// overwrites every slider with the LIVE firmware values as soon as the
// page connects. This is what fixes sliders showing a stale number
// (e.g. 15) after you change a gain in code and reflash.
//
// Touch handling: the meta viewport below disables pinch/double-tap
// zoom (maximum-scale=1, user-scalable=no), and every control button
// has -webkit-user-select/-webkit-touch-callout/user-select:none plus
// touch-action:manipulation, so tapping and holding them doesn't
// trigger the browser's text-selection highlight or zoom gesture. The
// inline handlers also call event.preventDefault() for the same reason
// — some mobile browsers still show a selection/callout on touchstart
// without it even with the CSS in place.
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<style>
body{font-family:sans-serif;text-align:center;background:#111;color:#eee;
     -webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
input[type=range]{width:60%;vertical-align:middle}
button{width:70px;height:70px;font-size:20px;margin:4px;
       -webkit-user-select:none;user-select:none;-webkit-touch-callout:none;
       touch-action:manipulation;-webkit-tap-highlight-color:transparent}
.nudge{width:38px;height:38px;font-size:18px;margin:0 4px;padding:0;vertical-align:middle}
.row{display:flex;align-items:center;justify-content:center}
#tel{font-size:14px;color:#8f8}
h3{margin:14px 0 2px 0;font-size:14px;color:#aaf}
#joyBase{width:220px;height:220px;border-radius:50%;background:#222;border:2px solid #556;
  margin:16px auto;position:relative;touch-action:none;-webkit-user-select:none;user-select:none}
#joyHandle{width:90px;height:90px;border-radius:50%;background:#8af;position:absolute;
  left:65px;top:65px;pointer-events:none}
#stopBtn{width:70px;height:44px;border-radius:8px;font-size:16px;margin:6px auto;display:block}
</style></head><body>
<h2>Balance Bot Tuning</h2>
<p id="tel">angle: -- &nbsp; out: --</p>

<h3>Inner loop (angle) — tune AFTER trim/floor below are symmetric</h3>
<p>Kp <span id="pv">15.0</span></p>
<div class="row"><button class="nudge" onclick="nudge('p','pv','P',-0.1,1)">-</button><input id="p" type="range" min="0" max="50" step="0.1" value="15"><button class="nudge" onclick="nudge('p','pv','P',0.1,1)">+</button></div>
<p>Ki <span id="iv">0.5</span></p>
<div class="row"><button class="nudge" onclick="nudge('i','iv','I',-0.05,2)">-</button><input id="i" type="range" min="0" max="5" step="0.05" value="0.5"><button class="nudge" onclick="nudge('i','iv','I',0.05,2)">+</button></div>
<p>Kd <span id="dv">0.3</span></p>
<div class="row"><button class="nudge" onclick="nudge('d','dv','D',-0.05,2)">-</button><input id="d" type="range" min="0" max="5" step="0.05" value="0.3"><button class="nudge" onclick="nudge('d','dv','D',0.05,2)">+</button></div>

<h3>Outer loop (velocity/drift) — tune LAST</h3>
<p>Kp_vel <span id="kvv">0.015</span></p>
<div class="row"><button class="nudge" onclick="nudge('kv','kvv','V',-0.005,3)">-</button><input id="kv" type="range" min="0" max="0.3" step="0.005" value="0.015"><button class="nudge" onclick="nudge('kv','kvv','V',0.005,3)">+</button></div>
<p>Ki_vel <span id="kiv">0.002</span></p>
<div class="row"><button class="nudge" onclick="nudge('ki2','kiv','W',-0.001,4)">-</button><input id="ki2" type="range" min="0" max="0.05" step="0.001" value="0.002"><button class="nudge" onclick="nudge('ki2','kiv','W',0.001,4)">+</button></div>

<h3>Calibration — tune FIRST, before anything above</h3>
<p>Angle Trim <span id="tv">-2.2</span> deg</p>
<div class="row"><button class="nudge" onclick="nudge('t','tv','X',-0.1,1)">-</button><input id="t" type="range" min="-15" max="15" step="0.1" value="-2.2"><button class="nudge" onclick="nudge('t','tv','X',0.1,1)">+</button></div>
<p>Min Output A <span id="mv">155</span></p>
<div class="row"><button class="nudge" onclick="nudge('m','mv','M',-1,0)">-</button><input id="m" type="range" min="0" max="255" step="1" value="155"><button class="nudge" onclick="nudge('m','mv','M',1,0)">+</button></div>

<h3>Motor balance — per-wheel trim (find floors first, then match speeds)</h3>
<p>Min Output B <span id="cv">147</span></p>
<div class="row"><button class="nudge" onclick="nudge('c','cv','C',-1,0)">-</button><input id="c" type="range" min="0" max="255" step="1" value="147"><button class="nudge" onclick="nudge('c','cv','C',1,0)">+</button></div>

<h3>Drive — continuous joystick</h3>
<div id="joyBase"><div id="joyHandle"></div></div>
<button id="stopBtn" onclick="send('S:1')">STOP</button>
<script>
// Joystick: drag within the base circle, snaps back to center and sends
// J:0.00:0.00 on release. Pointer Events (not separate touch/mouse
// handlers) so one code path covers phone touch AND desktop mouse.
// x: -1 (left) .. +1 (right). y: -1 (back) .. +1 (forward) — screen Y is
// inverted before sending so "up" on the pad means "forward", matching
// how a joystick is normally read.
(function(){
  var base = document.getElementById('joyBase');
  var handle = document.getElementById('joyHandle');
  var radius = 65; // px — how far the handle can travel from center
  var active = false;
  var lastSend = 0;

  function reset(){
    handle.style.transform = '';
    send('J:0.00:0.00');
  }
  function move(clientX, clientY){
    var rect = base.getBoundingClientRect();
    var dx = clientX - (rect.left + rect.width / 2);
    var dy = clientY - (rect.top + rect.height / 2);
    var dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > radius) { dx = dx * radius / dist; dy = dy * radius / dist; }
    handle.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
    var nx = dx / radius;
    var ny = -dy / radius;
    var now = Date.now();
    if (now - lastSend > 50) {   // throttle to ~20Hz, plenty for steering
      lastSend = now;
      send('J:' + nx.toFixed(2) + ':' + ny.toFixed(2));
    }
  }
  base.onpointerdown = function(e){
    e.preventDefault();
    active = true;
    base.setPointerCapture(e.pointerId);
    move(e.clientX, e.clientY);
  };
  base.onpointermove = function(e){
    if (!active) return;
    e.preventDefault();
    move(e.clientX, e.clientY);
  };
  function end(e){
    if (!active) return;
    active = false;
    reset();
  }
  base.onpointerup = end;
  base.onpointercancel = end;
})();
</script>
<script>
var ws = new WebSocket("ws://" + location.host + "/ws");
function send(msg){ ws.send(msg); }
function nudge(id, spanId, key, delta, decimals){
  var el = document.getElementById(id);
  var v = parseFloat(el.value) + delta;
  var lo = parseFloat(el.min), hi = parseFloat(el.max);
  if (v < lo) v = lo; if (v > hi) v = hi;
  v = parseFloat(v.toFixed(decimals));
  el.value = v;
  document.getElementById(spanId).innerText = v;
  send(key + ':' + v);
}
// Sets both the slider position and its label span from one value —
// used when the 'Y' sync message arrives right after connect, so the
// UI reflects what the firmware is ACTUALLY running rather than the
// hardcoded value="..." fallback baked into the HTML above.
function setSlider(id, spanId, val){
  document.getElementById(id).value = val;
  document.getElementById(spanId).innerText = val;
}
ws.onmessage = function(evt){
  var c = evt.data.charAt(0);
  if (c === 'T') {
    var parts = evt.data.split(':');
    document.getElementById('tel').innerText =
      'angle: ' + parts[1] + '   out: ' + parts[2];
  } else if (c === 'Y') {
    // One-time sync sent right after connect:
    // Y:Kp:Ki:Kd:Kp_vel:Ki_vel:trim:minOutputA:minOutputB:scaleA:scaleB
    var v = evt.data.split(':');
    setSlider('p','pv', v[1]);
    setSlider('i','iv', v[2]);
    setSlider('d','dv', v[3]);
    setSlider('kv','kvv', v[4]);
    setSlider('ki2','kiv', v[5]);
    setSlider('t','tv', v[6]);
    setSlider('m','mv', v[7]);
    setSlider('c','cv', v[8]);
  }
};
document.getElementById('p').oninput = e => { document.getElementById('pv').innerText=e.target.value; send('P:'+e.target.value); };
document.getElementById('i').oninput = e => { document.getElementById('iv').innerText=e.target.value; send('I:'+e.target.value); };
document.getElementById('d').oninput = e => { document.getElementById('dv').innerText=e.target.value; send('D:'+e.target.value); };
document.getElementById('kv').oninput = e => { document.getElementById('kvv').innerText=e.target.value; send('V:'+e.target.value); };
document.getElementById('ki2').oninput = e => { document.getElementById('kiv').innerText=e.target.value; send('W:'+e.target.value); };
document.getElementById('t').oninput = e => { document.getElementById('tv').innerText=e.target.value; send('X:'+e.target.value); };
document.getElementById('m').oninput = e => { document.getElementById('mv').innerText=e.target.value; send('M:'+e.target.value); };
document.getElementById('c').oninput = e => { document.getElementById('cv').innerText=e.target.value; send('C:'+e.target.value); };
</script></body></html>
)HTML";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        // Push the REAL live values to the newly-connected client so its
        // sliders sync to what the firmware is actually running, instead
        // of whatever numbers are hardcoded as fallbacks in the PAGE
        // HTML above. This is what fixes sliders showing a stale value
        // (e.g. Kp = 15) after you change a gain in code and reflash.
        char buf[128];
        float kp = Kp, ki = Ki, kd = Kd, kpv = Kp_vel, kiv = Ki_vel;
        float trim = ANGLE_TRIM_DEG;
        int minA = MIN_OUTPUT_A, minB = MIN_OUTPUT_B;
        float scaleA = MOTOR_A_SCALE, scaleB = MOTOR_B_SCALE;
        snprintf(buf, sizeof(buf), "Y:%.2f:%.2f:%.2f:%.3f:%.4f:%.1f:%d:%d:%.2f:%.2f",
                 kp, ki, kd, kpv, kiv, trim, minA, minB, scaleA, scaleB);
        client->text(buf);
        return;
    }
    if (type != WS_EVT_DATA) return;
    String msg((char*)data, len);
    char key = msg.charAt(0);
    float val = msg.substring(2).toFloat();
    switch (key) {
        case 'P': Kp = val; break;
        case 'I': Ki = val; break;
        case 'D': Kd = val; break;
        case 'V': Kp_vel = val; break;
        case 'W': Ki_vel = val; break;
        case 'X': ANGLE_TRIM_DEG = val; break;
        case 'M': MIN_OUTPUT_A = (int)val; break;
        case 'C': MIN_OUTPUT_B = (int)val; break;
        case 'E': MOTOR_A_SCALE = val; break;
        case 'G': MOTOR_B_SCALE = val; break;
        case 'F': targetAngleTrim = val ?  3.0 : 0.0; break;
        case 'B': targetAngleTrim = val ? -3.0 : 0.0; break;
        case 'L': steerBias       = val ?  30.0 : 0.0; break;   // swapped — was backwards
        case 'R': steerBias       = val ? -30.0 : 0.0; break;   // swapped — was backwards
        case 'S': targetAngleTrim = 0.0; steerBias = 0.0; break;
        case 'J': {
            // "J:x:y" — x,y each in [-1, 1] from the on-page joystick.
            // Reuses the same targetAngleTrim/steerBias variables (and
            // therefore the same slew-rate ramping) as the old F/B/L/R
            // buttons, just driven continuously instead of snapping to
            // a fixed value on press.
            String rest = msg.substring(2);
            int sep = rest.indexOf(':');
            float jx = rest.substring(0, sep).toFloat();
            float jy = rest.substring(sep + 1).toFloat();
            // JOY_MAX_TRIM_DEG/JOY_MAX_STEER are now file-scope constants,
            // shared with the ESP-NOW physical-joystick receiver above.
            targetAngleTrim = jy * JOY_MAX_TRIM_DEG;
            steerBias       = -jx * JOY_MAX_STEER;  // sign matches existing L/R convention
            break;
        }
    }
}

void setupTuningInterface() {
    // Explicit channel (4th arg) — must match WIFI_CHANNEL for ESP-NOW
    // from the physical joystick controller to be received at all.
    WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
    // This is the address the CONTROLLER's robotMac[] needs to contain —
    // the AP's MAC, not the station MAC WiFi.macAddress() would print.
    Serial.print("Robot AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", PAGE);
    });
    server.begin();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed — physical joystick won't work, webpage still will");
    } else {
        esp_now_register_recv_cb(onEspNowRecv);
    }
}

// =================== Setup ===================
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    #if RUN_I2C_SCANNER
    runI2CScanner();  // never returns — halts here, skips everything else
    #endif

    #if RUN_BASIC_MOTOR_GYRO_TEST
    runBasicMotorGyroTest();  // never returns — does its own IMU init
                               // with retries, skips everything below
    #endif

    if (!sox.begin_I2C()) { Serial.println("IMU not found"); while (1); }
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { while (1); }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.display();

    ledcAttach(AIN1, 20000, 8);
    ledcAttach(AIN2, 20000, 8);
    ledcAttach(BIN1, 20000, 8);
    ledcAttach(BIN2, 20000, 8);
    motorA(0); motorB(0);

    pinMode(ENC_A1, INPUT); pinMode(ENC_B1, INPUT);
    pinMode(ENC_A2, INPUT); pinMode(ENC_B2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_A1), encA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A2), encB_ISR, CHANGE);

    // THIS CALL WAS MISSING in the pasted version — without it, the
    // function above is defined but never runs, and setup() falls
    // straight through to gyro calibration + balancing even with
    // RUN_MOTOR_ENCODER_TEST set to 1.
    #if RUN_MOTOR_ENCODER_TEST
    runMotorEncoderTest();  // halts here (infinite loop inside) — never
                             // reaches gyro calibration or the balancing
                             // loop while this flag is 1.
    #endif

    Serial.println("Calibrating gyro bias — keep robot still...");
    const int CAL_SAMPLES = 200;
    float sum = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        sensors_event_t a, g, t;
        sox.getEvent(&a, &g, &t);
        sum += g.gyro.x * 180.0 / PI;
        delay(5);
    }
    gyroBias = sum / CAL_SAMPLES;
    Serial.print("Gyro bias: "); Serial.println(gyroBias, 3);

    setupTuningInterface();
    Serial.println("Ready.");
}

// =================== Main loop ===================
void loop() {
    static unsigned long lastTime = 0;
    static unsigned long lastDisplay = 0;
    static unsigned long lastTelemetry = 0;
    static float avgVelocityFiltered = 0.0;      // smoothed encoder velocity — see VEL_FILTER_ALPHA
    static float targetAngleTrimSmoothed = 0.0;  // ramped toward targetAngleTrim — see TRIM_SLEW_PER_SEC
    static float steerBiasSmoothed = 0.0;        // ramped toward steerBias — see STEER_SLEW_PER_SEC
    static bool motorEngaged = false;            // hysteresis deadband state — see below
    static float turnSuppress = 0.0;             // 0 = full Kp_vel authority, 1 = fully suppressed — see below

    unsigned long now = millis();
    if (now - lastTime < 10) return;
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Guard against a delayed loop iteration (e.g. WiFi/WebSocket
    // servicing) producing one abnormally large dt, which would
    // otherwise inflate the integral term and shrink the derivative
    // denominator in the same tick — see MAX_DT comment above.
    if (dt > MAX_DT) dt = MAX_DT;

    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    float accelAngle = atan2(accel.acceleration.y,
                             accel.acceleration.z) * 180.0 / PI - ANGLE_TRIM_DEG;
    float gyroRate = (gyro.gyro.x * 180.0 / PI) - gyroBias;

    angle = alpha * (angle + gyroRate * dt) + (1.0 - alpha) * accelAngle;

    // Ramp the commanded trim/steer toward their target values instead
    // of jumping instantly — avoids the derivative-kick-style spike a
    // step change would otherwise cause (see loop() notes below).
    float trimStep = TRIM_SLEW_PER_SEC * dt;
    if (targetAngleTrimSmoothed < targetAngleTrim) {
        targetAngleTrimSmoothed = min(targetAngleTrimSmoothed + trimStep, (float)targetAngleTrim);
    } else if (targetAngleTrimSmoothed > targetAngleTrim) {
        targetAngleTrimSmoothed = max(targetAngleTrimSmoothed - trimStep, (float)targetAngleTrim);
    }
    float steerStep = STEER_SLEW_PER_SEC * dt;
    if (steerBiasSmoothed < steerBias) {
        steerBiasSmoothed = min(steerBiasSmoothed + steerStep, (float)steerBias);
    } else if (steerBiasSmoothed > steerBias) {
        steerBiasSmoothed = max(steerBiasSmoothed - steerStep, (float)steerBias);
    }

    bool fallen = fabs(angle) > FALL_CUTOFF_DEG;
    int output = 0;
    // Hoisted out of the else-block below (rather than declared with
    // "float" inside it) so the OLED block further down can read the
    // latest value regardless of which branch ran this iteration.
    float avgVelocity = 0.0;

    if (fallen) {
        motorA(0); motorB(0);
        angleErrorIntegral = 0;
        velIntegral = 0;
        avgVelocityFiltered = 0.0;        // reset smoothing too — don't carry stale
                                           // velocity history into the next stand-up
        targetAngleTrimSmoothed = 0.0;    // don't carry a mid-ramp command into the
        steerBiasSmoothed = 0.0;          // next stand-up either — start neutral
        motorEngaged = false;             // start disengaged on the next stand-up too
        turnSuppress = 0.0;                // don't carry a mid-turn suppression level into the next stand-up
        countA = 0; countB = 0; lastCountA = 0; lastCountB = 0;
    } else {
        long dCountA = countA - lastCountA;
        long dCountB = countB - lastCountB;
        lastCountA = countA; lastCountB = countB;

        // Reverted to addition after both subtraction sign orders caused
        // an immediate runaway / total loss of balance on real hardware.
        // Kinematically, subtraction is arguably "more correct" (it
        // isolates real translation instead of canceling it), but that
        // also means it stops canceling the wheel movement the robot
        // does constantly just to hold itself up — a signal far larger
        // than Kp_vel/Ki_vel were ever tuned against. Addition is the
        // form that has actually balanced on this robot; don't change
        // it again without also re-tuning Kp_vel/Ki_vel from scratch
        // against the new signal scale.
        avgVelocity = ((dCountA + dCountB) / 2.0) / dt;

        // Raw avgVelocity is extremely noisy at this loop rate: dt is
        // ~10ms, so a single encoder-tick difference between iterations
        // (ordinary quantization noise, not real motion) swings the raw
        // value by tens of counts/sec. Smooth it with an EMA before it
        // feeds the outer loop, so tick-to-tick jitter doesn't get
        // interpreted as real velocity and jerk the setpoint around.
        avgVelocityFiltered = (1.0 - VEL_FILTER_ALPHA) * avgVelocityFiltered
                             + VEL_FILTER_ALPHA * avgVelocity;

        // Suppress the outer loop's proportional term while a turn is
        // actively commanded — same reasoning as before (during a turn,
        // imperfect wheel-speed matching can show up as a nonzero
        // "phantom" avgVelocity that isn't real drift, and at Kp_vel this
        // high that blip becomes a real spurious setpoint shift). BUT this
        // is now a smoothly ramped suppression factor instead of a hard
        // on/off switch — the old version snapped Kp_vel's full authority
        // back on in a single tick the instant steering ended, which is
        // exactly what caused the forward jerk right after a turn: full
        // Kp_vel plus whatever velocity reading existed at that exact
        // instant, applied all at once. Ramping it over TURN_SUPPRESS_SLEW
        // means there's no instant to re-apply a full-strength correction.
        float turnSuppressTarget = (fabs(steerBiasSmoothed) > 1.0) ? 1.0 : 0.0;
        const float TURN_SUPPRESS_SLEW = 3.0;   // per second — ~0.33s to fully engage/disengage
        if (turnSuppress < turnSuppressTarget) {
            turnSuppress = min(turnSuppress + TURN_SUPPRESS_SLEW * dt, turnSuppressTarget);
        } else if (turnSuppress > turnSuppressTarget) {
            turnSuppress = max(turnSuppress - TURN_SUPPRESS_SLEW * dt, turnSuppressTarget);
        }
        // Also taper how much the integral accumulates during a turn,
        // rather than freezing it outright — avoids a second discontinuity
        // when accumulation switches back on.
        velIntegral += avgVelocityFiltered * dt * (1.0 - turnSuppress);
        velIntegral = constrain(velIntegral, -500, 500);
        float velCorrection = -((Kp_vel * (1.0 - turnSuppress)) * avgVelocityFiltered
                               + Ki_vel * velIntegral);

        float setpoint = targetAngleTrimSmoothed + velCorrection;
        float error = setpoint - angle;

        angleErrorIntegral += error * dt;
        angleErrorIntegral = constrain(angleErrorIntegral,
                                        -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

        // Derivative-on-measurement: based on the angle itself, not on
        // the error. A step change in setpoint (from targetAngleTrim,
        // via FWD/BACK) no longer causes a one-tick derivative spike —
        // it only reacts to how fast the robot itself is actually
        // tilting. Combined with ramping the setpoint above, this is
        // what should fix the FWD/BACK overshoot and LEFT/RIGHT tipping.
        float derivative = -(angle - lastAngle) / dt;
        lastAngle = angle;

        output = (int)(Kp * error + Ki * angleErrorIntegral + Kd * derivative);
        output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);

        // Hysteresis (Schmitt-trigger) deadband instead of a single flat
        // threshold. A flat deadband still has ONE crossing point, and
        // right at standing-still equilibrium, sensor noise flickers the
        // raw output back and forth across it many times a second — each
        // crossing snaps output from 0 straight to ~MIN_OUTPUT (~148),
        // which is the actual jitter, not just "small output". Requiring
        // a HIGHER value to engage than to disengage means noise sitting
        // near the boundary can't keep re-triggering that jump — output
        // has to clearly commit to one side before the motor turns on,
        // and clearly fall off before it turns back off.
        const int ENGAGE_THRESHOLD    = 8;  // must exceed this to turn ON
        const int DISENGAGE_THRESHOLD = 3;  // must drop below this to turn OFF
        if (!motorEngaged && abs(output) > ENGAGE_THRESHOLD)    motorEngaged = true;
        if (motorEngaged  && abs(output) < DISENGAGE_THRESHOLD) motorEngaged = false;
        if (!motorEngaged) output = 0;

        // Rescale [0, MAX_OUTPUT] onto [MIN_OUTPUT_x, MAX_OUTPUT] per motor,
        // instead of snapping any nonzero output up to a flat floor. A flat
        // snap made every error under ~(MIN/Kp) degrees produce the exact
        // same output — bang-bang control with no proportionality right
        // around 0°, which is most of the time while balancing. This keeps
        // small errors proportionally smaller than large ones while still
        // guaranteeing enough duty to overcome each motor's own stiction.
        int outA = output, outB = output;
        if (outA > 0)      outA = MIN_OUTPUT_A + (outA * (MAX_OUTPUT - MIN_OUTPUT_A)) / MAX_OUTPUT;
        else if (outA < 0) outA = -MIN_OUTPUT_A + (outA * (MAX_OUTPUT - MIN_OUTPUT_A)) / MAX_OUTPUT;
        if (outB > 0)      outB = MIN_OUTPUT_B + (outB * (MAX_OUTPUT - MIN_OUTPUT_B)) / MAX_OUTPUT;
        else if (outB < 0) outB = -MIN_OUTPUT_B + (outB * (MAX_OUTPUT - MIN_OUTPUT_B)) / MAX_OUTPUT;

        // Shrink the steer bias automatically as the balance output grows,
        // instead of always adding the full fixed amount. Without this, a
        // fixed steerBias added on top of an already-large balance output
        // eats into that motor's remaining headroom before MAX_OUTPUT —
        // silently capping the balance correction right when it's needed
        // most, which is a plausible cause of tipping too far while turning.
        // This keeps steering authority proportional to how much room is
        // actually left, rather than fighting the balance loop for it.
        int headroomA = MAX_OUTPUT - abs(outA);
        int headroomB = MAX_OUTPUT - abs(outB);
        int headroom = min(headroomA, headroomB);
        float safeSteer = constrain(steerBiasSmoothed, (float)-headroom, (float)headroom);

        motorA((int)(outA * MOTOR_A_SCALE) + (int)safeSteer);
        motorB((int)(outB * MOTOR_B_SCALE) - (int)safeSteer);

        if (now - lastTelemetry >= 50) {
            lastTelemetry = now;
            String frame = "T:" + String(angle, 1) + ":" + String(output);
            ws.textAll(frame);
        }
    }

    // ---- OLED, throttled to 10Hz ----
    if (now - lastDisplay >= 100) {
        lastDisplay = now;
        display.clearDisplay();

        #if ENABLE_BATTERY_MONITOR
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("Batt: ");
        display.print(lastVbat, 2);
        display.print("V");
        #endif

        display.setTextSize(1);
        display.setCursor(0, 12);
        display.print("Angle:");
        display.setTextSize(2);
        display.setCursor(0, 22);
        display.print(angle, 1);
        display.setTextSize(1);
        display.setCursor(0, 46);
        display.print("Out: "); display.print(output);

        display.setCursor(64, 46);
        display.print("V:"); display.print(avgVelocityFiltered, 1);

        display.setCursor(0, 56);
        display.print("A:"); display.print(countA);
        display.print(" B:"); display.print(-countB);

        // Small tilt visual, right side of screen — independent box,
        // doesn't overlap the text on the left.
        drawTiltIndicator(105, 32, 18, angle);

        display.display();
    }

    #if ENABLE_BATTERY_MONITOR
    static unsigned long lastBatCheck = 0;
    if (now - lastBatCheck >= 1000) {
        lastBatCheck = now;
        // analogReadMilliVolts() uses the ESP32's factory-calibrated ADC
        // data, correcting for the chip's known ADC nonlinearity — far
        // more accurate than scaling raw analogRead() counts linearly,
        // which is what caused the earlier ~1V discrepancy vs. a DMM.
        lastVbat = (analogReadMilliVolts(VBAT_PIN) / 1000.0) * VBAT_DIVIDER_RATIO;
        if (lastVbat < VBAT_CUTOFF) {
            motorA(0); motorB(0);
            Serial.println("LOW BATTERY — motors disabled");
        }
    }
    #endif
}