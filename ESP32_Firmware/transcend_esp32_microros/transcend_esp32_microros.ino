// ================================================================
//  TRANSCEND — ESP32 micro_ROS Firmware  (Float32MultiArray version)
//  FIXED VERSION — resolves watchdog timeout & unreliable cmd delivery
//
//  Fix summary:
//    1. Debug printf moved INSIDE AGENT_CONNECTED case (was between cases
//       — C++ syntax error causing undefined behaviour)
//    2. Background spin thread on Pi side publishes at 50Hz but ESP32
//       executor spin window was too narrow — increased to 15ms
//    3. Watchdog bumped to 2000ms — Pi hw_bridge background thread has
//       ~6ms sleep loop, so at 50Hz commands arrive every 20ms; 1000ms
//       should be fine but 2000ms gives headroom during reconnects
//    4. Agent ping reduced to (100ms, 1 attempt) to minimise blocking
//    5. Agent check interval increased to 5000ms
//    6. Added minimum PWM deadband (30/255) so motors actually overcome
//       static friction — at x=0.25m/s initial PWM was only ~13 (5%),
//       below motor stall threshold
// ================================================================

#include <micro_ros_arduino.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

// ================================================================
//  CONFIG
// ================================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* AGENT_IP      = "YOUR_AGENT_IP";
const int   AGENT_PORT    = 8888;

// ================================================================
//  ROBOT PHYSICAL CONSTANTS
// ================================================================
const float WHEEL_RADIUS_M      = 0.022f;
const float WHEEL_SEPARATION_M  = 0.158f;
const float TICKS_PER_REV       = 1395.0f;
const float MAX_SPEED_TICKS     = 2865.0f;
const float TICKS_PER_RAD       = TICKS_PER_REV / (2.0f * M_PI);  // ≈ 222.0

// ================================================================
//  PINS
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

#define FL_ENC_A   34
#define FL_ENC_B   35
#define FR_ENC_A   36
#define FR_ENC_B   39
#define RL_ENC_A    4
#define RL_ENC_B   16
#define RR_ENC_A   17
#define RR_ENC_B   15

// ================================================================
//  MOTOR POLARITY  (-1 = reverse wired)
// ================================================================
#define DIR_FL  -1
#define DIR_FR   1
#define DIR_RL  -1
#define DIR_RR   1

// ================================================================
//  MOTOR DEADBAND — PWM values below this don't overcome static friction
//  Increase if motors still don't spin at low speeds
// ================================================================
#define PWM_DEADBAND  30    // out of 255

// ================================================================
//  PID GAINS
// ================================================================
float Kp = 3.0f;
float Ki = 4.0f;
float Kd = 0.0f;

#define PID_INTERVAL_MS   20      // 50 Hz
#define CMD_TIMEOUT_MS   2000     // watchdog — 2s gives headroom during reconnects

// ================================================================
//  ENCODER COUNTERS
// ================================================================
volatile long ticksFL = 0, ticksFR = 0, ticksRL = 0, ticksRR = 0;
long prevTicksFL = 0, prevTicksFR = 0, prevTicksRL = 0, prevTicksRR = 0;

// ================================================================
//  MOTOR STATE
// ================================================================
float targetFL = 0, targetFR = 0, targetRL = 0, targetRR = 0;  // rad/s
float actualFL = 0, actualFR = 0, actualRL = 0, actualRR = 0;  // rad/s
float pwmFL    = 0, pwmFR    = 0, pwmRL    = 0, pwmRR    = 0;  // [-255, 255]

float intFL = 0, intFR = 0, intRL = 0, intRR = 0;
float preFL = 0, preFR = 0, preRL = 0, preRR = 0;
float prevTargFL = 0, prevTargFR = 0, prevTargRL = 0, prevTargRR = 0;

bool driversEnabled = false;
unsigned long lastCmdTime = 0;

// ================================================================
//  micro_ROS OBJECTS
// ================================================================
rcl_node_t           node;
rcl_allocator_t      allocator;
rclc_support_t       support;
rclc_executor_t      executor;

rcl_subscription_t   cmd_sub;
rcl_publisher_t      state_pub;

std_msgs__msg__Float32MultiArray cmd_msg;
std_msgs__msg__Float32MultiArray state_msg;

float cmd_data[4]   = {0, 0, 0, 0};
float state_data[4] = {0, 0, 0, 0};

enum AgentState { WAITING_FOR_WIFI, WAITING_FOR_AGENT, AGENT_CONNECTED };
AgentState agent_state = WAITING_FOR_WIFI;

// ================================================================
//  ENCODER ISRs
// ================================================================
void IRAM_ATTR isrFL() { ticksFL += (digitalRead(FL_ENC_B) != digitalRead(FL_ENC_A)) ? 1 : -1; }
void IRAM_ATTR isrFR() { ticksFR += (digitalRead(FR_ENC_B) != digitalRead(FR_ENC_A)) ? 1 : -1; }
void IRAM_ATTR isrRL() { ticksRL += (digitalRead(RL_ENC_B) != digitalRead(RL_ENC_A)) ? 1 : -1; }
void IRAM_ATTR isrRR() { ticksRR += (digitalRead(RR_ENC_B) != digitalRead(RR_ENC_A)) ? 1 : -1; }

// ================================================================
//  MOTOR HARDWARE
// ================================================================
void enableDrivers() {
  digitalWrite(STBY_FRONT, HIGH);
  digitalWrite(STBY_REAR,  HIGH);
  driversEnabled = true;
}

void disableDrivers() {
  for (auto pin : {FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2})
    digitalWrite(pin, LOW);
  ledcWrite(FL_PWM, 0); ledcWrite(FR_PWM, 0);
  ledcWrite(RL_PWM, 0); ledcWrite(RR_PWM, 0);
  digitalWrite(STBY_FRONT, LOW);
  digitalWrite(STBY_REAR,  LOW);
  driversEnabled = false;
}

// FIX: apply deadband so small PWM values actually move the motors
void writeMotor(int in1, int in2, int pwmPin, float pwmVal, int dir) {
  int s = constrain((int)(pwmVal * dir), -255, 255);
  if (s > 0) {
    s = max(s, PWM_DEADBAND);          // enforce minimum forward PWM
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  ledcWrite(pwmPin, s);
  } else if (s < 0) {
    s = min(s, -PWM_DEADBAND);         // enforce minimum reverse PWM
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); ledcWrite(pwmPin, -s);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  ledcWrite(pwmPin, 0);
  }
}

void stopAll() {
  targetFL = targetFR = targetRL = targetRR = 0.0f;
  pwmFL    = pwmFR    = pwmRL    = pwmRR    = 0.0f;
  intFL    = intFR    = intRL    = intRR    = 0.0f;
  preFL    = preFR    = preRL    = preRR    = 0.0f;
  disableDrivers();
}

// ================================================================
//  PID COMPUTE (rad/s → PWM)
// ================================================================
float computePID(float target, float actual,
                 float &integral, float &prevErr,
                 float &prevTarget, float dt) {
  if (target == 0.0f) {
    integral = 0.0f; prevErr = 0.0f; prevTarget = 0.0f;
    return 0.0f;
  }
  if ((target > 0.0f && prevTarget < 0.0f) ||
      (target < 0.0f && prevTarget > 0.0f)) {
    integral = 0.0f; prevErr = 0.0f;
  }
  prevTarget = target;

  float error   = target - actual;
  integral     += error * dt;
  float awLimit = (Ki > 0.001f) ? (255.0f / Ki) : 1000.0f;
  integral      = constrain(integral, -awLimit, awLimit);
  float deriv   = (dt > 0.0f) ? (error - prevErr) / dt : 0.0f;
  prevErr       = error;

  return constrain(Kp * error + Ki * integral + Kd * deriv, -255.0f, 255.0f);
}

// ================================================================
//  PID UPDATE — 50 Hz
// ================================================================
void updatePID() {
  if (!driversEnabled) return;

  float dt = PID_INTERVAL_MS / 1000.0f;

  noInterrupts();
  long nowFL = ticksFL, nowFR = ticksFR;
  long nowRL = ticksRL, nowRR = ticksRR;
  interrupts();

  long dFL = nowFL - prevTicksFL;  prevTicksFL = nowFL;
  long dFR = nowFR - prevTicksFR;  prevTicksFR = nowFR;
  long dRL = nowRL - prevTicksRL;  prevTicksRL = nowRL;
  long dRR = nowRR - prevTicksRR;  prevTicksRR = nowRR;

  // FIX: derive speed sign from encoder ticks, apply polarity constant
  actualFL = ((float)dFL / dt) / TICKS_PER_RAD * DIR_FL;
  actualFR = ((float)dFR / dt) / TICKS_PER_RAD * DIR_FR;
  actualRL = ((float)dRL / dt) / TICKS_PER_RAD * DIR_RL;
  actualRR = ((float)dRR / dt) / TICKS_PER_RAD * DIR_RR;

  pwmFL = computePID(targetFL, actualFL, intFL, preFL, prevTargFL, dt);
  pwmFR = computePID(targetFR, actualFR, intFR, preFR, prevTargFR, dt);
  pwmRL = computePID(targetRL, actualRL, intRL, preRL, prevTargRL, dt);
  pwmRR = computePID(targetRR, actualRR, intRR, preRR, prevTargRR, dt);

  writeMotor(FL_IN1, FL_IN2, FL_PWM, pwmFL, DIR_FL);
  writeMotor(FR_IN1, FR_IN2, FR_PWM, pwmFR, DIR_FR);
  writeMotor(RL_IN1, RL_IN2, RL_PWM, pwmRL, DIR_RL);
  writeMotor(RR_IN1, RR_IN2, RR_PWM, pwmRR, DIR_RR);
}

// ================================================================
//  CALLBACK — /wheel_vel_cmd received
// ================================================================
void cmdCallback(const void* msg_in) {
  const std_msgs__msg__Float32MultiArray* msg =
    (const std_msgs__msg__Float32MultiArray*)msg_in;

  if (msg->data.size < 4) return;

  targetFL = msg->data.data[0];
  targetFR = msg->data.data[1];
  targetRL = msg->data.data[2];
  targetRR = msg->data.data[3];

  lastCmdTime = millis();

  if (!driversEnabled &&
      (targetFL != 0 || targetFR != 0 || targetRL != 0 || targetRR != 0)) {
    enableDrivers();
  }
}

// ================================================================
//  PUBLISH WHEEL STATE
// ================================================================
void publishWheelState() {
  state_data[0] = actualFL;
  state_data[1] = actualFR;
  state_data[2] = actualRL;
  state_data[3] = actualRR;
  rcl_publish(&state_pub, &state_msg, NULL);
}

// ================================================================
//  MESSAGE INIT
// ================================================================
void initFloat32MultiArray(std_msgs__msg__Float32MultiArray* msg,
                           float* data_buf, size_t size) {
  msg->layout.dim.data     = nullptr;
  msg->layout.dim.size     = 0;
  msg->layout.dim.capacity = 0;
  msg->layout.data_offset  = 0;
  msg->data.data           = data_buf;
  msg->data.size           = size;
  msg->data.capacity       = size;
}

// ================================================================
//  micro_ROS LIFECYCLE
// ================================================================
bool createEntities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "transcend_esp32", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_vel_cmd") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
        &state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_vel_state") != RCL_RET_OK) return false;

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;

  if (rclc_executor_add_subscription(
        &executor, &cmd_sub, &cmd_msg,
        &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

  Serial.println("[micro_ROS] Entities created. Node running.");
  return true;
}

void destroyEntities() {
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_publisher_fini(&state_pub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  stopAll();
  Serial.println("[micro_ROS] Entities destroyed.");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TRANSCEND ESP32 micro_ROS Firmware (FIXED) ===");

  for (auto pin : {FL_IN1, FL_IN2, FR_IN1, FR_IN2,
                   RL_IN1, RL_IN2, RR_IN1, RR_IN2,
                   STBY_FRONT, STBY_REAR}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  ledcAttach(FL_PWM, 1000, 8);
  ledcAttach(FR_PWM, 1000, 8);
  ledcAttach(RL_PWM, 1000, 8);
  ledcAttach(RR_PWM, 1000, 8);

  pinMode(FL_ENC_A, INPUT_PULLUP); pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP); pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR, CHANGE);

  initFloat32MultiArray(&cmd_msg,   cmd_data,   4);
  initFloat32MultiArray(&state_msg, state_data, 4);

  Serial.printf("[WiFi] Connecting to %s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500); Serial.print("."); attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected. ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
    agent_state = WAITING_FOR_AGENT;
  } else {
    Serial.println("\n[WiFi] FAILED. Restarting in 5s...");
    delay(5000);
    ESP.restart();
  }

  set_microros_wifi_transports(
    (char*)WIFI_SSID, (char*)WIFI_PASSWORD,
    (char*)AGENT_IP,  AGENT_PORT
  );

  Serial.printf("[micro_ROS] Will connect to agent at %s:%d\n", AGENT_IP, AGENT_PORT);
}

// ================================================================
//  LOOP — micro_ROS state machine + PID
//
//  KEY FIX: debug print and ALL logic is INSIDE the correct case block.
//  Previous version had debug code floating between case statements —
//  this is illegal C++ and caused unpredictable behaviour.
// ================================================================
void loop() {
  static unsigned long lastPID        = 0;
  static unsigned long lastPub        = 0;
  static unsigned long lastPing       = 0;
  static unsigned long lastAgentCheck = 0;
  static unsigned long lastDebug      = 0;   // debug timer — INSIDE loop(), not between cases
  unsigned long now = millis();

  switch (agent_state) {

    // ----------------------------------------------------------
    case WAITING_FOR_AGENT:
      if (now - lastPing > 500) {
        lastPing = now;
        if (rmw_uros_ping_agent(200, 3) == RMW_RET_OK) {
          Serial.println("[micro_ROS] Agent found! Creating entities...");
          if (createEntities()) {
            lastAgentCheck = now;
            agent_state = AGENT_CONNECTED;
          } else {
            Serial.println("[micro_ROS] Entity creation failed. Retrying...");
            destroyEntities();
          }
        }
      }
      break;

    // ----------------------------------------------------------
    case AGENT_CONNECTED:

      // FIX 1: increased spin window from 1ms → 15ms so UDP packets
      // are reliably picked up on a WiFi link
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(15));

      // PID at 50 Hz
      if (now - lastPID >= PID_INTERVAL_MS) {
        lastPID = now;
        updatePID();
      }

      // Publish wheel state at 50 Hz
      if (now - lastPub >= PID_INTERVAL_MS) {
        lastPub = now;
        publishWheelState();
      }

      // Watchdog — FIX 2: 2000ms instead of 500ms
      if (driversEnabled && (now - lastCmdTime > CMD_TIMEOUT_MS)) {
        Serial.println("[WATCHDOG] Timeout — stopping motors.");
        stopAll();
      }

      // FIX 3: debug print is NOW correctly inside AGENT_CONNECTED
      // Remove this block once wheels are confirmed spinning
      if (now - lastDebug > 500) {
        lastDebug = now;
        Serial.printf("[DBG] tgt:%.2f act:%.2f pwm:%.1f ticksFL:%ld\n",
          targetFL, actualFL, pwmFL, ticksFL);
      }

      // FIX 4: agent ping is (100ms, 1) not (500ms, 2) — 10x less blocking
      if (now - lastAgentCheck > 5000) {
        lastAgentCheck = now;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          Serial.println("[micro_ROS] Agent lost. Cleaning up...");
          destroyEntities();
          agent_state = WAITING_FOR_AGENT;
        }
      }
      break;

    // ----------------------------------------------------------
    case WAITING_FOR_WIFI:
      if (WiFi.status() == WL_CONNECTED) agent_state = WAITING_FOR_AGENT;
      break;
  }
}
