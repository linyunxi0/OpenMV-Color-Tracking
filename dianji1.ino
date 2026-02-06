/*
  Dual DC Motor Speed PID for Arduino Mega2560
  - Encoders: single channel A rising-edge only
  - Wheel PPR (counts per wheel revolution): 990  (11 PPR motor * 90:1 gearbox)
  - Motor driver: PWM + DIR per motor (your doc: PWM pins 5/6, DIR pins 4/7)
  - Serial command:
      1) Set wheel targets (rpm):  "LR <left_rpm> <right_rpm>\n"
         e.g. "LR 30 30"
      2) Set base/turn (rpm):      "BT <base_rpm> <turn_rpm>\n"
         e.g. "BT 25 10" -> left=35 right=15
      3) Tune PID:                 "PID <Kp> <Ki> <Kd>\n"
         e.g. "PID 1.2 0.6 0.02"
      4) Stop:                     "STOP\n"
*/
const bool INVERT_L = false;
const bool INVERT_R = true;   // 先把右轮设 true（因为你右轮方向反）

#include <Arduino.h>
// ================== OpenMV UART (Mega Serial2) ==================
#define CAM Serial2  // RX2=17, TX2=16

String camLine;

// 追踪参数：先用这组，后面再调
float KP_TURN = 0.06f;      // 像素误差 -> turn(rpm)
int   ERR_DB  = 10;         // 像素死区（防抖）
float TURN_MAX = 12.0f;     // 转向rpm上限

// ======= 到 20cm 停车参数 =======
int  AREA_STOP = 5200;              // 20cm 停止（你测到 area≈5200）
const uint32_t HOLD_MS = 5000;     // 停 10 秒

enum Mode { MODE_TRACK, MODE_HOLD, MODE_SEARCH };
Mode mode = MODE_SEARCH;

uint32_t holdUntilMs = 0;

// 停车结束后的冷却期：防止刚恢复又立刻再次触发停车
uint32_t coolUntilMs = 0;
const uint32_t COOL_MS = 800;       // 0.8 秒

// 防抖：连续 N 次满足“接近”才触发停车
int nearCount = 0;
const int NEAR_COUNT_REQ = 5;       // 5次*20ms=100ms


// 面积 -> 前进速度（逼近）
int AREA_FAR  = 600;       // 小于此认为远
int AREA_NEAR = 3000;      // 大于此认为很近
float BASE_MAX = 12.0f;    // 远时前进rpm
float BASE_MIN = 0.0f;     // 近时减速到0（可改成-6实现太近后退）

// 丢目标：先停一下，再原地搜红色
uint32_t lastSeenMs = 0;
const uint32_t LOST_GRACE_MS  = 200;   // 丢失短暂先停（抗抖）
const uint32_t LOST_SEARCH_MS = 1200;   // 超过这时间进入搜索
float SEARCH_TURN_RPM = 8.0f;          // 搜索转速（6~10）
int searchDir = +1;                   // +1 右转搜，-1 左转搜

// 让“视觉数据”也能刷新超时（避免 3 秒自动停）
bool camActive = true;


// ================== Pin Mapping ==================
const uint8_t PWM_L = 5;     // left motor PWM
const uint8_t PWM_R = 6;     // right motor PWM
const uint8_t DIR_L = 4;     // left motor direction
const uint8_t DIR_R = 7;     // right motor direction

// Encoder A channels (single channel used)
const uint8_t ENC_L_A = 21;  // left encoder A  (interrupt-capable)
const uint8_t ENC_R_A = 18;  // right encoder A (interrupt-capable)

// ================== Encoder / Speed ==================
const float WHEEL_PPR = 990.0f;        // counts per wheel revolution (A rising only)
const float DT = 0.02f;                // control period seconds (20ms)
const uint32_t DT_MS = (uint32_t)(DT * 1000);

// ================== Motor Output Limits ==================
const int PWM_MAX = 255;
const int PWM_MIN_START = 80;           // deadzone compensation: below this often won't move
const float RPM_STOP_EPS = 0.5f;        // treat target near 0 as stop

// ================== PID Defaults ==================
// 先给一组“能跑起来”的默认值：你后续用串口 PID 命令再调
float Kp = 1.2f;
float Ki = 0.8f;
float Kd = 0.02f;

// Anti-windup limit for integral term (in "PWM units")
const float I_LIM = 120.0f;

// Optional low-pass filter for measured rpm
const float RPM_LPF_ALPHA = 0.35f; // 0~1, larger = less filtering

// ================== Runtime Variables ==================
volatile long encCountL = 0;
volatile long encCountR = 0;

float rpmMeasL = 0, rpmMeasR = 0;
float rpmFiltL = 0, rpmFiltR = 0;

float targetRpmL = 0, targetRpmR = 0;

float iTermL = 0, iTermR = 0;
float lastErrL = 0, lastErrR = 0;

uint32_t lastControlMs = 0;

// Safety: stop if no command for a while (optional)
uint32_t lastCmdMs = 0;
const uint32_t CMD_TIMEOUT_MS = 3000;  // 3 seconds no command => stop

// ================== Encoder ISRs ==================
void isrEncL() { encCountL++; }
void isrEncR() { encCountR++; }

// ================== Helpers ==================
static inline int applyDeadzone(int pwmSigned) {
  if (pwmSigned == 0) return 0;
  int s = (pwmSigned > 0) ? 1 : -1;
  int mag = abs(pwmSigned);
  if (mag < PWM_MIN_START) mag = PWM_MIN_START;
  if (mag > PWM_MAX) mag = PWM_MAX;
  return s * mag;
}

void setMotor(int pwmPin, int dirPin, int pwmSigned, bool invertDir) {
  if (pwmSigned == 0) {
    analogWrite(pwmPin, 0);
    return;
  }

  bool forward = (pwmSigned > 0);
  if (invertDir) forward = !forward;

  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(pwmSigned), 0, 255));
}
// ✅ 新增：3 参数包装（兼容老调用）
void setMotor(int pwmPin, int dirPin, int pwmSigned) {
  bool inv = false;
  if (pwmPin == PWM_L) inv = INVERT_L;
  else if (pwmPin == PWM_R) inv = INVERT_R;
  setMotor(pwmPin, dirPin, pwmSigned, inv);
}

void stopAll() {
  setMotor(PWM_L, DIR_L, 0);
  setMotor(PWM_R, DIR_R, 0);
  iTermL = iTermR = 0;
  lastErrL = lastErrR = 0;
}

// Convert counts in DT interval to rpm
static inline float countsToRpm(long counts) {
  // rpm = (counts / PPR) * (60 / DT)
  return (counts * (60.0f / (WHEEL_PPR * DT)));
}

// PID controller returning signed PWM output
int pidStep(float targetRpm, float measRpm, float &iTerm, float &lastErr) {
  // For near-zero target, force stop
  if (fabs(targetRpm) < RPM_STOP_EPS) {
    iTerm = 0;
    lastErr = 0;
    return 0;
  }

  float err = targetRpm - measRpm;

  // Integral (anti-windup clamp)
  iTerm += Ki * err * DT;
  if (iTerm > I_LIM) iTerm = I_LIM;
  if (iTerm < -I_LIM) iTerm = -I_LIM;

  // Derivative on error
  float d = (err - lastErr) / DT;
  lastErr = err;

  float u = Kp * err + iTerm + Kd * d;

  // Convert to PWM (signed)
  int pwm = (int)lround(u);

  // Saturate
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  // Deadzone compensation
  pwm = applyDeadzone(pwm);

  return pwm;
}

// ================== Serial Command Parsing ==================
String line;

void handleLine(String s) {
  s.trim();            // ⭐关键：去掉首尾空格、\r、\n、tab
  lastCmdMs = millis();

  Serial.print("HANDLE: [");  // 调试：确认最终字符串
  Serial.print(s);
  Serial.println("]");

  if (s.equalsIgnoreCase("STOP")) {
    targetRpmL = 0;
    targetRpmR = 0;
    stopAll();
    Serial.println("OK STOP");
    return;
  }

 if (s.startsWith("LR") || s.startsWith("lr")) {
  long l = 0, r = 0;
  int n = sscanf(s.c_str(), "LR %ld %ld", &l, &r);
  if (n != 2) n = sscanf(s.c_str(), "lr %ld %ld", &l, &r);

  if (n == 2) {
    targetRpmL = (float)l;
    targetRpmR = (float)r;
    Serial.print("OK LR "); Serial.print(targetRpmL); Serial.print(" "); Serial.println(targetRpmR);
  } else {
    Serial.println("ERR LR");
  }
  return;
}


 if (s.startsWith("BT") || s.startsWith("bt")) {
  long base = 0, turn = 0;
  int n = sscanf(s.c_str(), "BT %ld %ld", &base, &turn);
  if (n != 2) n = sscanf(s.c_str(), "bt %ld %ld", &base, &turn);

  if (n == 2) {
    targetRpmL = (float)(base + turn);
    targetRpmR = (float)(base - turn);
    Serial.print("OK BT -> LR "); Serial.print(targetRpmL); Serial.print(" "); Serial.println(targetRpmR);
  } else {
    Serial.println("ERR BT");
  }
  return;
}


  if (s.startsWith("PID") || s.startsWith("pid")) {
    float p, i, d;
    if (sscanf(s.c_str(), "%*s %f %f %f", &p, &i, &d) == 3) {
      Kp = p; Ki = i; Kd = d;
      iTermL = iTermR = 0;
      lastErrL = lastErrR = 0;
      Serial.print("OK PID "); Serial.print(Kp); Serial.print(" "); Serial.print(Ki); Serial.print(" "); Serial.println(Kd);
    } else {
      Serial.println("ERR PID");
    }
    return;
  }
if (s.startsWith("RAW") || s.startsWith("raw")) {
  long l = 0, r = 0;
  int n = sscanf(s.c_str(), "RAW %ld %ld", &l, &r);
  if (n != 2) n = sscanf(s.c_str(), "raw %ld %ld", &l, &r);

  if (n == 2) {
    // 直接输出，绕过PID
    setMotor(PWM_L, DIR_L, (int)l, INVERT_L);
    setMotor(PWM_R, DIR_R, (int)r, INVERT_R);
    Serial.print("OK RAW "); Serial.print(l); Serial.print(" "); Serial.println(r);
  } else {
    Serial.println("ERR RAW");
  }
  return;
}

  if (s.equalsIgnoreCase("STAT")) {
    Serial.print("T "); Serial.print(targetRpmL); Serial.print(" "); Serial.print(targetRpmR);
    Serial.print(" | M "); Serial.print(rpmFiltL); Serial.print(" "); Serial.println(rpmFiltR);
    return;
  }

  Serial.println("ERR CMD");
}


void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    // 抓包：打印收到的字符和 ASCII 码
    //Serial.print("RX char='");
    //Serial.print(c);
    //Serial.print("'  code=");
   // Serial.println((int)(uint8_t)c);

    // 兼容所有行尾：\n 或 \r
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        Serial.print("LINE: "); Serial.println(line);   // 看看整行是什么
        handleLine(line);
        line = "";
      }
    } else {
      if (line.length() < 80) line += c;
    }
  }
}
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


// ================== OpenMV helpers ==================

void handleCamLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  // 正在停车(HOLD)时，忽略相机数据，防止覆盖停车指令
  if (mode == MODE_HOLD) return;

  // L：丢失（不更新 lastSeenMs，让 loop 的丢失逻辑去转圈）
  if (s[0] == 'L') return;

  // 只处理 E
  if (s[0] != 'E') return;

  long err = 0, area = 0;
  if (sscanf(s.c_str(), "E %ld %ld", &err, &area) != 2) return;

  // 收到视觉数据就刷新超时
  lastCmdMs  = millis();
  lastSeenMs = millis();

  // 可选：调试打印（刷屏会影响控制，调好后建议注释）
  // Serial.print("AREA="); Serial.println(area);

  // ======= 到达判定：接近则进入 HOLD（停10秒）=======
  if (AREA_STOP > 0) {
    // 冷却期内不触发停车
    if (millis() < coolUntilMs) {
      nearCount = 0;
    } else {
      if (area >= AREA_STOP) nearCount++;
      else nearCount = 0;

      if (nearCount >= NEAR_COUNT_REQ) {
        mode = MODE_HOLD;
        holdUntilMs = millis() + HOLD_MS;
        targetRpmL = 0;
        targetRpmR = 0;
        return;
      }
    }
  }

  // 正常识别到目标：进入 TRACK
  mode = MODE_TRACK;

  // 记录误差方向：丢失后按这个方向搜
  if (err > 0)      searchDir = +1;
  else if (err < 0) searchDir = -1;

  // deadband
  if (abs(err) < ERR_DB) err = 0;

  // turn
  float turn = KP_TURN * (float)err;
  turn = clampf(turn, -TURN_MAX, TURN_MAX);

  // base：面积越大越近 -> 越慢
  float base;
  if (area <= AREA_FAR) base = BASE_MAX;
  else if (area >= AREA_NEAR) base = BASE_MIN;
  else {
    float t = (float)(area - AREA_FAR) / (float)(AREA_NEAR - AREA_FAR);
    base = BASE_MAX + t * (BASE_MIN - BASE_MAX);
  }

  targetRpmL = base + turn;
  targetRpmR = base - turn;
}


void readCam() {
  while (CAM.available()) {
    char c = (char)CAM.read();
    if (c == '\n' || c == '\r') {
      if (camLine.length() > 0) {
        handleCamLine(camLine);
        camLine = "";
      }
    } else {
      if (camLine.length() < 80) camLine += c;
    }
  }
}



// ================== Setup / Loop ==================
void setup() {
  Serial.begin(115200);
  delay(200);
  CAM.begin(115200);   // OpenMV UART
  lastSeenMs = millis();

  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, RISING);

  stopAll();

  lastControlMs = millis();
  lastCmdMs = millis();

  Serial.println("READY");
  Serial.println("CMD: LR <Lrpm> <Rrpm> | BT <base> <turn> | PID <Kp> <Ki> <Kd> | STOP | STAT");
  mode = MODE_HOLD;
holdUntilMs = millis() + 500;  // 上电先停0.5秒防抖

}

void loop() {
  readSerial();
  if (camActive) readCam();

  uint32_t now = millis();
  if (now - lastControlMs >= DT_MS) {
    lastControlMs += DT_MS;

        // Safety timeout：如果你不用手动串口，而是用相机，就别让它 3 秒自动停
    // lastCmdMs 会在 handleCamLine(E...) 里刷新
    if (now - lastCmdMs > CMD_TIMEOUT_MS) {
      targetRpmL = 0;
      targetRpmR = 0;
    }
// ======= HOLD：停 10 秒 =======
if (mode == MODE_HOLD) {
  targetRpmL = 0;
  targetRpmR = 0;

  if ((int32_t)(now - holdUntilMs) >= 0) {
    mode = MODE_SEARCH;
    nearCount = 0;
    lastSeenMs = now;
    coolUntilMs = now + COOL_MS;
  }
}

if (mode != MODE_HOLD) {
    // ===== 丢目标策略：停一下 -> 原地转圈找红色 =====
    uint32_t dtLost = now - lastSeenMs;
    if (dtLost > LOST_SEARCH_MS) {
      float turn = searchDir * SEARCH_TURN_RPM;
      targetRpmL = +turn;
      targetRpmR = -turn;
    } else if (dtLost > LOST_GRACE_MS) {
      targetRpmL = 0;
      targetRpmR = 0;
    }
}

    // Grab and reset encoder counts atomically
    noInterrupts();
    long cL = encCountL; encCountL = 0;
    long cR = encCountR; encCountR = 0;
    interrupts();

    // Measured rpm (DT window)
    rpmMeasL = countsToRpm(cL);
    rpmMeasR = countsToRpm(cR);

    // Low-pass filter
    rpmFiltL = RPM_LPF_ALPHA * rpmMeasL + (1.0f - RPM_LPF_ALPHA) * rpmFiltL;
    rpmFiltR = RPM_LPF_ALPHA * rpmMeasR + (1.0f - RPM_LPF_ALPHA) * rpmFiltR;

    // PID -> PWM
    int pwmL = pidStep(targetRpmL, rpmFiltL, iTermL, lastErrL);
    int pwmR = pidStep(targetRpmR, rpmFiltR, iTermR, lastErrR);

    // Output
   setMotor(PWM_L, DIR_L, pwmL, INVERT_L);
   setMotor(PWM_R, DIR_R, pwmR, INVERT_R);

    // Debug print (每 200ms 一次，避免刷屏)
    static uint8_t div = 0;
    if (++div >= 10) { // 10*20ms=200ms
      div = 0;
      Serial.print("T "); Serial.print(targetRpmL, 1); Serial.print(" "); Serial.print(targetRpmR, 1);
      Serial.print(" | M "); Serial.print(rpmFiltL, 1); Serial.print(" "); Serial.print(rpmFiltR, 1);
      Serial.print(" | PWM "); Serial.print(pwmL); Serial.print(" "); Serial.println(pwmR);
    }
  }
}
