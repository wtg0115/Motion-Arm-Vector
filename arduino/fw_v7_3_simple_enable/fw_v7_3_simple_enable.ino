/*
  Arm Vector firmware v7.3 — unified-pose first live test, single-click PWM enable

  v7.2 → v7.3 (2026-07-15)
    - PWM 활성화를 8초 2단계 → 단일 클릭 E로 단순화. 2단계는 '확인하는 사이 8초 창이
      만료되어 PWM이 영영 안 켜지는' 문제를 일으켰다. 실질 안전게이트는 외부 서보전원
      스위치이고, 이동은 속도·가속도·중력·범위 제한이 잡으며, X로 즉시 차단 가능하다.
    - 나머지(첫 실기 STOW±제한, 중력가드, 실측 보정, 데드맨, 5각도 프로토콜)는 v7.2 유지.

  Physical channels
    CH0 : base rotation       / 25 kg servo / nominal 280 deg
    CH1 : shoulder up-down    / 35 kg servo / nominal 280 deg
    CH2 : elbow flexion       / relocated 20 kg servo / nominal 180 deg
    CH3 : wrist up-down       / 20 kg servo / nominal 180 deg
    CH4 : wrist axis rotation / 20 kg servo / nominal 180 deg

  Serial protocol, 115200 baud
    "a0,a1,a2,a3,a4\n" : exactly five target angles in degrees
    E : enable PWM at STOW immediately (single click)
    X : disable all PCA9685 outputs immediately (emergency stop)
    C : command calibrated STOW pose
    N : command calibrated teleoperation neutral pose
    P : print current/target status

  IMPORTANT
    This file boots with all PCA9685 outputs OFF even though calibration is ready.
    A single E enables PWM; keep servo external power OFF until the arm is supported
    and near STOW. All calibration values must remain finite and internally valid.

  Required library
    Adafruit PWM Servo Driver Library
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

static const uint8_t AXIS_COUNT = 5;
static const uint8_t PCA_OUTPUT_COUNT = 16;
static const float PWM_FREQ_HZ = 50.0f;
static const uint32_t PCA_OSCILLATOR_HZ = 27000000UL;

// ┌──────────────────── 사용자 실측 보정 구역 ────────────────────┐
// 2026-07-15 재튜닝 전면 반영 (manual_calibration v1.1 REPORT + 산술검산 통과).
// 15점 deg↔pulse↔us 최대오차 0.04cnt/0.6us. 모든 축 CENTER=1500us 기계중심.
const bool HARDWARE_CALIBRATION_READY = true;

const float SERVO_RANGE_DEG[AXIS_COUNT] = {280, 280, 180, 180, 180};

// 500us~2500us 창을 각 채널 명목범위에 매핑 (50Hz, 4.889us/count → 102.3~511.4).
// 채널 공통. 채널별 차이는 아래 deg 값이 담는다.
const float PULSE_MIN_COUNT[AXIS_COUNT] = {102.27, 102.27, 102.27, 102.27, 102.27};
const float PULSE_MAX_COUNT[AXIS_COUNT] = {511.36, 511.36, 511.36, 511.36, 511.36};
const bool SERVO_REVERSED[AXIS_COUNT] = {false, false, false, false, false};

// 실측 기구 명령각 (MARK MIN/MAX). 사용자가 이미 ~5deg 충돌여유를 포함해 설정함.
const float HARD_MIN_DEG[AXIS_COUNT] = {55.0, 97.0, 40.0, 35.0, 5.0};
const float HARD_MAX_DEG[AXIS_COUNT] = {225.0, 215.0, 175.0, 150.0, 175.0};
// TELEOP = HARD 안쪽 3deg (끝단 버징 방지). 시연에서 완전전개/완전트위스트가
// 필요하면 이 두 줄을 HARD와 동일하게 바꾼다.
const float TELEOP_MIN_DEG[AXIS_COUNT] = {58.0, 100.0, 43.0, 38.0, 8.0};
const float TELEOP_MAX_DEG[AXIS_COUNT] = {222.0, 212.0, 172.0, 147.0, 172.0};

// ★ 첫 실기 제한모드 ★
// 방향·배선·간섭 확인이 끝날 때까지 STOW 주변 작은 범위만 허용한다.
// index-v10.0의 같은 상수와 반드시 함께 변경한다.
const bool FIRST_LIVE_TEST_MODE = true;
const float FIRST_LIVE_TEST_DELTA[AXIS_COUNT] = {10.0, 10.0, 10.0, 8.0, 0.0};
// STOW = NEUTRAL = 로봇 연동(참조) 자세. CH2만 135deg(사람 팔꿈치 45deg 대응), 나머지는 중심.
const float STOW_POSE_DEG[AXIS_COUNT] = {140.0, 140.0, 135.0, 90.0, 90.0};
const float TELEOP_NEUTRAL_DEG[AXIS_COUNT] = {140.0, 140.0, 135.0, 90.0, 90.0};

// 중력 시뮬레이션 결과를 반영한 보수적 초기 제한.
// CH1과 CH2를 가장 느리게 두고 실기 확인 후 10% 단위로 조정한다.
const float MAX_VEL_DEG_S[AXIS_COUNT] = {45, 28, 42, 55, 55};
const float MAX_ACCEL_DEG_S2[AXIS_COUNT] = {160, 100, 150, 220, 220};

// 실측 링크 무게중심 기반 동적 감속 보호. CH1 앵커 미측정 동안 최대부하 폴백.
const bool USE_GRAVITY_MOTION_GUARD = true;
const float GRAVITY_GUARD_MIN_SCALE = 0.35f;

// ★ CH1 상완의 '수평/수직' 서보각은 아직 미측정이다. false인 동안 중력가드는
//   CH1을 항상 수평(최대토크)으로 간주해 보수적으로 감속한다(절대 덜 잡지 않음).
//   조립 5축 실기 전, CH1을 몰아 상완 수평/수직일 때 각도 2개를 측정해
//   CH1_HORIZONTAL_DEG/CH1_VERTICAL_DEG에 넣고 이 플래그를 true로 바꾼다.
const bool  CH1_GRAVITY_CAL_READY = false;
const float CH1_HORIZONTAL_DEG_PROV = 140.0f;   // 잠정(플래그 false면 이 값으로 고정 간주)
const float CH1_VERTICAL_DEG_PROV   = 215.0f;   // 잠정(플래그 true 전까지 미사용)
// └───────────────────────────────────────────────────────────────┘

static const uint32_t CONTROL_PERIOD_US = 10000;
static const uint32_t COMMAND_TIMEOUT_MS = 900;       // 1단계: 현재 자세 홀드
static const uint32_t DEADMAN_STOW_MS = 5000;          // 2단계: STOW 복귀 시작
static const uint32_t DEADMAN_OFF_MS = 15000;          // 3단계: STOW 도달 후 PWM OFF

float currentDeg[AXIS_COUNT];
float targetDeg[AXIS_COUNT];
float velocityDegS[AXIS_COUNT];
bool pwmEnabled = false;
uint32_t lastControlUs = 0;
uint32_t lastCommandMs = 0;
bool deadmanReturningToStow = false;

char lineBuffer[96];
uint8_t lineLength = 0;

float clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

bool finiteArray(const float values[AXIS_COUNT]) {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (!isfinite(values[i])) return false;
  }
  return true;
}

bool configurationValid() {
  if (!HARDWARE_CALIBRATION_READY) return false;
  if (!finiteArray(PULSE_MIN_COUNT) || !finiteArray(PULSE_MAX_COUNT) ||
      !finiteArray(HARD_MIN_DEG) || !finiteArray(HARD_MAX_DEG) ||
      !finiteArray(TELEOP_MIN_DEG) || !finiteArray(TELEOP_MAX_DEG) ||
      !finiteArray(STOW_POSE_DEG) || !finiteArray(TELEOP_NEUTRAL_DEG) ||
      !finiteArray(FIRST_LIVE_TEST_DELTA)) return false;

  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    if (SERVO_RANGE_DEG[ch] <= 0.0f) return false;
    if (PULSE_MIN_COUNT[ch] < 0.0f || PULSE_MAX_COUNT[ch] > 4095.0f ||
        PULSE_MIN_COUNT[ch] >= PULSE_MAX_COUNT[ch]) return false;
    if (HARD_MIN_DEG[ch] >= HARD_MAX_DEG[ch]) return false;
    if (TELEOP_MIN_DEG[ch] < HARD_MIN_DEG[ch] || TELEOP_MAX_DEG[ch] > HARD_MAX_DEG[ch] ||
        TELEOP_MIN_DEG[ch] > TELEOP_NEUTRAL_DEG[ch] || TELEOP_NEUTRAL_DEG[ch] > TELEOP_MAX_DEG[ch]) return false;
    if (STOW_POSE_DEG[ch] < HARD_MIN_DEG[ch] || STOW_POSE_DEG[ch] > HARD_MAX_DEG[ch]) return false;
    if (FIRST_LIVE_TEST_DELTA[ch] < 0.0f) return false;
    if (FIRST_LIVE_TEST_MODE) {
      const float liveMin = fmaxf(TELEOP_MIN_DEG[ch], STOW_POSE_DEG[ch] - FIRST_LIVE_TEST_DELTA[ch]);
      const float liveMax = fminf(TELEOP_MAX_DEG[ch], STOW_POSE_DEG[ch] + FIRST_LIVE_TEST_DELTA[ch]);
      if (liveMin > STOW_POSE_DEG[ch] || STOW_POSE_DEG[ch] > liveMax) return false;
    }
  }
  return true;
}

void disablePcaOutput(uint8_t channel) {
  pwm.setPWM(channel, 0, 0);
}

void disableAllOutputs() {
  pwmEnabled = false;
  deadmanReturningToStow = false;
  // 모든 16개 PCA 출력 비활성화. 사용하지 않는 소켓에 서보가 꽂혀도 신호가 나오지 않는다.
  for (uint8_t ch = 0; ch < PCA_OUTPUT_COUNT; ++ch) disablePcaOutput(ch);
  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) velocityDegS[ch] = 0.0f;
}

uint16_t angleToCount(uint8_t ch, float angleDeg) {
  float normalized = clampf(angleDeg, 0.0f, SERVO_RANGE_DEG[ch]) / SERVO_RANGE_DEG[ch];
  if (SERVO_REVERSED[ch]) normalized = 1.0f - normalized;
  const float count = PULSE_MIN_COUNT[ch] + normalized * (PULSE_MAX_COUNT[ch] - PULSE_MIN_COUNT[ch]);
  return (uint16_t)clampf(count, 0.0f, 4095.0f);
}

void writeOutputs() {
  if (!pwmEnabled || !configurationValid()) return;
  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    pwm.setPWM(ch, 0, angleToCount(ch, currentDeg[ch]));
  }
  // CH0~CH4 이외의 모든 출력은 매 주기 OFF 상태를 유지한다.
  for (uint8_t ch = AXIS_COUNT; ch < PCA_OUTPUT_COUNT; ++ch) disablePcaOutput(ch);
}

float commandMin(uint8_t ch) {
  float low = fmaxf(HARD_MIN_DEG[ch], TELEOP_MIN_DEG[ch]);
  if (FIRST_LIVE_TEST_MODE) low = fmaxf(low, STOW_POSE_DEG[ch] - FIRST_LIVE_TEST_DELTA[ch]);
  return low;
}

float commandMax(uint8_t ch) {
  float high = fminf(HARD_MAX_DEG[ch], TELEOP_MAX_DEG[ch]);
  if (FIRST_LIVE_TEST_MODE) high = fminf(high, STOW_POSE_DEG[ch] + FIRST_LIVE_TEST_DELTA[ch]);
  return high;
}

void loadPose(const float pose[AXIS_COUNT], bool teleopLimits) {
  if (!configurationValid()) return;
  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    const float low = teleopLimits ? commandMin(ch) : HARD_MIN_DEG[ch];
    const float high = teleopLimits ? commandMax(ch) : HARD_MAX_DEG[ch];
    targetDeg[ch] = clampf(pose[ch], low, high);
  }
  deadmanReturningToStow = false;
  lastCommandMs = millis();
}

bool enableAtStow() {
  if (!configurationValid()) {
    disableAllOutputs();
    Serial.println(F("ERR,CALIBRATION_REQUIRED"));
    return false;
  }
  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    const float safe = clampf(STOW_POSE_DEG[ch], HARD_MIN_DEG[ch], HARD_MAX_DEG[ch]);
    currentDeg[ch] = safe;
    targetDeg[ch] = safe;
    velocityDegS[ch] = 0.0f;
  }
  deadmanReturningToStow = false;
  pwmEnabled = true;
  writeOutputs();
  lastCommandMs = millis();
  return true;
}

void applyForbiddenPoseRules(float values[AXIS_COUNT]) {
  // 실제 저속 충돌시험에서 확인한 조합만 이 함수에 추가한다.
  // 추측 규칙은 넣지 않는다.
  (void)values;
}

float gravityMotionScale(const float values[AXIS_COUNT]) {
  if (!USE_GRAVITY_MOTION_GUARD || !configurationValid()) return 1.0f;

  // ══════════════════════════════════════════════════════════
  // v7.0 중력 가드 반전 수정 (2026-07-15) — index v9.3과 동일 모델
  // ══════════════════════════════════════════════════════════
  // [옛 v6.0 버그]  q1 = |CH1 - TELEOP_NEUTRAL[1]| ; risk ∝ sin(q1)
  //   "TELEOP_NEUTRAL(=177°)이 수직이고 무부하"라고 가정했다.
  //   그러나 실측 결과 CH1 177° = 상완 '수평' = 중력토크 최대다.
  //   → STOW(최대부하)에서 풀스피드, 수직(무부하)에서 감속. 부호가 반대였다.
  //     35kg 서보가 죽고 PCA9685 전원부가 탄 축이 바로 여기다.
  //
  // [재튜닝 앵커 2026-07-15]  CH2 175°=일직선 / 40°=최대굽힘 (실측).
  //   CH1 수평/수직은 아직 미측정 → CH1_GRAVITY_CAL_READY=false 동안 항상 수평 간주.
  const float CH1_HORIZONTAL = CH1_HORIZONTAL_DEG_PROV, CH1_VERTICAL = CH1_VERTICAL_DEG_PROV;
  const float CH2_STRAIGHT = 175.00f,  CH2_MINCMD = 40.00f;
  const float L1 = 11.6f, L2 = 14.8f, L3 = 7.0f;   // cm, 축간거리
  const float m1 = 140.0f, m2 = 140.0f, m3 = 100.0f;  // g, 집중질량 추정
  const float WORST_COM_CM = 22.8f;                // 수평+완전전개 기준

  // CH1 미교정 시 상완을 항상 수평(최대토크)으로 간주 = 절대 덜 감속하지 않음.
  const float ch1EffForGrav = CH1_GRAVITY_CAL_READY ? values[1] : CH1_HORIZONTAL;
  const float elevDeg = (ch1EffForGrav - CH1_HORIZONTAL) * 90.0f / (CH1_VERTICAL - CH1_HORIZONTAL);
  const float flexDeg = (CH2_STRAIGHT - values[2]) * 115.0f / (CH2_STRAIGHT - CH2_MINCMD);
  const float a = elevDeg * DEG_TO_RAD;
  const float f = flexDeg * DEG_TO_RAD;            // 전완은 상완에서 위로 접힌다
  const float P2x = L1 * cosf(a);
  const float P3x = P2x + L2 * cosf(a + f);
  const float P4x = P3x + L3 * cosf(a + f);
  const float M = m1 + m2 + m3;
  const float com = fabsf((m1 * P2x + m2 * P3x + m3 * P4x) / M);
  const float load = clampf(com / WORST_COM_CM, 0.0f, 1.0f);
  return clampf(1.0f - 0.65f * load, GRAVITY_GUARD_MIN_SCALE, 1.0f);
}

bool parseAngles(char *line, float parsed[AXIS_COUNT]) {
  uint8_t count = 0;
  char *save = nullptr;
  char *token = strtok_r(line, ",", &save);
  while (token != nullptr && count < AXIS_COUNT) {
    char *endPtr = nullptr;
    // AVR avr-libc 구버전에는 strtof가 없다. strtod는 항상 있고 AVR에선 double==float(32bit)라 동일.
    const float value = (float)strtod(token, &endPtr);
    while (*endPtr == ' ' || *endPtr == '\t') ++endPtr;
    if (endPtr == token || *endPtr != '\0' || !isfinite(value)) return false;
    parsed[count++] = value;
    token = strtok_r(nullptr, ",", &save);
  }
  return count == AXIS_COUNT && token == nullptr;
}

void acceptAngleCommand(float values[AXIS_COUNT]) {
  if (!configurationValid()) return;
  applyForbiddenPoseRules(values);
  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    targetDeg[ch] = clampf(values[ch], commandMin(ch), commandMax(ch));
  }
  deadmanReturningToStow = false;
  lastCommandMs = millis();
}

void printStatus() {
  Serial.print(F("STATE,"));
  Serial.print(pwmEnabled ? F("ENABLED") : F("OFF"));
  Serial.print(configurationValid() ? F(",CAL_OK") : F(",CAL_LOCKED"));
  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    Serial.print(',');
    if (isfinite(currentDeg[ch])) Serial.print(currentDeg[ch], 1); else Serial.print(F("NA"));
    Serial.print('/');
    if (isfinite(targetDeg[ch])) Serial.print(targetDeg[ch], 1); else Serial.print(F("NA"));
  }
  Serial.print(FIRST_LIVE_TEST_MODE ? F(",TEST_LIMITED") : F(",FULL_RANGE"));
  Serial.println();
}

void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return;

  if (line[1] == '\0') {
    switch (line[0]) {
      case 'E': case 'e': {
        // v7.3: 단일 단계 활성화. 8초 2단계 창은 '조심히 확인하다 창 만료 → PWM 영영 안 켜짐'
        //   문제를 일으켜 제거했다. 실질 안전게이트는 외부 서보전원 스위치이고,
        //   이동 자체는 속도·가속도·중력·범위 제한이 잡는다. 오작동 시 X로 즉시 차단.
        if (!configurationValid()) {
          disableAllOutputs();
          Serial.println(F("ERR,CALIBRATION_REQUIRED"));
          return;
        }
        if (enableAtStow()) Serial.println(F("ENABLED_5AXIS"));
        return;
      }
      case 'X': case 'x':
        disableAllOutputs();
        Serial.println(F("PWM_OFF"));
        return;
      case 'C': case 'c':
        if (!configurationValid()) Serial.println(F("ERR,CALIBRATION_REQUIRED"));
        else { loadPose(STOW_POSE_DEG, false); Serial.println(F("STOW")); }
        return;
      case 'N': case 'n':
        if (!configurationValid()) Serial.println(F("ERR,CALIBRATION_REQUIRED"));
        else { loadPose(TELEOP_NEUTRAL_DEG, true); Serial.println(F("NEUTRAL")); }
        return;
      case 'P': case 'p':
        printStatus();
        return;
      default:
        Serial.println(F("ERR,UNKNOWN_COMMAND"));
        return;
    }
  }

  float parsed[AXIS_COUNT];
  if (!parseAngles(line, parsed)) {
    Serial.println(F("ERR,EXPECTED_EXACTLY_5_ANGLES"));
    return;
  }
  if (!configurationValid()) {
    Serial.println(F("ERR,CALIBRATION_REQUIRED"));
    return;
  }
  if (!pwmEnabled) {
    Serial.println(F("ERR,PWM_OFF_SEND_E"));
    return;
  }
  acceptAngleCommand(parsed);
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      handleLine(lineBuffer);
      lineLength = 0;
    } else if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineLength = 0;
      Serial.println(F("ERR,LINE_TOO_LONG"));
    }
  }
}

void updateMotion() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;
  const float dt = clampf((nowUs - lastControlUs) / 1000000.0f, 0.001f, 0.05f);
  lastControlUs = nowUs;

  if (pwmEnabled) {
    const uint32_t silentMs = millis() - lastCommandMs;
    if (silentMs > DEADMAN_STOW_MS) {
      if (!deadmanReturningToStow) {
        deadmanReturningToStow = true;
        Serial.println(F("DEADMAN_STOW"));
      }
      for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
        targetDeg[ch] = clampf(STOW_POSE_DEG[ch], commandMin(ch), commandMax(ch));
      }
    } else if (silentMs > COMMAND_TIMEOUT_MS && !deadmanReturningToStow) {
      for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
        targetDeg[ch] = currentDeg[ch];
        velocityDegS[ch] = 0.0f;
      }
    }
  }

  if (!pwmEnabled || !configurationValid()) return;
  const float loadScale = gravityMotionScale(targetDeg);

  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    const float axisScale = (ch == 1 || ch == 2) ? loadScale : 1.0f;
    const float error = targetDeg[ch] - currentDeg[ch];
    float desiredVel = clampf(error / dt,
                              -MAX_VEL_DEG_S[ch] * axisScale,
                               MAX_VEL_DEG_S[ch] * axisScale);
    const float maxDv = MAX_ACCEL_DEG_S2[ch] * axisScale * dt;
    velocityDegS[ch] += clampf(desiredVel - velocityDegS[ch], -maxDv, maxDv);

    float step = velocityDegS[ch] * dt;
    if (fabsf(step) > fabsf(error)) {
      step = error;
      velocityDegS[ch] = 0.0f;
    }
    currentDeg[ch] = clampf(currentDeg[ch] + step, HARD_MIN_DEG[ch], HARD_MAX_DEG[ch]);
  }
  writeOutputs();

  if (deadmanReturningToStow && millis() - lastCommandMs > DEADMAN_OFF_MS) {
    bool atStow = true;
    for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
      if (fabsf(currentDeg[ch] - STOW_POSE_DEG[ch]) > 0.8f) { atStow = false; break; }
    }
    if (atStow) {
      disableAllOutputs();
      Serial.println(F("DEADMAN_PWM_OFF"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(PCA_OSCILLATOR_HZ);
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(20);

  for (uint8_t ch = 0; ch < AXIS_COUNT; ++ch) {
    currentDeg[ch] = NAN;
    targetDeg[ch] = NAN;
    velocityDegS[ch] = 0.0f;
  }
  disableAllOutputs();
  lastControlUs = micros();
  lastCommandMs = millis();
  if (configurationValid()) {
    Serial.println(F("READY_5AXIS_PWM_OFF"));
    if (FIRST_LIVE_TEST_MODE) Serial.println(F("INFO,FIRST_LIVE_TEST_LIMITED"));
  } else {
    Serial.println(F("READY_5AXIS_CALIBRATION_LOCKED"));
  }
}

void loop() {
  pollSerial();
  updateMotion();
}
