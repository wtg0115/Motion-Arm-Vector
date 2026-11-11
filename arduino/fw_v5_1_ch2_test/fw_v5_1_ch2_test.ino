/* ============================================================
   Arm Vector — Arduino 펌웨어 v5.1
   그리퍼 기능 제거 + 기존 그리퍼 서보를 PCA9685 CH2에 연결해 테스트

   배선 변경:
     - 기존 CH2 엘보우 서보 플러그 제거
     - 기존 그리퍼 서보 플러그를 PCA9685 CH2에 연결
     - PCA9685 CH5는 비워 둠

   브라우저 명령 형식은 호환성을 위해 계속 6개:
     ch0,ch1,ch2,ch3,ch4,ch5\n
   단, CH5 값은 완전히 무시하고 PWM도 꺼 둔다.
   CH2는 별도 재매핑 없이 입력 각도를 그대로 사용한다.

   필요 라이브러리:
     Adafruit PWM Servo Driver Library
   ============================================================ */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t NUM_COMMANDS = 6;
const uint8_t LAST_ACTIVE_CHANNEL = 4;  // CH0~CH4만 사용, CH5는 제거

const float SERVO_MAX[NUM_COMMANDS] = {
  280.0f,  // CH0 베이스
  280.0f,  // CH1 숄더
  180.0f,  // CH2: 기존 그리퍼 서보를 엘보우용으로 테스트
  180.0f,  // CH3 손목 pitch
  180.0f,  // CH4 손목 roll
  180.0f   // CH5 미사용
};

// 공통 PWM 범위. 끝에서 버징하면 해당 채널 값을 안쪽으로 줄여야 함.
const uint16_t PULSE_MIN[NUM_COMMANDS] = {110, 110, 110, 110, 110, 110};
const uint16_t PULSE_MAX[NUM_COMMANDS] = {590, 590, 590, 590, 590, 590};

// CH2는 일단 끝단을 피하고 중앙 부근에서만 시험
const float CH2_SAFE_MIN = 30.0f;
const float CH2_SAFE_MAX = 150.0f;

// 사용자가 수동 조종기로 확정한 시작 자세. CH5는 미사용.
const float START_COMMAND[NUM_COMMANDS] = {
  10.0f, 1.0f, 95.0f, 137.5f, 94.0f, 90.0f
};

// 한 루프당 최대 이동량. CH2는 충격 방지를 위해 더 천천히.
const float MAX_STEP_PER_LOOP[NUM_COMMANDS] = {
  1.20f, 0.80f, 0.45f, 1.00f, 1.00f, 0.00f
};

float currentAngle[NUM_COMMANDS];
float targetAngle[NUM_COMMANDS];
bool outputsEnabled = false;

char serialBuffer[96];
uint8_t serialIndex = 0;

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float sanitizeCommand(uint8_t channel, float commandValue) {
  if (channel == 2) {
    // CH2는 재매핑 없이 직접 제어하되 안전 범위만 제한
    return clampFloat(commandValue, CH2_SAFE_MIN, CH2_SAFE_MAX);
  }
  return clampFloat(commandValue, 0.0f, SERVO_MAX[channel]);
}

uint16_t angleToPulse(uint8_t channel, float angle) {
  angle = clampFloat(angle, 0.0f, SERVO_MAX[channel]);
  const float ratio = angle / SERVO_MAX[channel];
  const float pulse = PULSE_MIN[channel]
                    + ratio * (PULSE_MAX[channel] - PULSE_MIN[channel]);
  return (uint16_t)lroundf(pulse);
}

void writeServo(uint8_t channel, float angle) {
  pwm.setPWM(channel, 0, angleToPulse(channel, angle));
}

void disableUnusedCH5() {
  // PCA9685 full-off
  pwm.setPWM(5, 0, 4096);
}

void disableAllServoSignals() {
  for (uint8_t i = 0; i < NUM_COMMANDS; i++) {
    pwm.setPWM(i, 0, 4096);
  }
  outputsEnabled = false;
}

void setTargets(const float commands[NUM_COMMANDS]) {
  for (uint8_t i = 0; i <= LAST_ACTIVE_CHANNEL; i++) {
    targetAngle[i] = sanitizeCommand(i, commands[i]);
  }

  // CH5는 어떤 값이 와도 사용하지 않음
  targetAngle[5] = 90.0f;
  disableUnusedCH5();
  outputsEnabled = true;
}

bool isValidNumberToken(const char *text) {
  if (text == NULL) return false;

  while (*text == ' ' || *text == '\t') text++;
  if (*text == '+' || *text == '-') text++;

  bool hasDigit = false;
  bool hasDot = false;

  while (*text != '\0') {
    if (*text >= '0' && *text <= '9') {
      hasDigit = true;
    } else if (*text == '.' && !hasDot) {
      hasDot = true;
    } else if (*text == ' ' || *text == '\t') {
      while (*text == ' ' || *text == '\t') text++;
      return hasDigit && *text == '\0';
    } else {
      return false;
    }
    text++;
  }

  return hasDigit;
}

void printState(const float commands[NUM_COMMANDS]) {
  Serial.print(F("OK,CH2="));
  Serial.print(targetAngle[2], 1);
  Serial.print(F(",CH5=OFF,INPUT="));
  for (uint8_t i = 0; i < NUM_COMMANDS; i++) {
    if (i > 0) Serial.print(',');
    Serial.print(commands[i], 1);
  }
  Serial.println();
}

void parseSerialLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;

  if ((line[0] == 'C' || line[0] == 'c') && line[1] == '\0') {
    setTargets(START_COMMAND);
    Serial.println(F("START_POSE_CH5_OFF"));
    return;
  }

  if ((line[0] == 'X' || line[0] == 'x') && line[1] == '\0') {
    disableAllServoSignals();
    Serial.println(F("OUTPUT_OFF"));
    return;
  }

  float commands[NUM_COMMANDS];
  uint8_t count = 0;

  char *token = strtok(line, ",");
  while (token != NULL && count < NUM_COMMANDS) {
    if (!isValidNumberToken(token)) {
      Serial.println(F("ERR_NUMBER"));
      return;
    }
    commands[count++] = (float)atof(token);
    token = strtok(NULL, ",");
  }

  if (count != NUM_COMMANDS || token != NULL) {
    Serial.println(F("ERR_COUNT"));
    return;
  }

  setTargets(commands);
  printState(commands);
}

float moveToward(float current, float target, float maxStep) {
  const float difference = target - current;
  if (fabsf(difference) <= maxStep) return target;
  return current + (difference > 0.0f ? maxStep : -maxStep);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(20);

  for (uint8_t i = 0; i < NUM_COMMANDS; i++) {
    currentAngle[i] = START_COMMAND[i];
    targetAngle[i] = START_COMMAND[i];
  }

  // 첫 브라우저 명령 전까지 어떤 서보도 움직이지 않음
  disableAllServoSignals();

  Serial.println(F("READY_V5_GRIPPER_REMOVED_CH2_DIRECT"));
  Serial.println(F("CH2 direct safe range: 30-150 deg"));
  Serial.println(F("CH5 PWM: OFF"));
}

void loop() {
  while (Serial.available() > 0) {
    const char incoming = (char)Serial.read();

    if (incoming == '\n' || incoming == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        parseSerialLine(serialBuffer);
        serialIndex = 0;
      }
    } else if (serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = incoming;
    } else {
      serialIndex = 0;
      Serial.println(F("ERR_BUFFER"));
    }
  }

  if (outputsEnabled) {
    for (uint8_t i = 0; i <= LAST_ACTIVE_CHANNEL; i++) {
      currentAngle[i] = moveToward(
        currentAngle[i],
        targetAngle[i],
        MAX_STEP_PER_LOOP[i]
      );
      writeServo(i, currentAngle[i]);
    }
    disableUnusedCH5();
  }

  delay(15);
}
