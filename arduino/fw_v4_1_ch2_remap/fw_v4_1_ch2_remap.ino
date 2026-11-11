/* ============================================================
   Arm Vector — Arduino 펌웨어 v4.1 (CH2 엘보우 재매핑 · AVR 호환)
   ------------------------------------------------------------
   현재 수동조종기와 함께 사용:
     05_manual_controller_v4_CH2직결.html

   핵심 수정:
   1) CH2 명령 0도를 서보 물리 0도 끝점으로 보내지 않음
   2) CH2 명령 0도 = 엘보우 기준 위치(CH2_HOME_PHYSICAL)
   3) CH2 명령이 증가하면 기준 위치에서 굽힘 방향으로 이동
   4) CH2는 물리 안전범위 안에서만 움직임
   5) 부팅할 때 전체 서보를 중앙으로 강제 이동하지 않음

   수동조종기가 보내는 형식:
     ch0,ch1,ch2,ch3,ch4,ch5\n
   예:
     10,94,0,137,94,90

   필요 라이브러리:
     Adafruit PWM Servo Driver Library
   ============================================================ */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t NUM_SERVOS = 6;

// 브라우저에서 받을 수 있는 논리 명령 범위
const float COMMAND_MAX[NUM_SERVOS] = {
  280.0f,  // CH0 베이스
  280.0f,  // CH1 숄더
  180.0f,  // CH2 엘보우 굽힘량
  180.0f,  // CH3 손목 pitch
  180.0f,  // CH4 손목 roll
  180.0f   // CH5 그리퍼
};

// 각 서보의 물리 각도 범위
const float PHYSICAL_MAX[NUM_SERVOS] = {
  280.0f, 280.0f, 180.0f, 180.0f, 180.0f, 180.0f
};

// PCA9685 펄스 범위. 현재 쓰던 값 유지.
// 특정 서보가 끝에서 버징하면 해당 채널만 안쪽으로 줄여야 함.
const uint16_t PULSE_MIN[NUM_SERVOS] = {
  110, 110, 110, 110, 110, 110
};
const uint16_t PULSE_MAX[NUM_SERVOS] = {
  590, 590, 590, 590, 590, 590
};

// ============================================================
// CH2 전용 설정
// ============================================================

// 수동조종기 CH2가 0일 때 실제 서보가 갈 물리 각도.
// 기존에는 0 → 물리 0도로 가서 한쪽 끝에 붙었음.
// 이제 0 → 물리 90도로 보내 시작점부터 움직일 여유를 만듦.
const float CH2_HOME_PHYSICAL = 90.0f;

// CH2 방향. 반대로 접히면 +1을 -1로 바꾸고 다시 업로드.
const int8_t CH2_DIRECTION = +1;

// 수동조종기 CH2 1도당 실제 서보 이동량.
// 0.8이면 조종기 10도 변화 → 실제 서보 약 8도 변화.
const float CH2_GAIN = 0.80f;

// 조립된 상태에서 끝점 충돌을 피하기 위한 물리 제한
const float CH2_SAFE_MIN = 18.0f;
const float CH2_SAFE_MAX = 162.0f;

// 현재 확인된 자세를 부팅 시 위치 추정값으로 사용.
// 부팅만으로는 PWM을 출력하지 않고, 첫 명령이 들어온 뒤부터 움직임.
// CH2는 기존 코드에서 물리 0도 끝에 있었던 것으로 추정.
const float BOOT_ESTIMATE_PHYSICAL[NUM_SERVOS] = {
  10.0f, 94.0f, 0.0f, 137.0f, 94.0f, 90.0f
};

// 'C' 명령을 받을 때 이동할 논리 시작 자세
const float START_COMMAND[NUM_SERVOS] = {
  10.0f, 94.0f, 0.0f, 137.0f, 94.0f, 90.0f
};

// 한 번의 loop에서 움직일 수 있는 최대 물리 각도.
// CH2를 더 천천히 움직여 링크 충격을 줄임.
const float MAX_STEP_PER_LOOP[NUM_SERVOS] = {
  1.20f, 0.80f, 0.45f, 1.00f, 1.00f, 1.00f
};

float currentPhysical[NUM_SERVOS];
float targetPhysical[NUM_SERVOS];
bool outputsEnabled = false;

char serialBuffer[96];
uint8_t serialIndex = 0;

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

// 브라우저의 논리 명령을 실제 서보 물리 각도로 변환
float commandToPhysical(uint8_t channel, float commandValue) {
  commandValue = clampFloat(commandValue, 0.0f, COMMAND_MAX[channel]);

  if (channel == 2) {
    // CH2 명령 0 = 물리 기준각, 이후 값은 굽힘 변화량
    float physical = CH2_HOME_PHYSICAL
                   + (float)CH2_DIRECTION * commandValue * CH2_GAIN;
    return clampFloat(physical, CH2_SAFE_MIN, CH2_SAFE_MAX);
  }

  // 나머지 채널은 기존처럼 1:1
  return clampFloat(commandValue, 0.0f, PHYSICAL_MAX[channel]);
}

uint16_t physicalAngleToPulse(uint8_t channel, float physicalAngle) {
  physicalAngle = clampFloat(
    physicalAngle,
    0.0f,
    PHYSICAL_MAX[channel]
  );

  const float ratio = physicalAngle / PHYSICAL_MAX[channel];
  const float pulse = PULSE_MIN[channel]
                    + ratio * (PULSE_MAX[channel] - PULSE_MIN[channel]);

  return (uint16_t)lroundf(pulse);
}

void writePhysicalServo(uint8_t channel, float physicalAngle) {
  pwm.setPWM(channel, 0, physicalAngleToPulse(channel, physicalAngle));
}

void disableAllServoSignals() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    // PCA9685 full-off: 부팅 직후 임의 중앙 이동 방지
    pwm.setPWM(i, 0, 4096);
  }
  outputsEnabled = false;
}

void setTargetsFromCommands(const float commands[NUM_SERVOS]) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    targetPhysical[i] = commandToPhysical(i, commands[i]);
  }
  outputsEnabled = true;
}

void moveToStartCommand() {
  setTargetsFromCommands(START_COMMAND);
  Serial.println(F("START_POSE"));
}

void printSettings() {
  Serial.print(F("CH2_HOME="));
  Serial.print(CH2_HOME_PHYSICAL, 1);
  Serial.print(F(",DIR="));
  Serial.print(CH2_DIRECTION);
  Serial.print(F(",GAIN="));
  Serial.print(CH2_GAIN, 2);
  Serial.print(F(",SAFE="));
  Serial.print(CH2_SAFE_MIN, 1);
  Serial.print('-');
  Serial.println(CH2_SAFE_MAX, 1);
}

// Arduino Uno(AVR)에서도 확실히 동작하는 단순 실수 문자열 검사
// 브라우저는 10, 94.5 같은 일반 숫자만 보내므로 지수표기까지는 받지 않음.
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

void parseSerialLine(char *line) {
  // 앞뒤 공백 제거
  while (*line == ' ' || *line == '\t') line++;

  if ((line[0] == 'C' || line[0] == 'c') && line[1] == '\0') {
    moveToStartCommand();
    return;
  }

  if ((line[0] == 'X' || line[0] == 'x') && line[1] == '\0') {
    disableAllServoSignals();
    Serial.println(F("OUTPUT_OFF"));
    return;
  }

  if ((line[0] == 'R' || line[0] == 'r') && line[1] == '\0') {
    printSettings();
    return;
  }

  float commands[NUM_SERVOS];
  uint8_t count = 0;

  char *token = strtok(line, ",");
  while (token != NULL && count < NUM_SERVOS) {
    if (!isValidNumberToken(token)) {
      Serial.println(F("ERR_NUMBER"));
      return;
    }

    // strtof는 일부 Arduino Uno AVR 환경에서 선언되지 않아 atof 사용
    const float value = (float)atof(token);
    commands[count++] = value;
    token = strtok(NULL, ",");
  }

  // 숫자가 정확히 6개여야 함. 7번째 값이 있어도 거부.
  if (count != NUM_SERVOS || token != NULL) {
    Serial.println(F("ERR_COUNT"));
    return;
  }

  setTargetsFromCommands(commands);

  Serial.print(F("OK,CH2_LOGICAL="));
  Serial.print(commands[2], 1);
  Serial.print(F(",CH2_PHYSICAL="));
  Serial.println(targetPhysical[2], 1);
}

float moveToward(float current, float target, float maxStep) {
  const float difference = target - current;

  if (fabsf(difference) <= maxStep) {
    return target;
  }

  return current + (difference > 0.0f ? maxStep : -maxStep);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(20);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentPhysical[i] = BOOT_ESTIMATE_PHYSICAL[i];
    targetPhysical[i] = BOOT_ESTIMATE_PHYSICAL[i];
  }

  // 기존 펌웨어처럼 부팅 즉시 중앙으로 보내지 않음.
  disableAllServoSignals();

  Serial.println(F("READY_V4_CH2_REMAP"));
  printSettings();
  Serial.println(F("첫 6축 명령을 받을 때까지 PWM 출력 안 함"));
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
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      currentPhysical[i] = moveToward(
        currentPhysical[i],
        targetPhysical[i],
        MAX_STEP_PER_LOOP[i]
      );
      writePhysicalServo(i, currentPhysical[i]);
    }
  }

  delay(15);
}
