/* ============================================================
   Arm Vector — Arduino 펌웨어 v2 (PCA9685 + 서보 6개)
   ------------------------------------------------------------
   변경점 (v1 → v2):
   - 부팅하면 모든 서보를 각자의 '중앙'으로 자동 정렬 (센터링)
   - 각 서보의 가동범위(range)를 기준으로 중앙을 자동 계산
   - 시리얼로 'C' 한 글자 보내면 언제든 다시 센터링
   - 로봇에서 서보 분리한 상태로 켜면 → 전부 중앙 위치로 맞춰짐
     → 그 상태로 로봇에 조립하면 물리 중앙이 맞음

   역할:
   - 평소: USB 시리얼로 "a0,a1,a2,a3,a4,a5\n" 각도 받아 서보 구동
   - 'C\n' 수신: 즉시 전체 센터링
   - 부팅 직후: 자동 센터링

   필요 라이브러리: "Adafruit PWM Servo Driver Library"

   배선:
     아두이노 SDA(A4) ── PCA9685 SDA
     아두이노 SCL(A5) ── PCA9685 SCL
     아두이노 GND ───── PCA9685 GND
     아두이노 5V ────── PCA9685 VCC (로직 전원)
     [5V 10A 어댑터](+) ─ PCA9685 V+ 단자
     [5V 10A 어댑터](−) ─ PCA9685 GND
     서보 6개 ───────── PCA9685 채널 0~5
   ============================================================ */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t NUM_SERVOS = 6;

// 펄스 범위 (PCA9685 카운트) — 서보 물리 한계
const uint16_t SERVOMIN = 110;   // 약 0.5ms
const uint16_t SERVOMAX = 590;   // 약 2.5ms

// ── 각 서보의 가동 각도 범위 (최고 각도) ──
// 베이스만 270, 나머지 180. 네 키트 구성에 맞게 수정.
const uint16_t SERVO_RANGE[NUM_SERVOS] = { 270, 180, 180, 180, 180, 180 };

// ── 각 서보의 중앙 = range의 절반 (자동 계산) ──
// 270 → 135, 180 → 90. 코드가 알아서 계산하므로 손댈 필요 없음.
float servoCenter(uint8_t i) {
  return SERVO_RANGE[i] / 2.0;
}

// 현재/목표 각도
float curAngle[NUM_SERVOS];
float targetAngle[NUM_SERVOS];

// 스무딩 계수 (0~1, 클수록 빠름)
const float SMOOTH = 0.25;

// 센터링 이동용 (부팅/명령 시 천천히 중앙으로)
const float CENTER_SMOOTH = 0.08;   // 더 부드럽게 (서보 분리상태 안전)
bool centering = false;
unsigned long centerStart = 0;

char buf[64];
uint8_t bufIdx = 0;

// 각도 → PCA9685 펄스
uint16_t angleToPulse(uint8_t i, float angle) {
  float range = (float)SERVO_RANGE[i];
  if (angle < 0) angle = 0;
  if (angle > range) angle = range;
  return (uint16_t)(SERVOMIN + (angle / range) * (SERVOMAX - SERVOMIN));
}

void writeServo(uint8_t i, float angle) {
  pwm.setPWM(i, 0, angleToPulse(i, angle));
}

// ── 센터링 시작: 모든 서보 목표를 중앙으로 ──
void startCentering() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    targetAngle[i] = servoCenter(i);
  }
  centering = true;
  centerStart = millis();
  Serial.println("CENTERING");
}

// 시리얼 한 줄 파싱
void parseLine(char *line) {
  // 'C' 한 글자면 센터링 명령
  if ((line[0] == 'C' || line[0] == 'c') && (line[1] == '\0')) {
    startCentering();
    return;
  }

  float vals[NUM_SERVOS];
  uint8_t n = 0;
  char *tok = strtok(line, ",");
  while (tok != NULL && n < NUM_SERVOS) {
    vals[n++] = atof(tok);
    tok = strtok(NULL, ",");
  }
  if (n == NUM_SERVOS) {
    // 새 각도 명령이 오면 센터링 모드 해제 (사용자 제어 우선)
    centering = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      float a = vals[i];
      if (a < 0) a = 0;
      if (a > SERVO_RANGE[i]) a = SERVO_RANGE[i];
      targetAngle[i] = a;
    }
    Serial.println("OK");
  } else {
    Serial.println("ERR");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(20);

  // 현재 각도를 중앙으로 초기화하고 즉시 그 위치 출력
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    curAngle[i] = servoCenter(i);
    targetAngle[i] = servoCenter(i);
    writeServo(i, curAngle[i]);
  }

  Serial.println("READY");
  // 부팅 자동 센터링 (분리 상태로 켜면 전부 중앙 위치로)
  startCentering();
}

void loop() {
  // 시리얼 수신
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) {
        buf[bufIdx] = '\0';
        parseLine(buf);
        bufIdx = 0;
      }
    } else if (bufIdx < sizeof(buf) - 1) {
      buf[bufIdx++] = c;
    }
  }

  // 이동 (센터링 중이면 더 부드럽게, 아니면 일반 스무딩)
  float k = centering ? CENTER_SMOOTH : SMOOTH;
  bool allClose = true;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    curAngle[i] += (targetAngle[i] - curAngle[i]) * k;
    writeServo(i, curAngle[i]);
    if (fabs(targetAngle[i] - curAngle[i]) > 0.5) allClose = false;
  }

  // 센터링 완료 판정
  if (centering && allClose) {
    centering = false;
    Serial.println("CENTERED");
  }

  delay(15);
}
