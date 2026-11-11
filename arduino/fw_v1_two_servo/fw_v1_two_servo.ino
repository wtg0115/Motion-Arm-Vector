/*
 * arm_servo_receiver.ino — 시리얼 각도 수신 → 서보 2개 구동
 * ----------------------------------------------------------
 * 프로토콜: "S<어깨각>,E<팔꿈치각>\n"   예) S95,E60   (0~180 정수)
 *
 * 배선 (실물 조립 시):
 *   어깨 서보 신호 → D9, 팔꿈치 서보 신호 → D10
 *   서보 V+ → 외부 전원(AA×4, 6V), 서보 GND → 배터리(-)와 아두이노 GND 공통
 *   ※ 서보 전원을 아두이노 5V 핀에서 뽑지 말 것 (브라운아웃 리셋)
 *   전원단에 1000uF 캐패시터 병렬
 *
 * Wokwi 테스트: diagram.json을 같이 붙여넣고
 *   시리얼 모니터에 S90,E45 입력 → 서보가 도는지 확인
 */
#include <Servo.h>

#define DEBUG_ECHO 1          // 1: 수신값 ACK 출력 (운용 시 0으로)

const int PIN_SHOULDER = 9;
const int PIN_ELBOW    = 10;
const unsigned long STEP_MS = 15;  // 슬루 갱신 주기
const int MAX_STEP_DEG = 4;        // 주기당 최대 이동각 → 약 266도/초 상한

Servo servoS, servoE;
int targetS = 90, targetE = 90;    // 목표각 (수신값)
int curS = 90, curE = 90;          // 현재각 (슬루 적용)
unsigned long lastStep = 0;

char buf[32];
byte idx = 0;

void handleLine(char *line) {
  int s, e;
  if (sscanf(line, "S%d,E%d", &s, &e) == 2) {
    targetS = constrain(s, 0, 180);
    targetE = constrain(e, 0, 180);
#if DEBUG_ECHO
    Serial.print("ACK S"); Serial.print(targetS);
    Serial.print(" E");    Serial.println(targetE);
#endif
  }
  // 형식이 안 맞으면 조용히 무시 (노이즈 내성)
}

void setup() {
  Serial.begin(115200);
  servoS.attach(PIN_SHOULDER);
  servoE.attach(PIN_ELBOW);
  servoS.write(curS);
  servoE.write(curE);
  Serial.println("READY");
}

void loop() {
  // 1) 시리얼 라인 수신
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) { buf[idx] = '\0'; handleLine(buf); idx = 0; }
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;  // 너무 긴 라인 → 버림
    }
  }

  // 2) 슬루 리미터: 목표각으로 천천히 수렴 (패킷 점프로부터 서보 보호)
  unsigned long now = millis();
  if (now - lastStep >= STEP_MS) {
    lastStep = now;
    curS += constrain(targetS - curS, -MAX_STEP_DEG, MAX_STEP_DEG);
    curE += constrain(targetE - curE, -MAX_STEP_DEG, MAX_STEP_DEG);
    servoS.write(curS);
    servoE.write(curE);
  }
  // 통신이 끊기면 마지막 목표각 유지 (페일세이프 = 홀드)
}