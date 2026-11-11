"""
arm_to_servo.py — 카메라 팔 추적 → 로봇팔 서보 각도 송신
=========================================================
파이프라인:
  MediaPipe Pose → 어깨/팔꿈치 각도 → EMA 필터 + 데드밴드
  → 서보 범위 매핑 → 시리얼 "S<어깨>,E<팔꿈치>\\n" @ 30Hz

아두이노가 안 꽂혀 있으면 자동 DRY-RUN 모드:
  보낼 라인을 콘솔에 출력만 한다 → Wokwi 시리얼 모니터에 복붙해서 테스트 가능.

설치: pip install mediapipe opencv-python numpy pyserial
실행: python arm_to_servo.py        종료: q
"""

import time
import cv2
import numpy as np
import mediapipe as mp

try:
    import serial as pyserial
except ImportError:
    pyserial = None

# ── 설정 ───────────────────────────────────────────
PORT = "COM3"        # 장치 관리자 → 포트(COM & LPT)에서 확인
BAUD = 115200
SEND_HZ = 30         # 송신 주기
ARM = "right"        # "right" / "left"

# 각도 매핑: (입력 범위) -> (서보 범위)
# 실물 조립 후 움직임 방향이 반대면 해당 OUT 튜플 순서만 뒤집으면 됨.
SHOULDER_IN = (-90.0, 90.0)   # 화면 기준 상완 각: -90=팔 위, 0=수평, +90=팔 아래
SHOULDER_OUT = (180.0, 0.0)
ELBOW_IN = (40.0, 180.0)      # 팔꿈치 굽힘각: 40=완전 굽힘, 180=쭉 폄
ELBOW_OUT = (0.0, 140.0)

EMA_ALPHA = 0.35     # 클수록 반응 빠르고 떨림 큼 (0.2~0.5 권장)
DEADBAND = 1.5       # 이 각도(deg) 미만 변화는 무시 → 채터링 방지

# ── MediaPipe 준비 ─────────────────────────────────
mp_pose = mp.solutions.pose
LM = mp_pose.PoseLandmark
SHOULDER, ELBOW, WRIST = (
    (LM.RIGHT_SHOULDER, LM.RIGHT_ELBOW, LM.RIGHT_WRIST)
    if ARM == "right"
    else (LM.LEFT_SHOULDER, LM.LEFT_ELBOW, LM.LEFT_WRIST)
)

C_UPPER = (0, 165, 255)
C_FORE = (80, 255, 120)
C_JOINT = (255, 255, 255)


class SmoothAngle:
    """EMA + 데드밴드 필터"""

    def __init__(self, alpha, deadband):
        self.alpha = alpha
        self.deadband = deadband
        self.ema = None
        self.out = None

    def update(self, x):
        self.ema = x if self.ema is None else self.alpha * x + (1 - self.alpha) * self.ema
        if self.out is None or abs(self.ema - self.out) >= self.deadband:
            self.out = self.ema
        return self.out


def map_clamp(x, in_rng, out_rng):
    """선형 매핑 + 범위 클램프"""
    a, b = in_rng
    c, d = out_rng
    t = (x - a) / (b - a)
    t = min(1.0, max(0.0, t))
    return c + t * (d - c)


def open_serial():
    if pyserial is None:
        print("[DRY-RUN] pyserial 미설치 — 콘솔 출력만 한다")
        return None
    try:
        s = pyserial.Serial(PORT, BAUD, timeout=0)
        time.sleep(2.0)  # 우노는 포트 열리면 리셋되므로 부팅 대기
        print(f"[SERIAL] {PORT} 연결됨")
        return s
    except Exception as ex:
        print(f"[DRY-RUN] {PORT} 열기 실패({ex}) — 콘솔 출력만 한다")
        return None


def to_px(lm, w, h):
    return np.array([lm.x * w, lm.y * h])


def angle_deg(v1, v2):
    cos = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-9)
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def put_text(img, text, org, color=(255, 255, 255), scale=0.6):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def main():
    ser = open_serial()
    f_shoulder = SmoothAngle(EMA_ALPHA, DEADBAND)
    f_elbow = SmoothAngle(EMA_ALPHA, DEADBAND)

    pose = mp_pose.Pose(
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없어.")
        return

    send_interval = 1.0 / SEND_HZ
    last_send = 0.0
    last_dry_print = 0.0
    last_line = "-"

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        result = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if result.pose_landmarks:
            lm = result.pose_landmarks.landmark
            sh = to_px(lm[SHOULDER], w, h)
            el = to_px(lm[ELBOW], w, h)
            wr = to_px(lm[WRIST], w, h)

            upper = el - sh
            fore = wr - el
            raw_elbow = angle_deg(-upper, fore)
            raw_shoulder = np.degrees(np.arctan2(upper[1], upper[0]))

            # 필터 → 매핑
            sm_shoulder = f_shoulder.update(raw_shoulder)
            sm_elbow = f_elbow.update(raw_elbow)
            sv_s = int(round(map_clamp(sm_shoulder, SHOULDER_IN, SHOULDER_OUT)))
            sv_e = int(round(map_clamp(sm_elbow, ELBOW_IN, ELBOW_OUT)))

            # 송신 (30Hz 상한)
            now = time.monotonic()
            if now - last_send >= send_interval:
                last_send = now
                last_line = f"S{sv_s},E{sv_e}"
                if ser:
                    ser.write((last_line + "\n").encode())
                elif now - last_dry_print >= 0.2:  # DRY-RUN은 5Hz로만 콘솔 출력
                    last_dry_print = now
                    print(last_line)

            # 시각화
            p_sh, p_el, p_wr = sh.astype(int), el.astype(int), wr.astype(int)
            cv2.arrowedLine(frame, tuple(p_sh), tuple(p_el), C_UPPER, 4, tipLength=0.08)
            cv2.arrowedLine(frame, tuple(p_el), tuple(p_wr), C_FORE, 4, tipLength=0.08)
            for p in (p_sh, p_el, p_wr):
                cv2.circle(frame, tuple(p), 7, C_JOINT, -1)
                cv2.circle(frame, tuple(p), 7, (0, 0, 0), 2)

            y = 30
            put_text(frame, f"raw   sh {raw_shoulder:+6.1f}  el {raw_elbow:6.1f}", (10, y)); y += 26
            put_text(frame, f"filt  sh {sm_shoulder:+6.1f}  el {sm_elbow:6.1f}", (10, y)); y += 26
            put_text(frame, f"servo S{sv_s:3d}  E{sv_e:3d}", (10, y), (0, 255, 255)); y += 26
            put_text(frame, f"tx    {last_line}", (10, y), (200, 200, 200)); y += 26
        else:
            put_text(frame, "팔 안 보임 — 송신 일시정지(서보는 마지막 각도 유지)", (10, 30), (0, 200, 255))

        mode = "SERIAL " + PORT if ser else "DRY-RUN"
        put_text(frame, f"[{mode}]  q: 종료", (10, h - 15), (180, 180, 180), 0.5)
        cv2.imshow("Arm -> Servo", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    pose.close()
    if ser:
        ser.close()


if __name__ == "__main__":
    main()