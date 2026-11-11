"""
실시간 팔 벡터 시각화 — 카메라(MediaPipe Pose) 버전
======================================================
웹캠으로 어깨·팔꿈치·손목을 추적해서
  - 상완 벡터(어깨 -> 팔꿈치)
  - 전완 벡터(팔꿈치 -> 손목)
로 치환하고, 팔 움직임과 동시에 화면에 그린다.

설치:  pip install mediapipe opencv-python numpy
실행:  python arm_vector_camera.py
종료:  q 키

참고: 네 기하 수행평가(2-link 로봇팔)는 '각도 -> 끝점 위치'(정기구학)였지.
      이건 그 반대야 -- 실측 위치에서 각도를 역산해 실시간으로 보는 거.
"""

import cv2
import numpy as np
import mediapipe as mp

# ── 설정 ───────────────────────────────────────────
ARM = "right"           # "right" 또는 "left" (화면은 거울 모드)
SHOW_3D_DEPTH = False   # True면 z(깊이) 성분도 텍스트로 표시

mp_pose = mp.solutions.pose
LM = mp_pose.PoseLandmark
SHOULDER, ELBOW, WRIST = (
    (LM.RIGHT_SHOULDER, LM.RIGHT_ELBOW, LM.RIGHT_WRIST)
    if ARM == "right"
    else (LM.LEFT_SHOULDER, LM.LEFT_ELBOW, LM.LEFT_WRIST)
)

# 색상 (BGR)
C_UPPER = (0, 165, 255)    # 상완 — 주황
C_FORE = (80, 255, 120)    # 전완 — 초록
C_JOINT = (255, 255, 255)  # 관절 — 흰색


def to_px(lm, w, h):
    """MediaPipe normalized 좌표(0~1) -> 화면 픽셀 좌표"""
    return np.array([lm.x * w, lm.y * h])


def angle_deg(v1, v2):
    """두 벡터 사이 각도(도)"""
    cos = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-9)
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def put_text(img, text, org, color=(255, 255, 255), scale=0.6):
    """검은 외곽선 + 컬러 글씨 (가독성)"""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def main():
    pose = mp_pose.Pose(
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없어. 다른 앱이 쓰고 있는지, 카메라 번호(0)가 맞는지 확인해봐.")
        return

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)  # 거울 모드
        h, w = frame.shape[:2]

        result = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if result.pose_landmarks:
            lm = result.pose_landmarks.landmark
            sh = to_px(lm[SHOULDER], w, h)
            el = to_px(lm[ELBOW], w, h)
            wr = to_px(lm[WRIST], w, h)

            # ── 벡터 치환 ──
            upper = el - sh   # 상완 벡터
            fore = wr - el    # 전완 벡터
            elbow_angle = angle_deg(-upper, fore)                        # 팔꿈치 굽힘 각
            shoulder_angle = np.degrees(np.arctan2(upper[1], upper[0]))  # 수평선 기준 상완 각

            # ── 그리기 ──
            p_sh, p_el, p_wr = sh.astype(int), el.astype(int), wr.astype(int)
            cv2.arrowedLine(frame, tuple(p_sh), tuple(p_el), C_UPPER, 4, tipLength=0.08)
            cv2.arrowedLine(frame, tuple(p_el), tuple(p_wr), C_FORE, 4, tipLength=0.08)
            for p in (p_sh, p_el, p_wr):
                cv2.circle(frame, tuple(p), 7, C_JOINT, -1)
                cv2.circle(frame, tuple(p), 7, (0, 0, 0), 2)

            # ── 정보 패널 ──
            y = 30
            put_text(frame, f"upper arm  ({upper[0]:+5.0f},{upper[1]:+5.0f})  |{np.linalg.norm(upper):4.0f}px", (10, y), C_UPPER); y += 28
            put_text(frame, f"forearm    ({fore[0]:+5.0f},{fore[1]:+5.0f})  |{np.linalg.norm(fore):4.0f}px", (10, y), C_FORE); y += 28
            put_text(frame, f"elbow angle    {elbow_angle:5.1f} deg", (10, y)); y += 28
            put_text(frame, f"shoulder angle {shoulder_angle:+5.1f} deg", (10, y)); y += 28

            if SHOW_3D_DEPTH:
                put_text(frame, f"z  sh{lm[SHOULDER].z:+.2f} el{lm[ELBOW].z:+.2f} wr{lm[WRIST].z:+.2f}", (10, y), (200, 200, 200), 0.5)
        else:
            put_text(frame, "팔이 안 보여 — 어깨~손목이 프레임 안에 들어오게", (10, 30), (0, 200, 255))

        put_text(frame, "q: 종료", (10, h - 15), (180, 180, 180), 0.5)
        cv2.imshow("Arm Vector - Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    pose.close()


if __name__ == "__main__":
    main()