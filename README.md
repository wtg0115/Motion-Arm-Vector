# Arm Vector — 팔 따라 움직이는 로봇팔

휴대폰·태블릿 카메라로 사람 팔의 관절 위치를 읽어 벡터로 바꾸고, 그 각도를 아두이노 서보로 보내 로봇팔이 사람 팔을 따라 움직이게 하는 프로젝트예요.
카메라 인식과 계산은 브라우저 하나로 돌아가고, 따로 설치할 것이 없어요. 로봇 쪽은 `arduino/` 폴더의 펌웨어를 올려 써요.

- **최신 버전 바로 쓰기 (v10.4):** https://wtg0115.github.io/Motion-Arm-Vector/
- **모든 버전 모아 보기:** https://wtg0115.github.io/Motion-Arm-Vector/versions/
- **프로젝트 전체 설명:** [docs/project-overview.md](docs/project-overview.md) · **실측값·상수:** [docs/data-sheet.md](docs/data-sheet.md)

## 어떻게 돌아가나

1. **인식** — MediaPipe(자세·손 인식 모델)가 카메라 화면에서 어깨·팔꿈치·손목·손가락 위치를 찾아요.
2. **계산** — 관절 위치를 벡터로 바꾸고, 내적으로 관절 각도를 구해요. 손목 회전은 쿼터니언으로 다뤄요.
3. **다듬기** — One Euro 필터로 떨림을 줄이고, 기준자세를 등록해 사람마다 다른 팔 길이·가동범위를 맞춰요.
4. **제어** — 브라우저에서 아두이노로 바로 연결(Web Serial, 115200bps)해 서보 각도를 보내요. 아두이노는 PCA9685 서보 드라이버로 서보를 움직여요.

| 채널 | 관절 | 서보 |
|---|---|---|
| CH0 | 베이스 회전 | 25kg, 280° |
| CH1 | 숄더(어깨) | 35kg, 280° |
| CH2 | 엘보우(팔꿈치) | 20kg, 180° |
| CH3 | 손목 상하 | 20kg, 180° |
| CH4 | 손목 축회전 | 20kg, 180° |

## 화면(웹앱) 버전 기록

| 버전 | 날짜 | 바뀐 점 | 출처 |
|---|---|---|---|
| [**v10.4 시동 3모드**](versions/v10.4-start-modes.html) | 2026-07-16 | 발표 최종판. 시동 방식 3가지(엔터 즉시 / 손목만 확인 / 전체 자세 맞춤), COM 번호가 바뀌어도 아두이노 자동 재연결 | 대화 기록에서 복원 |
| [v10.3 최종](versions/v10.3-final.html) | 2026-07-16 | 시작 자세 일치 기준 완화(일치도 0.82→0.55, 유지 800→600ms) | GitHub |
| [v10.3 데모 조정](versions/v10.3-demo-tuned.html) | 2026-07-16 | 손목 축회전(CH4) ±80° 켬, 첫 실기 제한을 데모용으로 넓힘(±25/20/25/20/82°), 손목 상하(CH3) 방향 반전 수정, 중력 감속 0.35→0.6 | 내 컴퓨터 |
| [v10.2](versions/v10.2-simple-enable.html) | 2026-07-16 | PWM 한 번에 켜기 | 내 컴퓨터 |
| [v10.1 오른팔 기준값](versions/v10.1-right-arm-baseline.html) | 2026-07-16 | 사람 관절 각도를 서보 각도로 **직접** 바꾸는 미러링(방법 B) 시작 — 사람이 L자를 만들면 로봇도 L자. 오른팔 기준값 수집, 첫 실기 제한(±10/10/10/8°), PWM 두 번 확인 켜기, 긴급정지, 손목 8자세 진단 | 대화 기록에서 복원 |
| [v9.7](versions/v09.7-hand-gate.html) | 2026-07-15 | 손 선택 기준 완화(0.16→0.28), 1번 기준자세가 잠겨야 다음 단계 진행, 손 탈락 단계 3분리 표시, 빈 값(null)을 0으로 보던 버그 수정. v9.5 등록자세 상대 쿼터니언, v9.6 손목 7자세 진단 포함 | GitHub |
| [v8.7.0](versions/v08.7.0-joint-rom.html) | 2026-07-13 | 자세 등록, 진단 모드, 관절마다 최고 가동범위(ROM) 측정, CH0~CH4 사용 | GitHub |
| [v7.5](versions/v07.5-robot-control.html) | 2026-07-08 | v7.3에 인물 고정 추가 | GitHub |
| [v7.3](versions/v07.3-robot-control.html) | 2026-07-08 | 첫 로봇 제어판: Web Serial로 아두이노 연결, 서보·그리퍼 제어 | GitHub |
| [Multi v4.1](versions/v04.1-multi-workers-fix.html) | 2026-06-12 | 작업 스레드를 일반(classic) 방식으로 바꿔 `ModuleFactory not set` 오류 수정 | 대화 기록에서 복원 |
| [Multi v4](versions/v04-multi-workers.html) | 2026-06-12 | 자세 인식과 손 인식을 작업 스레드 두 개로 나눠 동시에 처리 | GitHub |
| [Multi v3](versions/v03-multi.html) | 2026-06-12 | One Euro 필터로 떨림 감소 | GitHub |
| [Multi v2.1](versions/v02.1-multi.html) | 2026-06-11 | v2를 다듬은 판, v3의 바탕 | 대화 기록 |
| [Multi v2](versions/v02-multi-fingers.html) | 2026-06-11 | 최대 4명 동시 인식, 손가락 인식, 어깨선 기준으로 손을 사람에게 짝지음 | GitHub(비공개) |
| [v1 Mobile](versions/v01-mobile.html) | 2026-06-11 | 첫 웹판. 한 사람 팔 벡터를 폰 브라우저에서 표시 | 대화 기록 |
| [Python 판](python/arm_vector_camera.py) | 2026-06-09 | 노트북 카메라용 첫 판(OpenCV + MediaPipe) | 대화 기록 |

v10.x 파일들은 화면 제목이 모두 "v10.1"로 남아 있어요. 버전 이름은 코드 안의 `[v10.3]` · `[v10.4]` 표시와 대화 기록으로 붙였어요.

## 아두이노 펌웨어 기록 (`arduino/`)

폴더 하나에 스케치 하나라서 아두이노 IDE에서 폴더째 열면 돼요. v2 이후는 **Adafruit PWM Servo Driver Library**가 필요해요.

| 펌웨어 | 날짜 | 하는 일 | 짝이 맞는 화면 | 출처 |
|---|---|---|---|---|
| [**v7.4 데모 조정**](arduino/fw_v7_4_demo_tuned/fw_v7_4_demo_tuned.ino) | 2026-07-16 | v7.3에서 첫 실기 제한을 데모용(±25/20/25/20/82°)으로 넓히고 중력 감속 0.6 | v10.3 · v10.4 | 대화 기록에서 복원 |
| [v7.3](arduino/fw_v7_3_simple_enable/fw_v7_3_simple_enable.ino) | 2026-07-16 | `E` 한 번으로 PWM 켜기, `X` 즉시 정지, `C` STOW 자세, `N` 조종 기준 자세 | v10.2 | 내 컴퓨터 |
| [v7.2 첫 실기](arduino/fw_v7_2_first_live/fw_v7_2_first_live.ino) | 2026-07-16 | `E`를 8초 안에 두 번 보내야 PWM 켜짐, 첫 실기 이중 제한 | v10.1 | 대화 기록에서 복원 |
| [v5.1](arduino/fw_v5_1_ch2_test/fw_v5_1_ch2_test.ino) | 2026-07-12 | 그리퍼 제거, 그리퍼 서보를 CH2에 꽂아 시험 | 수동 조종기 | 내 컴퓨터 |
| [v4.1](arduino/fw_v4_1_ch2_remap/fw_v4_1_ch2_remap.ino) | 2026-07-12 | CH2 엘보우 0°를 기준 위치로 재매핑 | 수동 조종기 v4 | 내 컴퓨터 |
| [v2](arduino/fw_v2_centering/fw_v2_centering.ino) | 2026-07-12 | 서보 6개를 가동범위 가운데로 정렬(조립 전 영점용) | 초기 조립 | 내 컴퓨터 |
| [v1 서보 2개](arduino/fw_v1_two_servo/fw_v1_two_servo.ino) | 2026-06-11 | `S95,E60` 형식으로 어깨·팔꿈치 서보 2개 | [python/arm_to_servo.py](python/arm_to_servo.py) | 대화 기록 |

## 알아 둘 점
- **"대화 기록에서 복원"** 은 Claude와의 대화에 남은 수정 기록을 남아 있던 원본 파일에 순서대로 다시 적용해 만든 파일이에요. 같은 방법으로 v10.3 최종판을 다시 만들어 보니 원본과 한 글자도 다르지 않았어요. 실물 로봇으로 다시 시험해 보지는 않았어요.
- v10.3 · v10.4 화면은 v10.1을 바탕으로 만들어져서 PWM 켜기가 두 번 확인 방식이고, 짝인 펌웨어 v7.4는 v7.3(한 번 켜기)을 바탕으로 만들어졌어요. 펌웨어 v7.4 파일 첫 줄 설명은 v7.3 그대로예요.
- v9.8 · v9.9 화면과 펌웨어 v6 · v7.0 · v7.1은 수정 기록만 있고 바탕 파일이 없어서 복원하지 못했어요.

## 문서 (`docs/`)

| 문서 | 내용 |
|---|---|
| [project-overview.md](docs/project-overview.md) | 원리 · 구조 · 만든 과정 · 문제와 해결 · 최종 시동 방법 |
| [data-sheet.md](docs/data-sheet.md) | 실측값 · 펌웨어 상수 · 매핑 상수 · 검증 수치 |
| [handover-2026-07-15-v9.8.md](docs/handover-2026-07-15-v9.8.md) | v9.8 / 펌웨어 v7.0 시점 인수인계 |
| [handover-2026-07-16-v10.1.md](docs/handover-2026-07-16-v10.1.md) | v10.1 첫 실기판을 만든 이유와 바뀐 점 |

## 저장소 정리 기록 (2026-11-11)
이 프로젝트의 버전이 저장소 8개, 컴퓨터, Claude 대화에 흩어져 있던 것을 여기 하나로 모았어요. 원래 저장소(aVXYVSh, `-`, ARM--VECTER, arm-vector-, arm-vectorV3, icdoJ, sodie)는 내용을 옮긴 뒤 지웠어요.
