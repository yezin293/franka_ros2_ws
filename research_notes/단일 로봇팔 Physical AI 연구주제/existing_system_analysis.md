# 기존 시스템 분석 (코디네이터 작성, 사용자와의 대화 기록 기반)

이 노트는 웹 조사가 아니라, 이 워크스페이스에서 실제로 구현/검증된 내용을 코디네이터가 직접 정리한 것이다. 보고서 작성 시 "현재 구현된 것"의 사실관계는 이 파일을 우선한다.

## 하드웨어 (현재 → 다음 단계 변경 예정)

- 로봇: Franka Research 3(FR3) 1대, Franka Hand 그리퍼. (Dual-arm 토르소 구조도 만들어봤으나 실험 가능한 팔은 1대뿐 — bimanual 연구는 제외)
- 카메라: 기존에는 고정 포스트에 OBSBOT Tiny 2 Lite 단안 RGB 카메라(eye-to-hand). **다음 단계에서는 eye-in-hand RGB-D 깊이 카메라 1대로 전환 예정** — OBSBOT은 계속 있지만 필수 아님.
- 작업공간: 트레이 2개(pick/place).

## 소프트웨어 — 이미 구현되어 재사용 가능한 것

- ROS2 Humble + MoveIt2(OMPL RRTConnect) + ros2_control, 실물 FR3에서 검증 완료.
- `fr3_skills` 패키지(`Fr3Motion` 클래스): 관절/Cartesian 플래닝, TCP 변환(link8↔fr3_hand_tcp 오프셋 처리), 그리퍼 제어(open/close/grasp, width+epsilon+force), 테이블 충돌 모델, activate/deactivate 컨트롤러 자동화.
- `pick()`/`place()`: approach(OMPL)→descend(Cartesian path)→grasp/release→retreat(Cartesian path) 패턴. 실물 검증됨.
- 카메라-로봇 외부 캘리브레이션: 체커보드 네 귀퉁이를 로봇 그리퍼로 직접 터치해 robot-frame 좌표 획득 + bilinear 보간 + `cv2.solvePnP`. 내부 캘리브레이션(intrinsics)도 체커보드로 완료. (단, 이건 **eye-to-hand** 전용 방법론 — eye-in-hand로 바뀌면 카메라가 로봇과 같이 움직이므로 이 방식 자체는 재사용 불가하고, hand-eye calibration(예: `cv2.calibrateHandEye` 또는 유사 기법)으로 바뀌어야 함. 다만 "로봇 그리퍼로 기준점을 직접 터치해서 3D 정답 좌표를 얻는다"는 핵심 아이디어/노하우는 eye-in-hand 캘리브레이션에도 그대로 활용 가능.)
- 픽셀→3D 변환: ray-plane intersection(현재는 알려진 높이의 평면 가정). RGB-D로 바뀌면 깊이값을 직접 쓸 수 있어 이 가정이 불필요해지거나, 평면 가정 자체를 검증/완화하는 연구 소재가 될 수 있음.
- 물체 검출: 명도 임계값 + ROI 제한 (OpenCV, 학습 기반 아님).
- 실시간 파이프라인: `object_pose_publisher`(연속 검출+발행, 디버그 영상 스트림) + `vision_pick_place`(구독→pick→place→home, 설정은 yaml로 분리) — ROS2 노드 구조, 설정 분리 패턴은 그대로 재사용 가능.
- 안전장치: 목표 pose의 z 최소값 체크, 테이블 collision object, plan-only 기본 모드.

## 소프트웨어 — 아직 없는 것 (신규 개발 필요)

- 학습 기반 어떤 것도 없음(검출, 정책, 실패감지 전부 classical/rule-based).
- 깊이 센서 데이터 처리 파이프라인 전무(RGB만 다뤄봄).
- 로봇이 스스로 카메라 시점을 바꾸는 기능 없음(카메라가 고정이었으므로).
- 행동 후 재관측→오차 감지→보정하는 피드백 루프 없음(지금은 한 번 검출→한 번 이동→끝, 열린 루프).
- 실패 감지/복구 로직 없음(그립 실패 시 그냥 에러 던지고 멈춤).
- 시연 데이터 수집/정책 학습 관련 코드 전무.
- 자연어 인터페이스 전무.

## 알려진 제약/교훈 (연구 설계 시 고려할 것)

- 그리퍼 grasp 액션은 물리적 성공과 액션 성공 판정(폭 허용오차)이 다를 수 있음 — 학습 기반 실패감지를 넣을 때 "진짜 실패"와 "판정 실패"를 구분해야 함.
- MoveIt2 OMPL(샘플링 기반)은 같은 목표라도 매번 팔꿈치 자세가 달라짐(7DOF 중 6DOF만 구속) — 재현성이 필요한 연구(예: 동일 조건 반복 실험)에서는 Cartesian path나 joint-space 목표로 고정하는 게 나을 수 있음.
- 손목 목표점이 유효해도 팔꿈치/전완 등 다른 링크가 트레이 등과 충돌 가능 — 안전마진은 항상 별도 점검 필요.
- 연구실 환경이 번잡해서(의자, 옷 등 배경 클러터) 순수 영상 임계값 기반 검출은 ROI 제한 없이는 쉽게 오검출됨.
