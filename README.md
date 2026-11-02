# franka_ros2_ws: FR3 실물 제어 + Bin Picking

Franka Research 3 실물을 ROS2 Humble + MoveIt2로 제어하고, 이후 카메라 기반 Bin Picking으로 확장하는 워크스페이스.

## 이 저장소에 있는 것

| 경로 | 내용 |
|---|---|
| src/fr3_skills/ | 동작 API (move_home, 이후 move_to_pose, gripper, pick/place) |
| scripts/ | 시험용 스크립트 |
| fr3.repos | 원본 패키지(libfranka, franka_ros2, ...) 버전 목록 |
| migration/ | franka_ros2 v2.0.2 수정 패치 + 새 컴퓨터 설치 가이드 |
| CLAUDE_HANDOFF_KO.md | 진행 기록 (작업 재개 시 가장 먼저 읽기) |

원본 패키지는 저장소에 포함하지 않는다.

## 새 컴퓨터에서 받기

```bash
git clone https://github.com/yezin293/franka_ros2_ws.git ~/franka_ros2_ws
```

```bash
cd ~/franka_ros2_ws
```

```bash
vcs import --recursive src < fr3.repos
```

```bash
git -C src/franka_ros2 apply ../../migration/franka_ros2_local_changes.patch
```

이후 의존성 설치, 빌드, rtprio, 네트워크 설정은 migration/MIGRATION_KO.md 4~7단계, 두 컴퓨터를 오가는 방법(git pull → 작업 → git push)은 9단계를 따른다.

## 카메라(OBSBOT) 설정

Bin picking용 카메라는 USB 절전모드(autosuspend) 때문에 가만히 두면 몇 초 후 pan/tilt 각도 제어 명령이 안 먹는다(영상은 계속 나와서 헷갈리기 쉽다). 새 컴퓨터에서는 아래를 한 번만 해두면 된다.

```bash
sudo cp scripts/99-obsbot-no-autosuspend.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

적용 후 카메라 USB 케이블을 한 번 뽑았다 꽂으면 확인된다(`cat /sys/bus/usb/devices/3-2.1/power/control`이 `on`으로 나오면 정상 — USB 포트가 다르면 경로도 바뀔 수 있다).

카메라 각도(두 트레이가 다 보이는 작업 각도)는 다음으로 맞춘다. 재부팅/재연결 후에는 기본값으로 돌아가므로 매번 다시 설정해야 한다.

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=tilt_absolute=-115200,pan_absolute=18000
```

**그래도 각도가 안 맞거나 계속 움직이면**: OBSBOT 자체 AI 사람 추적 기능이 켜져 있을 수 있다 — 카메라 렌즈에 손바닥을 1~2초 펴서 보여주면 꺼진다(파란불 두 번 깜빡 → 초록불 고정). 그래도 안 되면 짐벌이 완전히 멈춘 상태일 수 있으니 USB 케이블을 재연결한다.
