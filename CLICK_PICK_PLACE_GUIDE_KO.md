# FR3 + D435i 클릭 기반 주사위 Pick & Place 실행 가이드

이 문서는 D435i 화면에서 검은색 40 mm 주사위를 클릭하여 다음 시퀀스를 실행하는 방법을 정리한다.

1. 주사위 윗면 중앙 클릭
2. Depth 기반 정사각형 크기 및 회전각 검증
3. 그리퍼 homing 및 실제 개방 폭 확인
4. 주사위 방향에 맞춰 그리퍼 회전
5. 접근, 파지, 수직 상승 후 HOME/READY 운반 자세로 이동
6. 내려놓을 위치 클릭
7. 같은 파지 높이로 하강, 놓기, 상승

## 실행 전 확인

- Franka Desk에서 로봇 상태가 정상(초록색)이어야 한다.
- FCI가 활성화되어 있어야 한다.
- D435i RGB와 aligned depth가 실행 중이어야 한다.
- 주사위와 놓을 위치는 트레이 벽 및 가운데 볼트에서 충분히 떨어져 있어야 한다.
- 접촉 또는 예상하지 못한 움직임이 보이면 즉시 Desk Stop을 누른다.

## 터미널 1: FR3 및 그리퍼 Bringup

```bash
cd /home/nuc/ros2_ws/src/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_bringup franka.launch.py \
  robot_ip:=172.16.0.2 \
  arm_id:=fr3 \
  use_fake_hardware:=false \
  load_gripper:=true
```

이 터미널은 계속 실행해 둔다.

## 터미널 2: MoveIt 실행

```bash
cd /home/nuc/ros2_ws/src/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config move_group.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true \
  execute:=true
```

이 터미널도 계속 실행해 둔다.

## 터미널 3: 팔 컨트롤러 로드

bringup을 다시 시작할 때마다 한 번 실행한다.

```bash
cd /home/nuc/ros2_ws/src/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run controller_manager spawner fr3_arm_controller \
  --inactive \
  -p src/franka_ros2/franka_fr3_moveit_config/config/fr3_arm_controller_real.yaml
```

컨트롤러는 평소 `inactive` 상태이며, 각 이동 직전에만 활성화되고 완료 후 다시 비활성화된다.

## 터미널 4: 전체 Pick & Place 시퀀스

```bash
cd /home/nuc/ros2_ws/src/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/click_pick_place_sequence.py
```

### 첫 번째 클릭: 집을 주사위

- 주사위의 큰 검은 윗면 중앙을 클릭한다.
- Depth에서 약 40×40 mm 정사각형이 검출되어야 한다.
- 검출 평면의 법선이 로봇 베이스 Z축과 20° 이내여야 하며, 수직인 앞면·옆면은 거부한다.
- 정사각형 검증에 실패하면 로봇은 움직이지 않는다.
- 검증된 변의 방향에 맞춰 그리퍼 yaw가 자동 정렬된다.
- 그리퍼 homing 후 실제 개방 폭이 75 mm 이상이어야 하강을 시작한다.
- 파지 후 먼저 파지 높이에서 20 cm 수직 상승한다.
- 이어서 `config/poses.yaml`에 저장된 `home` 자세로 이동한다.

### 두 번째 클릭: 내려놓을 위치

- 물체가 없는 평평한 트레이 바닥을 클릭한다.
- 트레이 벽과 가운데 볼트에서 최소 6 cm 이상 떨어진 위치를 선택한다.
- 로봇은 배치 TCP `z=0.0402 m`까지 하강한다.
- 그리퍼를 열어 주사위를 트레이 바닥에 놓는다.
- 물체를 놓은 뒤 20 cm 수직 상승한다.
- 이어서 `config/poses.yaml`의 `home` 자세로 이동하며 그리퍼는 열린 상태를 유지한다.

## 이미 물체를 들고 있을 때: 놓기만 실행

```bash
cd /home/nuc/ros2_ws/src/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/click_pick_place_sequence.py --place-held
```

## 클릭 없는 자동 반대편 트레이 이송

자동 검출과 무작위 배치를 먼저 계산만 하고 로봇을 움직이지 않으려면:

```bash
python3 scripts/auto_cube_transfer.py
```

검출 이미지와 결과는 다음 위치에 저장된다.

```text
calibration_data/auto_cube_detection.jpg
calibration_data/auto_cube_detection.json
```

결과를 확인한 뒤 실제 자동 이송을 실행하려면:

```bash
python3 scripts/auto_cube_transfer.py --execute
```

재현 가능한 동일한 무작위 배치 목표를 사용하려면 seed를 지정한다.

```bash
python3 scripts/auto_cube_transfer.py --execute --seed 42
```

지정한 횟수만큼 좌우 트레이 이송을 반복하려면:

```bash
python3 scripts/auto_cube_transfer.py --execute --episodes 10
```

`Ctrl+C`로 중단할 때까지 계속 반복하려면:

```bash
python3 scripts/auto_cube_transfer.py --execute --continuous
```

각 에피소드가 끝나면 HOME/READY 자세에서 주사위를 다시 검출한다. 현재 주사위가 오른쪽 트레이에 있으면 왼쪽으로, 왼쪽 트레이에 있으면 오른쪽으로 자동 배치하므로 방향이 매 에피소드마다 번갈아 바뀐다. Homing은 첫 에피소드에서만 수행하고 이후에는 매번 실제 그리퍼 개방 폭을 확인한다.

자동 시퀀스는 다음 순서로 동작한다.

1. RGB-D 전체 화면에서 어두운 후보를 검색한다.
2. 수평면 기울기, 약 40×40 mm 크기와 정사각형 비율을 검증한다.
3. 검출 중심이 실제 좌·우 트레이 영역 내부이고 주사위 윗면 높이인지 검증한다.
4. 비슷한 후보가 둘 이상이면 자동 선택하지 않고 중단한다.
5. 주사위를 파지하고 20 cm 상승한 뒤 HOME/READY 자세로 이동한다.
6. 현재 주사위가 있는 트레이의 반대편 트레이를 선택한다.
7. 트레이 중심 볼트에서 반지름 5.5 cm인 원 위의 안전 후보를 샘플링한다.
8. 배치 yaw를 0~90°에서 무작위로 선택한다.
9. 배치 후 다음 에피소드의 검출 트레이가 직전 목표 트레이와 일치하는지 확인한다.
10. 바닥보다 설정된 높이에서 놓은 뒤 HOME/READY 자세로 복귀한다.

`--execute`가 없으면 절대로 로봇을 움직이지 않는다. 트레이나 볼트 위치가 변경되면 코드의 트레이 중심 및 안전 반지름을 다시 측정해야 한다.

### 자동 인식 화면 실시간 확인

자동 이송과 별도의 터미널에서 다음을 실행한다. 이 스크립트는 로봇을 움직이지 않는다.

```bash
cd /home/nuc/ros2_ws/src/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/auto_cube_visualizer.py
```

표시 색상:

- 노란 윤곽: 검사 중인 어두운 후보
- 자홍색 사각형: Depth·크기·수평면·트레이 위치 검사를 통과한 후보
- 굵은 초록 사각형: 자동 이송 코드가 최종 선택할 후보
- 빨간 상태 글자: 유효한 주사위가 없음

HOME 시야의 자동 노출에서도 검은 주사위를 찾도록 자동 검출의 밝기 임계값은 `115`를 사용한다. 밝기를 완화하더라도 Depth 크기·수평면·트레이 위치 검사를 모두 통과해야 최종 후보가 된다.

창을 종료하려면 `Q` 또는 `Esc`를 누른다. 같은 영상은 `/auto_cube/debug_image` 토픽으로도 발행된다.

```bash
ros2 run rqt_image_view rqt_image_view /auto_cube/debug_image
```

## D435i aligned depth 확인

```bash
ros2 topic list | grep aligned_depth_to_color
```

토픽이 없다면 실행 중인 RealSense 노드에서 다음 파라미터를 설정한다.

```bash
ros2 param set /camera/camera enable_depth true
ros2 param set /camera/camera enable_sync true
ros2 param set /camera/camera align_depth.enable true
```

확인할 주요 토픽:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
```

## 그리퍼만 점검

### Homing

```bash
ros2 run fr3_skills gripper --home
```

### 완전히 열기

```bash
ros2 run fr3_skills gripper --open --speed 0.05
```

### 실제 손가락 폭 확인

```bash
ros2 topic echo /franka_gripper/joint_states --once
```

완전히 열렸을 때 두 finger joint 위치의 합이 약 0.08 m여야 한다. 예:

```text
fr3_finger_joint1: 약 0.040 m
fr3_finger_joint2: 약 0.040 m
총 개방 폭: 약 0.080 m
```

## 접촉, User Stop 또는 빨간불 발생 시

접촉 후 기존 ROS 프로세스를 그대로 사용하지 않는다.

1. Desk Stop으로 움직임을 멈춘다.
2. Desk에서 오류를 복구한다.
3. 로봇을 안전한 높은 위치로 이동한다.
4. Desk 상태가 정상인지 확인한다.
5. 기존 bringup과 MoveIt 터미널을 종료한다.
6. 위의 터미널 1, 2, 3 순서로 모두 새로 시작한다.
7. 높은 위치에서 gripper homing과 완전 개방을 확인한다.
8. 전체 시퀀스를 새로 시작한다.

User Stop이 실행 중인 FCI 동작을 중단하면 `ros2_control_node`가 종료될 수 있다. Desk가 다시 초록색이어도 ROS bringup은 별도로 재시작해야 한다.

## 현재 안전상 제한

- MoveIt에는 실제 테이블 충돌 모델이 있지만 트레이 벽과 가운데 볼트는 아직 포함되어 있지 않다.
- 따라서 클릭 위치 주변의 그리퍼 전체 폭까지 고려해 벽과 볼트를 피해야 한다.
- 카메라 hand-eye의 절대 Z에는 오차가 있어, Z는 측정된 트레이 바닥 `0.0222 m`와 40 mm 주사위 크기를 기준으로 보정한다.
- 파지 TCP 높이는 트레이 바닥보다 약 28 mm 높은 `z=0.0502 m`를 사용한다.
- 배치 TCP 높이는 1 cm 낮춘 `z=0.0402 m`를 사용한다.
- 팔의 직선 이동 속도 계수는 `0.16`으로, 기존 `0.08`보다 약 2배 빠르다.
- HOME/READY 관절 이동은 설정된 안전 상한 속도 계수 `0.20`을 사용한다.
- 그리퍼 열기·닫기 속도는 안정적인 파지를 위해 기존 값을 유지한다.
- 주사위가 아닌 물체에는 현재 크기 및 높이 설정을 그대로 사용하면 안 된다.

## 관련 스크립트

- `scripts/click_pick_place_sequence.py`: 전체 2클릭 Pick & Place 시퀀스
- `scripts/select_black_cube_rgbd.py`: RGB-D 클릭, 정사각형 및 회전각 검출
- `scripts/approach_selected_cube.py`: 선택 좌표 접근
- `scripts/retreat_current_tcp.py`: 현재 TCP에서 수직 상승
- `scripts/handeye_d435i.json`: D435i hand-eye 캘리브레이션

