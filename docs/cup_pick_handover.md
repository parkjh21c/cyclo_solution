# 컵 잡기, 컵 핸드오버(양방향), 붓기

2026-11-10~11 작업 기록. 로봇에서 `1` → `6` → `7`, 그리고 셀 집기부터 `2` → `3` → `4` → `5` → `6` → `7` → `8`
→ `9` 순서로 동작을 확인했습니다. 이후 내려놓기가 이어질 예정입니다(아래 "다음 단계").

## 전체 흐름

```
6  /pick_cup        왼손이 컵을 옆에서 잡아 8 cm 들어 올림 (MoveIt 컵 모델을 왼손에 부착)
7  /handover_cup    왼손 → 오른손 컵 넘김, 끝나면 오른손이 컵을 든 상태로 정지
8  /pour_cup        오른손이 바구니 위에서 오른쪽 → 왼쪽으로 130° 기울였다가 넘김 자세로 복귀
9  /handover_cup_back  오른손 → 왼손 컵 넘김, 왼손이 컵을 든 채 정지, 오른손은 초기 자세
```

셀 핸드오버(`3`, `d`의 HANDOVER)는 마지막에 `giver_home`으로 오른팔을 초기 자세로 되돌립니다. 이후 바구니에 남은
셀을 확인하려면 초기 자세가 필요하고, 컵 넘김 목표도 오른팔이 초기 자세에 있다고 가정하고 풀었기 때문입니다.
Pilz PTP의 관절 직선 경로는 `basket_table`을 지나므로 메뉴 `1`과 같은 OMPL을 씁니다.

`/handover_cup`은 `handover.yaml`의 시퀀스 세 개를 `CupHandoverMotion`이 이어서 실행합니다.

| 시퀀스 | 단계 | 플래너 |
|---|---|---|
| `cup_left_to_right` | `cup_carry`(왼손, 컵을 든 채 넘김 자세) → `cup_ready_via`(오른손, 초기 자세에서 j1만 먼저) → `cup_ready`(오른손, 잡는 위치 20 cm 뒤) | Pilz PTP(관절), 한 팔씩 |
| (`CUP_CONTACT_SCENE`) | 오른손 손가락·손바닥과 컵의 접촉 허용 | 장면 변경 |
| `cup_left_to_right_grasp` | `cup_insert`(오른손 직선 접근) → `receiver_close` | Pilz LIN, 그리퍼 |
| (`CUP_DETACH`, `CUP_ATTACH`) | MoveIt 컵을 왼손에서 떼어 오른손에 붙임 | 장면 변경 |
| `cup_left_to_right_release` | `giver_release`(왼손 열기) → `cup_retreat`(왼손 12 cm 후퇴) | 그리퍼, Pilz LIN |

접촉 허용을 삽입 직전에만 적용하는 이유: 허용을 시퀀스 시작 전에 걸었을 때 오른팔이 이동 중에 컵에
부딪혀도 계획이 통과했습니다.

`cup_ready_via`가 필요한 이유: 바구니가 장면에 있으면 초기 자세 → `cup_ready`의 관절 직선 경로에서 오른손
손가락이 `basket_wall_y+`에 닿습니다(t 0.23). `ptp_path_check.py --via-search`로 j1을 먼저 옮기는 경유
자세를 찾았습니다(j5, j6, j7만 먼저 옮겨도 통과했지만 팔이 벽 근처를 낮게 지나갑니다).

## 오른손 → 왼손 컵 넘김 (`/handover_cup_back`)

`CupHandoverMotion`을 주는 손 `right`로 하나 더 만들어 `cup_right_to_left` 세 시퀀스를 실행합니다. 정방향 넘김의
자세·경로를 거꾸로 씁니다(새 IK 없음).

| 시퀀스 | 단계 | 정방향 대응 |
|---|---|---|
| `cup_right_to_left` | `receiver_open`(왼손 열기 확인) | 시퀀스가 비면 안 되므로 둔 단계 |
| (`CUP_CONTACT_SCENE`) | 왼손 손가락·손바닥과 컵의 접촉 허용 | 같음 |
| `cup_right_to_left_grasp` | `cup_insert`(왼손 LIN, 0.38 −0.054 0.947) → `receiver_close`(0.8) | `cup_retreat`의 반대 |
| (`CUP_DETACH`, `CUP_ATTACH`) | 컵 모델을 오른손에서 왼손으로 | 같음 |
| `cup_right_to_left_release` | `giver_release` → `cup_retreat`(오른손 LIN 20 cm 뒤) → `giver_via` → `giver_home` | `cup_insert`, `cup_ready_via`의 반대 |

로봇에서 `6` → `7` → `8` → `9`가 첫 시도에 통과했습니다(빈 컵).

## 붓기 (`/pour_cup`, `CupPourMotion`)

`handover.yaml`의 `cup_pour_right` 시퀀스 하나를 실행합니다. 장면은 바꾸지 않습니다(컵은 오른손에 붙은 채).
취소하면 그 자리에 멈추고 되돌리지 않습니다.

| 단계 | 내용 | 플래너 |
|---|---|---|
| `pour_above` | 손을 컵 축 기준 −90° 돌려(앞쪽) 바구니 위로, 컵 바닥은 벽 위 3 cm | Pilz PTP(관절) |
| `pour_lower` | 붓는 높이로 내림 | Pilz LIN |
| `pour_tilt` | TCP 고정, +y(오른쪽 → 왼쪽)로 130° 회전 | Pilz LIN |
| `pour_hold` | 2초 대기 | wait |
| `pour_untilt`, `pour_raise` | 되돌리고 올림 | Pilz LIN |
| `pour_return` | 넘김 자세(시작 관절값)로 복귀 | Pilz PTP(관절) |

- 목표는 `scripts/generate_cup_pour_targets.py`로 만들었습니다. 오른손이 컵을 든 상태(`7` 이후)와 장면의
  바구니를 읽고, 130°에서 컵 입구가 벽 위 5 cm, 바구니 중심에서 오른쪽 5 cm(`--shifts=-0.05`)에 오도록 TCP를
  정합니다. 컵과 바구니·테이블·로봇의 충돌을 모두 보고, 관절 한계 0.05 rad 안의 해는 버립니다.
- 손 방향을 그대로(+y) 두면 바구니 위로 IK가 없습니다. −90°의 첫 해는 관절 2가 한계(0)에 붙어서 한계 여유
  점검을 넣었습니다. −75° 이하로 돌리면 기울일 때 컵 몸통이 오른쪽 벽(`basket_wall_x-`)에 닿습니다.
- 로봇 결과: 60°에서는 내용물이 떨어지지 않았고, 130°(오른쪽 5 cm)에서 내용물이 모두 바구니에 떨어졌습니다.
  컵이 흔들리거나 미끄러지지 않았습니다(빈 컵 → 가벼운 내용물).

## 사용한 값

| 항목 | 값 | 근거 |
|---|---|---|
| 왼손 잡는 자세 | 방향 0°, 손끝 15° 아래, 컵 바닥 + 7.5 cm, 패드 중심이 컵 축을 2.5 cm 지남 | 수평(0°)이면 팔이 닿지 않음. 10~20° 기울이면 yaw −45°~+30°에서 모두 IK 가능 |
| 넘김 지점 | base_link x 0.38, y 0.0, 컵 바닥 z 0.88 | 후보 12곳이 모두 통과해서 여유가 큰 가운데 값 |
| 왼손 넘김 자세 | 방향 −90°, 15° 기울임, 7.5 cm | 컵을 든 채 수직축으로만 90° 돌림(컵은 계속 세워짐) |
| 오른손 | 방향 +90°, 수평, 컵 바닥 + 12 cm, 패드 중심이 컵 축을 1 cm 지남 | 2.5 cm는 `receiver_close`에서 충돌 판정 |
| 오른손 대기 | 잡는 위치 20 cm 뒤 | 12 cm는 경로가 컵·왼손과 충돌, 25 cm는 관절 7이 한계값, 30 cm 이상은 IK 없음 |
| 컵 | 높이 0.165, 충돌 모델 반지름 0.05(실제 약 0.043) | `approach.yaml`, `pick.yaml` |

## 사용한 IK

**목표 관절값을 만드는 데는 MoveIt의 `/compute_ik`(KDL)를 썼습니다.**

- 솔버: `ffw_moveit_config/config/kinematics.yaml`의 `arm_l`, `arm_r` 그룹 모두
  `kdl_kinematics_plugin/KDLKinematicsPlugin`(search_resolution 0.005, timeout 0.005). 도구에서는 요청마다
  timeout 0.5 s를 지정합니다. 7축이라 같은 손 자세에 해가 여러 개이고, KDL은 수치 해법이라 **시작값(seed)에 따라
  해가 달라지고 해를 못 찾을 수도 있습니다.**
- 사용 방식: 실제 로봇의 현재 관절 상태를 seed로 쓰고, 이전 구간의 해를 다음 구간의 seed로 이어 넣습니다
  (`scripts/cup_layout_check.py`의 `check`, `line`). `avoid_collisions=false`로 IK만 풀고, 충돌은 해에 대해
  `/check_state_validity`로 따로 봅니다. 직선 구간은 2 cm·8° 간격으로 나눠 점마다 풉니다.
- 풀어서 YAML에 적은 것은 관절 목표(`cup_carry`, `cup_ready`)뿐이고, 직선 구간(`cup_insert`, `cup_retreat`)은
  TCP 자세(`base_link`)를 적었습니다. 직선 구간의 관절 해는 실행 시 Pilz LIN이 내부 IK(체인 KDL)로 풉니다.
- 쓰지 않은 것: DYNAMIXEL 집기의 해석 IK(`ffw_pick/analytic_ik.py`), DYNAMIXEL 핸드오버 목표를 만든
  `scripts/generate_sg2_targets.py`의 scipy 최소제곱 IK. 컵 핸드오버 목표는 MoveIt IK 기반이라
  **로봇 모델과 현재 장면이 있어야 만들 수 있습니다**(데모가 켜져 있어야 함).
- 오프라인 IK로 만든 목표와 실제 로봇은 다릅니다: 관절 허용 오차(`tolerance 0.0175`,
  `planning_tolerance 0.0005`) 안에서 어긋나고, 컵 위치는 실제로 잰 값(손목 depth)입니다.

## 도구

| 도구 (모두 읽기 전용, 로봇을 움직이지 않음) | 용도 |
|---|---|
| `scripts/generate_cup_handover_targets.py` | 넘김 목표 관절값·자세를 IK로 풀어 YAML 출력. `--simultaneous`는 양팔 동시(시험 실패, 아래) |
| `scripts/cup_layout_check.py` | 넘김 지점 후보의 도달·충돌 점검, 위 도구들의 공용 라이브러리 |
| `scripts/right_ready_check.py` | `cup_carry` 직후 자세에서 오른손 대기 거리 후보별 경로 충돌 점검 |
| `scripts/generate_cup_pour_targets.py` | 붓기 목표 생성. 손 방향·붓는 방향·위치 후보별 표와 `cup_pour_right` YAML 출력 |
| `scripts/ptp_path_check.py` | YAML의 관절 단계를 지금 상태에서 PTP로 갈 때 시작·경로 충돌 점검, `--via-search`로 경유 자세 탐색 |
| `scripts/basket_info.py` | 장면의 바구니·테이블 위치와 크기 출력 |

모두 `full_demo`가 켜져 있어야 합니다(`/compute_ik`, `/check_state_validity`, `/get_planning_scene`).
컨테이너 `/tmp`에 복사해서 실행하고, 같은 폴더의 `cup_layout_check.py`를 함께 복사해야 합니다.

## 알려진 문제와 제한

**IK와 목표**
- 목표 관절값은 한 지점(넘김 지점, 컵 측정 x 0.447 y 0.237)에서 풀었습니다. `cup_carry`는 현재 관절에서
  고정 목표로 가는 PTP라서, 컵을 잡은 위치가 크게 다르면 경로가 달라지고 충돌하면 계획이 실패합니다(움직이지는 않음).
  다른 위치에서 검증하지 않았습니다.
- 오른손 대기 자세의 관절 7이 한계(1.5804)에 가깝습니다(1.519). 거리 25 cm는 한계에 걸렸고 30 cm 이상은 IK가 없습니다.
- 리프트·초기 자세·넘김 지점을 바꾸면 목표를 다시 만들어야 합니다.

- 붓기 목표는 바구니가 (0.412, −0.247), yaw 90°일 때 푼 고정값입니다. 측정마다 바구니가 2 cm 정도 달라졌고,
  −90°에서 오른쪽 이동량 3~7 cm가 모두 통과해서 y로 ±2 cm 여유가 있습니다. 크게 옮기면 다시 만들어야 합니다.

**충돌 모델**
- 컵 충돌 모델이 실제 컵보다 굵습니다(반지름 0.05 대 약 0.043). 손바닥이 모델에 닿는 깊이가 약 2.7 cm라서
  삽입 깊이를 2.5 cm로 두면 `receiver_close`가 충돌로 막혔습니다. 오른손 깊이는 1 cm로 줄였고, 접촉 허용에
  손바닥도 포함돼 있습니다. 허용 구간에서는 실제 접촉이 계획에서 막히지 않습니다.
- 도구 점검은 컵 접촉을 무시하는 모드(`check`)와 포함하는 모드(`state_collisions(ignore_cup=False)`)가 섞여
  있습니다. 넘김 후보 점검(`cup_layout_check.py`)은 컵 충돌을 보지 않습니다.
- 경로 점검은 일정 간격 표본이라 표본 사이의 짧은 충돌은 놓칠 수 있습니다. 실제 실행 때는 Pilz와 평활화 단계가
  다시 검사합니다.

**동작과 시간**
- 두 팔이 동시에 움직이지 않습니다. OMPL 양팔(`dual_joint`)은 경로가 무작위라 팔이 높이 올라가고 컵이 크게 기울어
  내용물이 쏟아질 정도였고, Pilz PTP를 양팔 그룹에 적용하면(`pipeline`/`planner` 지정) `MoveIt error 99999`로
  계획에 실패했습니다. 동시 이동은 실행기가 각 팔을 동시에 실행하도록 고쳐야 가능합니다.
- 왼손 `cup_carry`는 Pilz PTP라 이동 중 컵이 약간 기웁니다(한 위치에서 최대 5.9°).
- 오른손 삽입은 정해진 위치로 직선 이동합니다. 손가락 사이(최대 개폭 90 mm, 컵 지름 약 85~88 mm)에 여유가
  1~2 mm뿐이라 컵이 한쪽으로 치우치면 걸릴 수 있습니다. 센서로 보정하지 않습니다.

**장면(MoveIt 컵 모델)**
- 컵 모델은 `PICK_ATTACH`(왼손) → `CUP_ATTACH`(오른손)로 따라가지만, 중간에 실패하거나 취소하면 장면과 실제가
  어긋난 채 남을 수 있습니다. 초기 자세(`1`, `d`)가 그리퍼를 열기 전에 손에 붙은 컵 모델을 떼고 삭제하므로,
  `1`이 항상 장면을 정리합니다.
- 7 이후 오른손이 컵을 든 채 정지합니다. 이때 `1`을 누르면 그리퍼가 열려 컵이 떨어집니다.
- 컵을 잡았는지는 센서로 확인하지 않습니다(`gripper_wait_only`, `holding_state: UNVERIFIED`).

**진단**
- `trajectory_smoother.cpp`가 평활화 경로 충돌 시 접촉한 링크 쌍을 메시지에 붙입니다
  (`Smoothed path is in collision; A / B`). 계획 단계의 MoveIt 실패(`-2`, `-31`, `99999`)는 원인이 코드만으로
  구분되지 않아서, 필요하면 위 도구로 재현해야 합니다.
- 저장된 손목 컵 위치는 왼팔이 초기 자세를 떠난 뒤 120 s가 지나면 만료됩니다(`CUP_NOT_FOUND`).
- `d`에서 셀 핸드오버 `open_approach`가 `Robot moved during planning: gripper_r_joint1`로 한 번 실패했습니다
  (집은 직후 그리퍼가 아직 움직이는 중으로 보임). 바로 `3`을 실행하면 통과했습니다. 원인은 아직 보지 않았습니다.

## 시험 절차

```bash
# 호스트: C++ 변경이 있을 때만 복사 후 빌드. 설정만 바뀌었으면 아래 두 줄과 full_demo 재시작
docker cp config/handover.yaml docker-demo_1f-1:/root/ros2_ws/src/demo_1f/config/handover.yaml
docker cp config/handover.yaml docker-demo_1f-1:/root/ros2_ws/install/demo_1f/share/demo_1f/config/handover.yaml
```
`full_demo` → 메뉴 `1`(초기 자세) → `6`(컵 잡기) → `7`(넘김) → `8`(붓기, 바구니가 장면에 있어야 함:
`2` 또는 `d`). 단독 미리보기:
`ros2 action send_goal /handover_cup demo_1f/action/ApproachCup '{arm: left, cup_id: cup, execute: false}'`
`ros2 action send_goal /pour_cup demo_1f/action/ApproachCup '{arm: right, cup_id: cup, execute: false}'`

## 다음 단계

원래 책상 위치에 내려놓기, 전체 데모 연결(`PICK_CUP` 다음에 이어 붙임).
그 뒤 위 문제들을 다듬습니다.
