# Mobin

TurtleBot3 simulation과 실제 board·sensor·motor 실습을 병렬로 진행하고, 검증된 결과를 Yahboom MicroROS-Pi5와 자체 소형 Humanoid의 Sim2Real로 통합하는 리포지토리

## 목표

| 트랙·목표 | 결과 |
|---|---|
| Simulation | TurtleBot3 calibration, warehouse 회피, 재현 가능한 dataset |
| Embedded mobile robot | Yahboom의 SW adapter·safety와 HW motor·sensor·watchdog 검증 |
| Embedded humanoid | 자체 CAD·Raspberry Pi 5·MCU의 SW·HW 계층 제작 |
| 통합 목표 | 학습 policy를 replay·shadow·guarded 순서로 실물에서 검증 |

전체 순서와 단계별 통과 조건은 [PATCH 로드맵](patch/README.md)에 정리되어 있다.

## 디렉토리 구조

```text
mobin/
├── code/                                # 직접 작성하는 코드
│   ├── scripts/                        # calibration 실행 script
│   ├── python/                         # rclpy 학습 package
│   └── cpp/                            # rclcpp 학습 package
├── docker/                              # Jazzy/Gazebo Docker 구성
├── forks/                               # 독립 Git fork 세 개
├── data/                                # bag·calibration 결과
├── trials/                              # 실행 조건·지표·포트폴리오 증거
├── docs/                                # Git·fork·license 안내
└── patch/                               # Patch 단위 구현 절차
    ├── simulation/                       # Docker·Gazebo·학습, PATCH-00부터
    └── embedded/                         # SW·HW 실물 실습, PATCH-00부터
```

## 검증 환경과 trial

개발 기준 환경은 다음과 같다. 각 실행 당시의 commit·CPU·memory·GPU driver·Docker version은 `trials/<이름>/environment.txt`에도 별도로 저장된다.

| 항목 | 개발 기준 |
|---|---|
| Host | Ubuntu 24.04, x86_64 |
| Container | `osrf/ros:jazzy-desktop-full` 기반 `tb3-jazzy-lab:local` |
| ROS/Gazebo | ROS 2 Jazzy, Gazebo Harmonic, `ros_gz` |
| GPU | NVIDIA GeForce GTX 1050 4 GiB, driver 580.173.02 |
| CPU fallback | `compose.yaml`만 사용하면 Mesa software rendering |

| Trial | 조건 | 검증 대상 |
|---|---|---|
| `regression-test` | C++·Python build/test | 기존 기능 회귀 |
| `deterministic_crossing` | person+cart 고정 경로, robot 입력 `0.10 m/s` | association, CPA danger, 정지·복귀 |
| `seeded-multi-person-tracking` | 사람 3명, seed 42, simulation clock 60초 | Camera–LiDAR 공통 ROI의 IDF1, ID switch, FP, FN, sensor coverage |
| `safety-test` | 합성 입력 기반 core test | stale, LiDAR emergency, stable-safe, command watchdog |

실행 명령과 산출물은 [PATCH-07](patch/simulation/PATCH-07-lidar-association-and-tracking.md)과 [PATCH-08](patch/simulation/PATCH-08-warehouse-patrol-semantic-safety.md)을 따른다. 기존 이름의 trial은 자동으로 덮어쓰지 않는다.

## 시작하기

```bash
git clone <상위-저장소-URL> mobin
cd mobin
mkdir -p forks

git clone --branch jazzy \
  https://github.com/JungSeong/turtlebot3_simulations.git \
  forks/turtlebot3_simulations

git clone --recursive \
  https://github.com/JungSeong/direct_visual_lidar_calibration.git \
  forks/direct_visual_lidar_calibration

git clone --branch ros2 \
  https://github.com/JungSeong/aws-robomaker-small-warehouse-world.git \
  forks/aws-robomaker-small-warehouse-world

git -C forks/turtlebot3_simulations remote add upstream \
  https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

git -C forks/direct_visual_lidar_calibration remote add upstream \
  https://github.com/koide3/direct_visual_lidar_calibration.git

git -C forks/aws-robomaker-small-warehouse-world remote add upstream \
  https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
```

## Git remote와 기준 버전

| 로컬 저장소 | `origin`: 수정 결과를 push할 내 fork | `upstream`: 변경을 가져올 원본 | 기준 branch / commit |
|---|---|---|---|
| `forks/turtlebot3_simulations` | `JungSeong/turtlebot3_simulations` | `ROBOTIS-GIT/turtlebot3_simulations` | `jazzy` / `45633014a14e8f438495b532a723e4ad45cbbd31` |
| `forks/direct_visual_lidar_calibration` | `JungSeong/direct_visual_lidar_calibration` | `koide3/direct_visual_lidar_calibration` | `main` / `02a0dc039f5509708f384be4ff3228e0ae09352d` |
| `forks/aws-robomaker-small-warehouse-world` | 내 GitHub fork | `aws-robotics/aws-robomaker-small-warehouse-world` | `ros2` / `ee0af733315e78432408c3cd98d378ecee5f767c` |

수정 전 patch를 위한 브랜치를 생성한다

```bash
git -C forks/turtlebot3_simulations switch -c practice/replace-lidar-with-3d
git -C forks/direct_visual_lidar_calibration switch -c practice/calibration-experiment
git -C forks/aws-robomaker-small-warehouse-world \
  switch -c practice/gazebo-harmonic
```

자세한 clone, branch, upstream 동기화, commit·push 절차는 [Fork workflow와 license 준수](docs/how_to_fork_and_license.md)를 따른다.

## License

| 대상 | 확인된 license | fork 수정·배포 시 핵심 확인 |
|---|---|---|
| TurtleBot3 simulations | Apache-2.0 | LICENSE와 기존 notice 유지, 수정 파일에 변경 사실 표시 |
| direct_visual_lidar_calibration | README·`package.xml`의 MIT 선언 | 정확한 copyright·permission notice 유지 |
| AWS Small Warehouse ROS 2 고정 commit | 루트 `LICENSE`와 `package.xml` 모두 MIT-0 | 두 원문을 보존하고 출처·commit·Harmonic 변경 사실 기록 |
| Ultralytics package·YOLO11 weight | 기본 AGPL-3.0 또는 별도 Enterprise | 공개 portfolio의 전체 license 적합성과 상업·비공개 사용 조건 확인 |

공개·상업 배포 전 [license checklist](docs/how_to_fork_and_license.md#license)를 확인한다.

## 창고 장애물 회피 실행

PATCH-05/06 구현은 `avoidance` service 하나로 world, robot, obstacle, 선택한 controller를 실행한다. 현재 controller는 `/scan` 거리만 사용하는 반응형 회피다. 별도 `perception` service는 YOLO11-seg의 person Instance Label Map과 `/calib/points`를 연결하고, constrained PCL RANSAC으로 바닥을 제거한 뒤 `odom` 기준 Hungarian tracking 결과를 발행한다. PATCH-08의 `patrol` service는 `/tracking/persons`와 LiDAR emergency distance로 `/cmd_vel_mux`를 감속·정지하고, watchdog을 거쳐 `/cmd_vel`을 발행한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

TURTLEBOT3_MODEL=waffle_pi_3d_large \
TURTLEBOT3_WORLD_DIR=/opt/aws_warehouse/worlds/small_warehouse \
TURTLEBOT3_WORLD=small_warehouse_harmonic.world \
AVOIDANCE_SCENARIO=crossing \
AVOIDANCE_IMPLEMENTATION=cpp \
AVOIDANCE_START_DELAY=10.0 \
X_POSE=-2.0 Y_POSE=-0.5 YAW=0.0 \
GAZEBO_GUI=true LAUNCH_RVIZ=true \
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  up --force-recreate avoidance
```

`waffle_pi_3d_large`는 warehouse 가시성을 위한 3배 model이다. Calibration 기준 크기는 원본 `waffle_pi_3d`를 사용한다. `crossing`의 사람과 pallet jack는 같은 model link에 속해 하나의 pose로 항상 함께 이동한다. 사람+cart는 통로의 먼 지점 `(0.9,-5.0)`에서 출발해 +y 방향으로 접근하므로 robot camera에 갑자기 나타나지 않는다. Scenario 전용 robot 속도는 `0.14 m/s`, 오른쪽 LiDAR sector 범위는 `15~85°`다. 오른쪽 sector가 점유된 동안 정지하고, 사람+cart가 통과하면 오른쪽으로 회피한다. 전체 parameter·license·검증은 [PATCH-05](patch/simulation/PATCH-05-obstacle-scenarios.md)와 [PATCH-06](patch/simulation/PATCH-06-obstacle-avoidance.md)을 따른다.

다인 검증은 `AVOIDANCE_SCENARIO=multi_person`, `PEDESTRIAN_COUNT=2~5`, `PEDESTRIAN_RANDOM_SEED=42`, `PEDESTRIAN_PATH_MODE=random_walk`을 위 명령에 추가한다. 고정 `crossing`은 사람+cart 회귀시험에만 유지한다. 전체 Nav2 patrol·tracking·semantic safety 실행은 [PATCH-08](patch/simulation/PATCH-08-warehouse-patrol-semantic-safety.md)을 따른다. Simulation이 실행된 상태에서 GPU perception을 별도 terminal에 시작한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
YOLO_DEVICE=0 docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  up --force-recreate perception
```

Gazebo를 재시작하면 simulation clock도 0으로 돌아간다. 기존 TF 시간을 남기지 않도록 `perception`도 위 명령의 `--force-recreate`로 다시 시작한다.

`perception`은 gitignored `model/yolo-seg/yolo11s-seg.pt`를 사용한다. Weight 준비와 RANSAC parameter는 [PATCH-07](patch/simulation/PATCH-07-lidar-association-and-tracking.md)을 따른다.

## Perception·safety 시각화

| 화면·topic | 보여주는 내용 | 포트폴리오 용도 |
|---|---|---|
| `/camera/detections_annotated` | 동기화 frame의 모든 YOLO bbox; 성공 시 초록 point·`range`, 실패 시 `YOLO only` | Camera 검출과 LiDAR association을 분리해 확인 |
| `/segmentation/instance_labels` | `mono8`: 0=배경, 1..N=frame-local person instance | segmentation과 Detection2D 대응 확인 |
| `/fusion/ground_points`, `/fusion/non_ground_points` | RANSAC이 분리한 ground/non-ground point | 바닥 제거 오판 확인 |
| `/fusion/ground_plane`, `/fusion/diagnostics` | 채택한 plane과 inlier·원점거리 | RANSAC 제약 검증 |
| RViz `/tracking/markers` | `odom` 기준 track ID·위치·속도·불확실성 | 다인 tracking 근거 |
| RViz `/safety/markers` | 선택된 사람의 safety radius와 CPA 지점 | 충돌 예측 근거 |
| `cpa-stop.png` | `t_cpa`·`d_cpa`·`d_safe`와 `/cmd_vel_mux → /cmd_vel_safe` | CPA 위험 판정으로 정지한 근거 |
| `stale-failsafe.png` | scan·odom·tracking age, timeout, 입력·안전 명령 | 입력 갱신 지연에 반응한 근거 |

원본 영상 해상도의 YOLO binary mask(`retina_masks=True`, `masks.data`)를 직접 `mono8` label로 변환한다. Polygon 외곽선 추출·재채움은 하지 않아 mask의 구멍과 분리 영역을 보존한다. Mask 해상도가 원본 영상과 다르면 거부하며, instance끼리 겹친 pixel은 기존처럼 뒤 label이 우선한다.

C++ association은 전체 유효 cloud를 Camera로 투영해 **동일한 반올림 pixel**의 최소 깊이보다 `camera_occlusion_depth_tolerance` 초과 뒤쪽인 점을 instance 후보에서 제외한다. `yolo_lidar_fusion.launch.py camera_occlusion_depth_tolerance:=0.05`로 설정하며 유한한 실수 `>=0`, 단위 m다. 기본 5cm는 초기 허용 오차이며 실측 보증값이 아니다. 이웃 pixel·허용 오차 이내 점은 보존한다. 지면·label 필터 전의 모든 표면을 깊이 근거로 쓰며, `/fusion/non_ground_points`는 가림 제외 전 지면 분리 결과다. 같은 pixel도 시선 차이가 있으므로 완전한 가림 증명은 아니며 앞쪽 반사점 없이 벽만 관측하면 오채택이 남는다.

Annotated image는 평상시 overhead를 피하려고 기본 비활성화다. Trial에서는 `PUBLISH_ANNOTATED_IMAGE=true`로 자동 활성화된다. 가림·지면 제외 후 mask 내부의 최근접 유효 깊이 후보가 나머지 유효 후보 전체보다 `foreground_support_ratio`배 이상 많은 point를 가질 때 채택한다. 기본값은 `2.0`이며, `yolo_lidar_fusion.launch.py foreground_support_ratio:=2.0`으로 설정한다. 비율은 유한한 무차원 실수 `>1`이어야 하며 클수록 거부가 늘어난다. 이는 전경 지지에 대한 휴리스틱이며 사람임을 보장하지 않는다. `/fusion/diagnostics`와 `evidence.json.association_outcomes`는 `no_points`, `multiple_depth_candidates`(복수 후보 중 전경 지지 부족), `ground_rejection`, `tf_unavailable`을 누적한다. `camera_occluded_points`는 처리된 cloud 전체의 누적 제외 **점 수**이며 instance 거부 건수와 합산하지 않는다. 가림 때문에 점이 부족한 instance는 `no_points`로 분류하고 지면 거부에 포함하지 않는다. `association.png`는 **같은 instance**의 bbox가 image 안에 80% 이상 있고 경계에 닿지 않으며 association도 성공한 첫 frame만 저장한다. `cpa-stop.png`와 `stale-failsafe.png`는 해당 event가 실제 발생해야 생성된다.

## TODO

1. [O] [Fork clone·수정·license 준수](docs/how_to_fork_and_license.md)

### Simulation

1. [O] [PATCH-00: Jazzy·Gazebo Docker 환경](patch/simulation/PATCH-00-jazzy-gazebo-docker-setup.md)
2. [ ] [PATCH-01: 2D LiDAR 측정을 3D LiDAR 측정으로 교체](patch/simulation/PATCH-01-replace-2d-lidar-with-3d-lidar.md)
3. [ ] [PATCH-02: Calibration scene과 MCAP](patch/simulation/PATCH-02-calibration-scene-recording.md)
4. [ ] [PATCH-03: Extrinsic 계산](patch/simulation/PATCH-03-run-calibration.md)
5. [ ] [PATCH-04: URDF 반영과 정량 검증](patch/simulation/PATCH-04-apply-and-verify.md)
6. [O] [PATCH-05: AWS Warehouse의 Gazebo Harmonic 이식](patch/simulation/PATCH-05-obstacle-scenarios.md)
7. [O] [PATCH-06: 장애물 회피 node](patch/simulation/PATCH-06-obstacle-avoidance.md)
8. [ ] [PATCH-07: Person LiDAR–BBox Association과 Tracking](patch/simulation/PATCH-07-lidar-association-and-tracking.md)
9. [ ] [PATCH-08: Warehouse Patrol과 Semantic Safety](patch/simulation/PATCH-08-warehouse-patrol-semantic-safety.md)
10. [ ] [PATCH-15: GitHub Actions CI/CD](patch/simulation/PATCH-15-github-actions-ci-cd.md)
11. [ ] [PATCH-16: Sim2Real dataset과 domain randomization](patch/simulation/PATCH-16-sim2real-dataset-collection.md)
12. [ ] [PATCH-17: 저조도 터널 calibration 강건성](patch/simulation/PATCH-17-low-light-tunnel-robustness.md)
13. [ ] [PATCH-18: Behavior Tree calibration workflow](patch/simulation/PATCH-18-behavior-tree-calibration-orchestration.md)
14. [ ] [PATCH-19: Mobile Robot 강화학습](patch/simulation/PATCH-19-mobile-robot-reinforcement-learning.md)
15. [ ] [PATCH-20: Humanoid Isaac Lab Sim2Real](patch/simulation/PATCH-20-humanoid-isaac-lab-sim2real.md)

### Embedded

1. [ ] [PATCH-00: Yahboom bring-up과 interface](patch/embedded/PATCH-00-yahboom-hardware-bringup.md)
2. [ ] [PATCH-01: Yahboom 안전 장애물 회피](patch/embedded/PATCH-01-yahboom-safe-obstacle-avoidance.md)
3. [ ] [PATCH-02: Yahboom RL Sim2Real](patch/embedded/PATCH-02-yahboom-rl-sim2real.md)
4. [ ] [PATCH-03: Humanoid reference와 요구사항](patch/embedded/PATCH-03-humanoid-reference-and-requirements.md)
5. [ ] [PATCH-04: Humanoid CAD와 URDF](patch/embedded/PATCH-04-humanoid-cad-and-urdf.md)
6. [ ] [PATCH-05: Humanoid 전장과 ROS 2 제어](patch/embedded/PATCH-05-humanoid-electronics-and-control.md)

두 목록은 병렬로 진행한다. 실물 command를 허용하는 통합 조건은 [PATCH 로드맵의 통합 지점](patch/README.md#통합-지점)을 따른다.
