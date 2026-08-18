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
mobile-robot-calibration-repo/
├── docker/                               # Jazzy/Gazebo Docker 구성
├── docs/                                 # Git·fork·license 안내
├── forks/                               # 상위 저장소가 추적하지 않는 독립 Git 저장소
│   ├── turtlebot3_simulations/          # 내 TurtleBot3 fork
│   ├── direct_visual_lidar_calibration/ # 내 calibration fork
│   └── aws-robomaker-small-warehouse-world/ # 내 warehouse asset fork
└── patch/                                # Patch 단위 구현 절차
    ├── simulation/                       # Docker·Gazebo·학습, PATCH-00부터
    └── embedded/                         # SW·HW 실물 실습, PATCH-00부터
```

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

공개·상업 배포 전 [license checklist](docs/how_to_fork_and_license.md#license)를 확인한다.

## TODO

1. [O] [Fork clone·수정·license 준수](docs/how_to_fork_and_license.md)

### Simulation

1. [O] [PATCH-00: Jazzy·Gazebo Docker 환경](patch/simulation/PATCH-00-jazzy-gazebo-docker-setup.md)
2. [ ] [PATCH-01: 2D LiDAR 측정을 3D LiDAR 측정으로 교체](patch/simulation/PATCH-01-replace-2d-lidar-with-3d-lidar.md)
3. [ ] [PATCH-02: Calibration scene과 MCAP](patch/simulation/PATCH-02-calibration-scene-recording.md)
4. [ ] [PATCH-03: Extrinsic 계산](patch/simulation/PATCH-03-run-calibration.md)
5. [ ] [PATCH-04: URDF 반영과 정량 검증](patch/simulation/PATCH-04-apply-and-verify.md)
6. [ ] [PATCH-05: AWS Warehouse의 Gazebo Harmonic 이식](patch/simulation/PATCH-05-obstacle-scenarios.md)
7. [ ] [PATCH-06: 장애물 회피 node](patch/simulation/PATCH-06-obstacle-avoidance.md)
8. [ ] [PATCH-07: 저조도 터널 calibration 강건성](patch/simulation/PATCH-07-low-light-tunnel-robustness.md)
9. [ ] [PATCH-08: Behavior Tree calibration workflow](patch/simulation/PATCH-08-behavior-tree-calibration-orchestration.md)
10. [ ] [PATCH-09: GitHub Actions CI/CD](patch/simulation/PATCH-09-github-actions-ci-cd.md)
11. [ ] [PATCH-10: Sim2Real dataset과 domain randomization](patch/simulation/PATCH-10-sim2real-dataset-collection.md)
12. [ ] [PATCH-11: Mobile Robot 강화학습](patch/simulation/PATCH-11-mobile-robot-reinforcement-learning.md)
13. [ ] [PATCH-12: Humanoid Isaac Lab Sim2Real](patch/simulation/PATCH-12-humanoid-isaac-lab-sim2real.md)

### Embedded

1. [ ] [PATCH-00: Yahboom bring-up과 interface](patch/embedded/PATCH-00-yahboom-hardware-bringup.md)
2. [ ] [PATCH-01: Yahboom 안전 장애물 회피](patch/embedded/PATCH-01-yahboom-safe-obstacle-avoidance.md)
3. [ ] [PATCH-02: Yahboom RL Sim2Real](patch/embedded/PATCH-02-yahboom-rl-sim2real.md)
4. [ ] [PATCH-03: Humanoid reference와 요구사항](patch/embedded/PATCH-03-humanoid-reference-and-requirements.md)
5. [ ] [PATCH-04: Humanoid CAD와 URDF](patch/embedded/PATCH-04-humanoid-cad-and-urdf.md)
6. [ ] [PATCH-05: Humanoid 전장과 ROS 2 제어](patch/embedded/PATCH-05-humanoid-electronics-and-control.md)

두 목록은 병렬로 진행한다. 실물 command를 허용하는 통합 조건은 [PATCH 로드맵의 통합 지점](patch/README.md#통합-지점)을 따른다.
