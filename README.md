# Mobin

TurtleBot3 Gazebo simulation에서 2D/3D LiDAR-Camera Extrinsic Calibration 및 DQN 강화학습을 실습해 보기 위한 리포지토리

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
2. [O] [Jazzy·Gazebo Docker 환경 구성](patch/PATCH-00-jazzy-gazebo-docker-setup.md)
3. [ ] [기존 2D LiDAR를 3D LiDAR로 교체](patch/PATCH-01-replace-2d-lidar-with-3d-lidar.md)
4. [ ] [Calibration 월드와 rosbag](patch/PATCH-02-calibration-scene-recording.md)
5. [ ] [Extrinsic 계산](patch/PATCH-03-run-calibration.md)
6. [ ] [URDF 적용과 정량 검증](patch/PATCH-04-apply-and-verify.md)
7. [ ] [AWS Warehouse를 Gazebo Harmonic으로 이식](patch/PATCH-05-obstacle-scenarios.md)
8. [ ] [장애물 회피 노드](patch/PATCH-06-obstacle-avoidance.md)
9. [ ] [저조도 터널 Extrinsic 강건성 평가](patch/PATCH-07-low-light-tunnel-robustness.md)
10. [ ] [Behavior Tree 기반 Calibration Workflow 조정](patch/PATCH-08-behavior-tree-calibration-orchestration.md)
11. [ ] [GitHub Actions CI/CD](patch/PATCH-09-github-actions-ci-cd.md)
12. [ ] [Sim2Real Dataset 수집과 Domain Randomization](patch/PATCH-10-sim2real-dataset-collection.md)
