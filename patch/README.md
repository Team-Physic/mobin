# Mobile Robot Camera-LiDAR Lab

## 내려받은 리포

| 디렉터리 | 기준 브랜치 / 커밋 | 용도 |
|---|---|---|
| `forks/turtlebot3_simulations/` | `jazzy` / `45633014a14e8f438495b532a723e4ad45cbbd31` | Gazebo Sim 로봇, 센서, 월드, 회피 노드 |
| `forks/direct_visual_lidar_calibration/` | `main` / `02a0dc039f5509708f384be4ff3228e0ae09352d` | 3D LiDAR-Camera extrinsic calibration |

현재 호스트는 Ubuntu 24.04이며 `/opt/ros`가 없으므로 ROS 2 Jazzy와 Gazebo Harmonic은 Docker에서 실행한다.

## 기본 선택

- 로봇: `TURTLEBOT3_MODEL=waffle_pi`
- 기존 2D LDS `/scan`: 장애물 회피용으로 유지
- 추가 3D LiDAR `/calib/points`: extrinsic calibration 전용
- 카메라: `/camera/image_raw`, `/camera/camera_info`
- 보정 방법: 라이선스와 GPU 의존성이 적은 manual initial guess 사용
- 두 source는 `forks/`의 독립 Git 저장소에서 수정
- Calibration 실행 이미지는 로컬 calibration fork의 Jazzy Dockerfile로 build

Fork clone, `origin`/`upstream`, 실습 branch, license 의무는 [Fork workflow와 license 준수](../docs/fork_workflow_and_licensing.md)를 먼저 따른다.

## Patch 순서

1. [Fork clone·수정·license 준수](../docs/fork_workflow_and_licensing.md)
2. [PATCH-00: Jazzy·Gazebo Docker 환경 구성](PATCH-00-jazzy-gazebo-docker-setup.md)
3. [PATCH-01: 3D LiDAR와 센서 프레임](PATCH-01-add-3d-lidar.md)
4. [PATCH-02: Calibration 월드와 rosbag](PATCH-02-calibration-scene-recording.md)
5. [PATCH-03: Extrinsic 계산](PATCH-03-run-calibration.md)
6. [PATCH-04: URDF 적용과 정량 검증](PATCH-04-apply-and-verify.md)
7. [PATCH-05: 정적·동적 장애물 시나리오](PATCH-05-obstacle-scenarios.md)
8. [PATCH-06: 장애물 회피 노드](PATCH-06-obstacle-avoidance.md)
9. [PATCH-07: 저조도 터널 Extrinsic 강건성 평가](PATCH-07-low-light-tunnel-robustness.md) — PATCH-04 이후 선택 확장
10. [PATCH-08: Behavior Tree 기반 Calibration Workflow 조정](PATCH-08-behavior-tree-calibration-orchestration.md) — PATCH-04 이후 실행 자동화 확장
11. [PATCH-09: GitHub Actions CI/CD](PATCH-09-github-actions-ci-cd.md) — build 재현성과 development image delivery
12. [PATCH-10: Sim2Real Dataset 수집과 Domain Randomization](PATCH-10-sim2real-dataset-collection.md) — episode MCAP, LeRobot 변환, 제어 tuning과 domain randomization

PATCH-00부터 PATCH-06은 앞 번호 patch의 완료 조건을 통과한 뒤 진행한다. PATCH-07은 PATCH-04까지만 완료하면 PATCH-05/06과 독립적으로 수행할 수 있다.
PATCH-08의 calibration capture·평가 흐름은 PATCH-04 완료 후 적용한다. 장애물 회피를 포함한 pose 이동 복구까지 연결하려면 PATCH-05/06도 먼저 완료한다.
PATCH-09의 기본 CI는 PATCH-00 이후 적용할 수 있다. Python/C++ test와 headless Gazebo 검증은 해당 구현 PATCH가 완료된 뒤 확장한다.
PATCH-10은 PATCH-02의 bag 기록 규약을 확장한다. DQN/회피 데이터는 PATCH-05/06 이후, calibration robustness 데이터는 PATCH-07 이후 수집한다.

## 의도적으로 제외한 범위

- 기본 2D LDS를 calibration 입력으로 변환하지 않는다. `direct_visual_lidar_calibration`은 intensity가 포함된 `PointCloud2`가 필요하다.
- Calibration 중 동적 장애물을 사용하지 않는다. 움직이는 물체는 영상과 포인트클라우드 대응을 깨뜨린다.
- Nav2 전체 스택은 먼저 넣지 않는다. PATCH-06의 단순 회피가 통과한 뒤 목표점 주행이 필요할 때 추가한다.
