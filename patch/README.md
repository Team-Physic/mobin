# Mobin Robot Sim2Real Roadmap

이 프로젝트는 세 단계를 순서대로 진행한다.

| 단계 | 목표 | 완료 지점 |
|---|---|---|
| 1. Simulation 기반 | TurtleBot3에서 Camera-LiDAR calibration, warehouse 회피, dataset 수집 | PATCH-00~10 |
| 2. 실물 mobile robot | Yahboom MicroROS-Pi5에서 안전 회피와 강화학습 policy 검증 | PATCH-11~14 |
| 3. 자체 Humanoid | 직접 만든 CAD·전장·제어를 Isaac Lab에서 학습해 실물에 적용 | PATCH-15~18 |

**앞 단계의 인터페이스·데이터·안전 검증을 다음 robot에 재사용한다. TurtleBot3 simulation에서 곧바로 Humanoid 학습으로 건너뛰지 않는다.**

## 내려받은 리포

| 디렉터리 | 기준 브랜치 / 커밋 | 용도 |
|---|---|---|
| `forks/turtlebot3_simulations/` | `jazzy` / `45633014a14e8f438495b532a723e4ad45cbbd31` | Gazebo Sim 로봇, 센서, 월드, 회피 노드 |
| `forks/direct_visual_lidar_calibration/` | `main` / `02a0dc039f5509708f384be4ff3228e0ae09352d` | 3D LiDAR-Camera extrinsic calibration |
| `forks/aws-robomaker-small-warehouse-world/` | `ros2` / `ee0af733315e78432408c3cd98d378ecee5f767c` | Gazebo Classic용 ROS 2 package에서 Harmonic으로 이식할 SDF·mesh·map |

향후 PATCH-13 구현 시 [ROBOTIS `turtlebot3_machine_learning`](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning)의 공식 `jazzy` branch를 내 계정으로 fork해 `forks/turtlebot3_machine_learning/`에 추가한다. 지금은 아직 내려받거나 dependency로 고정하지 않는다.

현재 호스트는 Ubuntu 24.04이며 `/opt/ros`가 없으므로 ROS 2 Jazzy와 Gazebo Harmonic은 Docker에서 실행한다.

## 기본 선택

- 로봇: `TURTLEBOT3_MODEL=waffle_pi`
- 원본 `waffle_pi`: 기존 2D LDS `/scan` 유지
- 파생 `waffle_pi_3d`: 기존 `base_scan` 측정을 3D로 교체하고 `/calib/points`를 calibration에 사용
- 카메라: `/camera/image_raw`, `/camera/camera_info`
- 보정 방법: 라이선스와 GPU 의존성이 적은 manual initial guess 사용
- 세 source는 `forks/`의 독립 Git 저장소에서 수정
- Calibration 실행 이미지는 로컬 calibration fork의 Jazzy Dockerfile로 build

Fork clone, `origin`/`upstream`, 실습 branch, license 의무는 [Fork workflow와 license 준수](../docs/how_to_fork_and_license.md)를 먼저 따른다.

## Patch 순서

1. [Fork clone·수정·license 준수](../docs/how_to_fork_and_license.md)
2. [PATCH-00: Jazzy·Gazebo Docker 환경 구성](PATCH-00-jazzy-gazebo-docker-setup.md)
3. [PATCH-01: 기존 2D LiDAR를 3D LiDAR로 교체](PATCH-01-replace-2d-lidar-with-3d-lidar.md)
4. [PATCH-02: Calibration 월드와 rosbag](PATCH-02-calibration-scene-recording.md)
5. [PATCH-03: Extrinsic 계산](PATCH-03-run-calibration.md)
6. [PATCH-04: URDF 적용과 정량 검증](PATCH-04-apply-and-verify.md)
7. [PATCH-05: AWS Warehouse를 Gazebo Harmonic으로 이식](PATCH-05-obstacle-scenarios.md)
8. [PATCH-06: 장애물 회피 노드](PATCH-06-obstacle-avoidance.md)
9. [PATCH-07: 저조도 터널 Extrinsic 강건성 평가](PATCH-07-low-light-tunnel-robustness.md) — PATCH-04 이후 선택 확장
10. [PATCH-08: Behavior Tree 기반 Calibration Workflow 조정](PATCH-08-behavior-tree-calibration-orchestration.md) — PATCH-04 이후 실행 자동화 확장
11. [PATCH-09: GitHub Actions CI/CD](PATCH-09-github-actions-ci-cd.md) — build 재현성과 development image delivery
12. [PATCH-10: Sim2Real Dataset 수집과 Domain Randomization](PATCH-10-sim2real-dataset-collection.md) — episode MCAP, LeRobot 변환, 제어 tuning과 domain randomization
13. [PATCH-11: Yahboom 실물 연결과 interface 고정](PATCH-11-yahboom-hardware-bringup.md) — vendor 환경 inventory, topic·TF·watchdog 실측
14. [PATCH-12: Yahboom 안전 장애물 회피](PATCH-12-yahboom-safe-obstacle-avoidance.md) — safety supervisor, simulation·real 공통 metric
15. [PATCH-13: Mobile Robot 강화학습](PATCH-13-mobile-robot-reinforcement-learning.md) — 공식 TurtleBot3 Jazzy DQN baseline과 randomization 평가
16. [PATCH-14: Yahboom RL Sim2Real](PATCH-14-yahboom-rl-sim2real.md) — replay, shadow, guarded 실물 승격
17. [PATCH-15: 공개 Humanoid와 설계 요구사항](PATCH-15-humanoid-reference-and-requirements.md) — reference·license 검증, biped MVP, actuator rig
18. [PATCH-16: 자체 Humanoid CAD와 URDF](PATCH-16-humanoid-cad-and-urdf.md) — link·joint, visual·collision, mass·inertia, 제작 revision
19. [PATCH-17: Humanoid 전장과 ROS 2 제어](PATCH-17-humanoid-electronics-and-control.md) — Raspberry Pi 5, MCU, `ros2_control`, watchdog
20. [PATCH-18: Humanoid Isaac Lab Sim2Real](PATCH-18-humanoid-isaac-lab-sim2real.md) — URDF→USD, PPO, domain randomization, 실물 승격

PATCH-00부터 PATCH-06은 앞 번호 patch의 완료 조건을 통과한 뒤 진행한다. PATCH-05는 AWS 자산 license 확인, Harmonic server load, 실제 collision 검증까지 통과해야 한다. PATCH-07은 PATCH-04까지만 완료하면 PATCH-05/06과 독립적으로 수행할 수 있다.
PATCH-08의 calibration capture·평가 흐름은 PATCH-04 완료 후 적용한다. 장애물 회피를 포함한 pose 이동 복구까지 연결하려면 PATCH-05/06도 먼저 완료한다.
PATCH-09의 기본 CI는 PATCH-00 이후 적용할 수 있다. Python/C++ test와 headless Gazebo 검증은 해당 구현 PATCH가 완료된 뒤 확장한다.
PATCH-10은 PATCH-02의 bag 기록 규약을 확장한다. DQN/회피 데이터는 PATCH-05/06 이후, calibration robustness 데이터는 PATCH-07 이후 수집한다. 실제 Yahboom system identification 절차는 PATCH-11의 interface 확인 뒤 완료한다.
PATCH-11~14는 Yahboom vendor Humble image와 firmware를 보존한 상태에서 진행한다. PATCH-12의 결정론적 회피보다 충돌이 많은 PATCH-13 policy는 PATCH-14 실물 주행으로 넘기지 않는다.
PATCH-15~18은 새 Humanoid의 장기 단계다. PATCH-15 요구사항과 single-joint rig, PATCH-16 CAD·URDF, PATCH-17 hardware safety를 통과한 뒤 PATCH-18 policy를 실물 actuator에 연결한다.

## 의도적으로 제외한 범위

- 원본 `waffle_pi`의 2D LDS는 수정하지 않는다. 복사한 `waffle_pi_3d`에서 기존 LiDAR 측정 설정만 3D `PointCloud2`용으로 교체한다.
- Calibration 중 동적 장애물을 사용하지 않는다. 움직이는 물체는 영상과 포인트클라우드 대응을 깨뜨린다.
- Nav2 전체 스택은 먼저 넣지 않는다. PATCH-06의 단순 회피가 통과한 뒤 목표점 주행이 필요할 때 추가한다.
- Yahboom 원본 source·PDF·3D model은 명시적 재배포 license를 확인하기 전 이 저장소에 복사하지 않는다.
- 첫 Humanoid에서 손·팔·계단·vision policy를 동시에 만들지 않는다. 12-DOF 후보의 stand·저속 평지 보행부터 검증한다.
- Berkeley Humanoid Lite·ToddlerBot·Open Duck Mini의 CAD를 자체 설계인 것처럼 복제하지 않는다. component별 license와 attribution을 확인한다.
