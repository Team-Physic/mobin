# Git Fork WorkFlow 및 License

- 작성일: 2026-08-07
- 대상: TurtleBot3 simulation, LiDAR-Camera calibration source, AWS warehouse asset
- 확인 기준: local fork clone, upstream repository, license 원문
- 결론: **세 source를 독립 fork로 관리하면 직접 수정·push할 수 있다. 단, 재배포 시 각 upstream과 nested dependency의 license 고지를 유지해야 한다.**

## Repository 구성

| 로컬 경로 | `origin` : 내 fork를 가리키는 remote | `upstream` : 원본 프로젝트를 가리키는 remote | 기준 branch |
|---|---|---|---|
| `forks/turtlebot3_simulations` | `JungSeong/turtlebot3_simulations` | `ROBOTIS-GIT/turtlebot3_simulations` | `jazzy` |
| `forks/direct_visual_lidar_calibration` | `JungSeong/direct_visual_lidar_calibration` | `koide3/direct_visual_lidar_calibration` | `main` |
| `forks/aws-robomaker-small-warehouse-world` | 내 GitHub fork | `aws-robotics/aws-robomaker-small-warehouse-world` | `ros2`의 고정 commit |

## 시작하기

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo
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

Calibration fork는 Sophus, json, nanoflann을 nested submodule로 사용하므로 `--recursive`가 필요하다. 상위 실습 repository 자체에는 submodule을 사용하지 않는다.

확인:

```bash
git -C forks/turtlebot3_simulations remote -v
git -C forks/turtlebot3_simulations status --short --branch

git -C forks/direct_visual_lidar_calibration remote -v
git -C forks/direct_visual_lidar_calibration status --short --branch
git -C forks/direct_visual_lidar_calibration submodule status --recursive

git -C forks/aws-robomaker-small-warehouse-world remote -v
git -C forks/aws-robomaker-small-warehouse-world rev-parse HEAD
```

## 실습 branch에서 수정 및 commit, push

Upstream 기준 branch에 직접 commit하지 않고 patch별 실습 branch를 만든다.

```bash
git -C forks/turtlebot3_simulations switch jazzy
git -C forks/turtlebot3_simulations switch -c practice/replace-lidar-with-3d

git -C forks/direct_visual_lidar_calibration switch main
git -C forks/direct_visual_lidar_calibration switch -c practice/calibration-experiment

git -C forks/aws-robomaker-small-warehouse-world \
  switch --detach ee0af733315e78432408c3cd98d378ecee5f767c
git -C forks/aws-robomaker-small-warehouse-world \
  switch -c practice/gazebo-harmonic
```

수정 후 각 fork에 따로 저장한다.

```bash
git -C forks/turtlebot3_simulations status --short
git -C forks/turtlebot3_simulations add <수정한-파일>
git -C forks/turtlebot3_simulations commit -m "feat: add calibration lidar"
git -C forks/turtlebot3_simulations push -u origin practice/replace-lidar-with-3d
```

## Upstream 변경 가져오기

작업 branch가 clean한 상태에서 수행한다.

```bash
git -C forks/turtlebot3_simulations fetch upstream
git -C forks/turtlebot3_simulations switch jazzy
git -C forks/turtlebot3_simulations merge --ff-only upstream/jazzy
git -C forks/turtlebot3_simulations push origin jazzy

git -C forks/direct_visual_lidar_calibration fetch upstream
git -C forks/direct_visual_lidar_calibration switch main
git -C forks/direct_visual_lidar_calibration merge --ff-only upstream/main
git -C forks/direct_visual_lidar_calibration push origin main
```

`--ff-only`가 실패하면 fork와 upstream history가 갈라진 상태다. 자동 merge하지 말고 commit 차이를 확인한 뒤 별도 branch에서 해결한다.

## License

### 확인된 license

| 대상 | 확인된 license | 수정·재배포 시 확인할 내용 |
|---|---|---|
| [ROBOTIS TurtleBot3 simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) | Apache-2.0 | LICENSE 제공, 기존 copyright·patent·trademark·attribution notice 유지, 수정 파일에 변경 사실 표시 |
| [direct_visual_lidar_calibration README](https://github.com/koide3/direct_visual_lidar_calibration/blob/main/README.md)와 [package.xml](https://github.com/koide3/direct_visual_lidar_calibration/blob/main/package.xml) | MIT 선언 | 기존 copyright와 permission notice를 source 또는 배포물에 유지 |
| [AWS Small Warehouse ROS 2 고정 commit](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ee0af733315e78432408c3cd98d378ecee5f767c) | 루트 `LICENSE`와 `package.xml` 모두 `MIT-0` | 두 파일을 보존하고 원본 commit·변경 사실 기록. `gazebo_ros` 기반 Gazebo Classic launch는 실행하지 않음 |
| Sophus | MIT | `thirdparty/Sophus/LICENSE.txt` 유지 |
| nlohmann/json | MIT 중심, bundled component별 추가 고지 가능 | 해당 checkout의 license·third-party notice 확인 |
| nanoflann | BSD | `thirdparty/nanoflann/COPYING`의 copyright·조건·면책문 유지 |

[Apache-2.0 원문](https://www.apache.org/licenses/LICENSE-2.0)은 재배포 시 license 사본 제공, 수정 파일 표시, 기존 notice 유지, upstream에 NOTICE가 있으면 해당 attribution 유지 조건을 명시한다. 또한 Apache-2.0은 upstream 상표 사용 권한을 일반적으로 부여하지 않는다.

[MIT license 원문](https://opensource.org/license/mit)은 software의 전체 또는 substantial portion을 배포할 때 copyright notice와 permission notice를 포함하도록 요구한다.

현재 확인한 calibration 기준 commit은 README와 `package.xml`에 MIT를 선언하지만 repository root에 독립된 `LICENSE` 파일이 없다. 외부 배포 전에는 upstream이 제공한 정확한 copyright·permission notice를 확인해야 하며, 저작권자 표기를 추측해 새로 만들지 않는다.

AWS warehouse ROS 2 고정 commit의 루트 `LICENSE`는 MIT-0 본문이고 `package.xml`도 `MIT-0`으로 선언한다. 모델 하위에는 별도 `LICENSE`, `COPYING`, `NOTICE`가 없고 `model.config`의 author 값은 비어 있다. MIT-0은 attribution을 조건으로 요구하지 않지만, 이 fork는 출처와 변경 이력을 추적하도록 루트 `LICENSE`와 원본 commit을 보존한다. fork의 루트 `LICENSE`와 `package.xml`을 모두 유지하고, 공개·상업 배포 전에는 별도 license 검토를 거친다. Harmonic 이식 범위와 검증 명령은 [PATCH-05](../patch/PATCH-05-obstacle-scenarios.md)에 기록한다.

### Fork 실습 checklist

- [ ] upstream의 `LICENSE`, `COPYING`, `NOTICE` 파일을 삭제하거나 덮어쓰지 않는다.
- [ ] Apache-2.0 파일을 수정해 배포할 때 해당 파일을 변경했다는 사실을 눈에 띄게 남긴다.
- [ ] README에 fork이며 비공식 실습용임을 표시하고 upstream 공식 제품으로 오인시키지 않는다.
- [ ] binary, Docker image, archive를 배포할 때도 필요한 license·notice를 함께 제공한다.
- [ ] nested submodule을 업데이트하면 새 commit의 license를 다시 확인한다.
- [ ] warehouse world·mesh·texture를 복사하거나 수정하면 source URL, commit SHA, 변경 사실을 함께 기록한다.
- [ ] upstream의 license metadata를 임의로 변경하거나 재라이선스하지 않는다.
- [ ] 새 dependency를 추가하기 전에 license 호환성과 배포 의무를 확인한다.
- [ ] 논문·알고리즘 인용 요구는 software license와 별도로 확인한다.
