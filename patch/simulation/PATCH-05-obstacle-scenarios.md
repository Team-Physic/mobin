# Simulation PATCH-05: AWS Warehouse ROS 2 branch를 Gazebo Harmonic으로 이식

- 작성 기준: 2026-08-14
- 원본: [`aws-robotics/aws-robomaker-small-warehouse-world`](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)
- 원본 branch: `ros2`
- 고정 commit: `ee0af733315e78432408c3cd98d378ecee5f767c`
- 대상 runtime: ROS 2 Jazzy, Gazebo Harmonic의 Gazebo Sim 8

## 결론

**원본 warehouse의 SDF, DAE mesh, texture, map을 수정해 사용하는 것은 원본 `LICENSE`가 허용한다.**

다만 다음 조건을 지킨다.

| 조건 | 이 프로젝트에서 지키는 방법 |
|---|---|
| MIT-0 원문과 provenance 보존 | AWS fork의 루트 `LICENSE`, 원본 commit, Harmonic 변경 이력을 유지한다. |
| 원본과 변경본 구분 | 원본 commit과 Harmonic 변경 내용을 README 및 commit에 기록한다. |
| package metadata 보존 | 루트 `LICENSE`와 `package.xml`의 MIT-0 선언을 다른 license로 바꾸지 않는다. |
| 오래된 dependency 실행 금지 | ROS 2 Python launch가 호출하는 `gazebo_ros`와 Gazebo Classic을 실행하지 않는다. |
| 자산 출처 고정 | branch 이름만 쓰지 않고 commit SHA를 함께 기록한다. |

**보관 저장소라는 사실은 기존 open-source license를 취소하지 않는다.**
그러나 upstream은 2026-07-21 archived 상태이며 production 사용을 권장하지 않는다.
이 PATCH는 실습용 이식이다.

이 내용은 저장소 파일에 근거한 준수 방법이며 법률 자문은 아니다. 외부 공개·상업 배포 전에는 조직의 license 검토를 거친다.

## 확인한 라이선스 근거

고정 commit의 [루트 `LICENSE`](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/blob/ee0af733315e78432408c3cd98d378ecee5f767c/LICENSE)에는 다음 권한이 있다.

```text
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software
```

MIT-0은 저작자 표시를 재사용 조건으로 요구하지 않는다.
이 프로젝트에서는 출처와 변경 이력을 추적하도록 루트 `LICENSE` 원문도 보존한다.

고정 commit에서 확인한 결과:

| 검사 대상 | 결과 |
|---|---|
| repository-level license | 루트 `LICENSE` 한 개, MIT-0 본문 |
| `package.xml` | `<license>MIT-0</license>` |
| 모델별 LICENSE/COPYING/NOTICE | 없음 |
| `model.config` author | 모두 빈 값 |
| DAE·texture·PSD 자산 수 | 60개 |

루트 `LICENSE`는 MIT-0 본문이고 `package.xml`도 MIT-0으로 선언한다.
두 파일의 license 표기가 일치하므로 별도 재라이선스나 metadata 통합은 필요하지 않다.
**따라서 원본 `LICENSE`, 고정 commit, 수정 내역을 함께 유지한다.**

재확인 명령:

```bash
git -C forks/aws-robomaker-small-warehouse-world rev-parse HEAD
git -C forks/aws-robomaker-small-warehouse-world show HEAD:LICENSE
git -C forks/aws-robomaker-small-warehouse-world \
  show HEAD:package.xml | grep -n '<license>'
git -C forks/aws-robomaker-small-warehouse-world ls-files |
  grep -Ei '(^|/)(LICENSE|COPYING|NOTICE|AUTHORS|CREDITS)(\.|$)'
```

첫 명령은 반드시 다음 값을 출력해야 한다.

```text
ee0af733315e78432408c3cd98d378ecee5f767c
```

## 왜 이 world를 선택하는가

2026-08-14 기준 491 stars이며 local mesh·texture·map과 선반·pallet jack·clutter를 포함한다.
별점보다 **Fuel 없이 재현 가능한 고정 commit과 warehouse 구조**가 선택 이유다.
575 stars인 Open-RMF demos는 standalone warehouse 자산이 목적이 아니며, ROS 2 stack 전체를 가져와 범위가 커지므로 제외했다.

## 개념

**공식 `ros2` branch도 Gazebo Classic용이다.** `ament_cmake`와 Python launch는 ROS 2 형식이지만, launch가 `gazebo_ros`의 `gzserver`·`gzclient`를 실행한다. 이 PATCH는 ROS 2 package의 SDF·mesh·texture를 보존하면서 실행 계층만 `ros_gz_sim`과 Gazebo Harmonic으로 바꾼다.
`GZ_SIM_RESOURCE_PATH`는 `model://이름`의 검색 경로이며, Harmonic physics engine은 collision과 로봇 운동을 계산한다.

## 성공 조건

1. 고정 commit에서 만든 별도 practice branch가 존재한다.
2. 원본 `LICENSE`와 `package.xml`이 보존된다.
3. Harmonic server가 world를 `Error Code 9`, `14`, `19` 없이 연다.
4. Bullet Featherstone이 physics engine으로 로드된다.
5. `model://` URI가 모두 local fork에서 해결된다.
6. Gazebo GUI에 바닥·벽·선반·pallet jack·clutter가 보인다.
7. TurtleBot3가 생성되고 `/cmd_vel`에 따라 움직인다.
8. 선반 collision 앞에서 로봇이 통과하지 못한다.

## 1. 내 GitHub 계정에서 fork한다

원본 페이지에서 Fork를 누른 뒤 내 계정에 저장소를 만든다.
아래 명령은 확인된 `JungSeong` fork의 `ros2` branch를 clone한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin

git clone --branch ros2 \
  https://github.com/JungSeong/aws-robomaker-small-warehouse-world.git \
  forks/aws-robomaker-small-warehouse-world

git -C forks/aws-robomaker-small-warehouse-world remote add upstream \
  https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
```

고정 commit에서 실습 branch를 만든다.

```bash
git -C forks/aws-robomaker-small-warehouse-world fetch upstream ros2
git -C forks/aws-robomaker-small-warehouse-world \
  switch --detach ee0af733315e78432408c3cd98d378ecee5f767c
git -C forks/aws-robomaker-small-warehouse-world \
  switch -c practice/gazebo-harmonic
```

`detached HEAD`에서 바로 수정하지 않는다.
마지막 명령이 수정 내용을 저장할 이름 있는 branch를 만든다.

확인:

```bash
git -C forks/aws-robomaker-small-warehouse-world status --short --branch
git -C forks/aws-robomaker-small-warehouse-world rev-parse HEAD
git -C forks/aws-robomaker-small-warehouse-world remote -v
```

## 2. 원본 world를 복사한다

원본 Gazebo Classic world는 비교용으로 남긴다.

```bash
cp \
  forks/aws-robomaker-small-warehouse-world/worlds/small_warehouse/small_warehouse.world \
  forks/aws-robomaker-small-warehouse-world/worlds/small_warehouse/small_warehouse_harmonic.world
```

`small_warehouse_harmonic.world`의 XML 선언 다음에 provenance를 넣는다.

```xml
<!--
  Derived from aws-robotics/aws-robomaker-small-warehouse-world
  commit: ee0af733315e78432408c3cd98d378ecee5f767c
  license: ../../LICENSE
  modified: Gazebo Harmonic systems and Bullet Featherstone selection
-->
```

**원본 commit, license 위치, 변경 목적을 새 파일 안에서도 확인할 수 있게 한다.**

## 3. Harmonic system을 world에 명시한다

원본 world에는 Gazebo Sim system plugin이 없다.
그대로 실행하면 host별 기본 `server.config`에 의존한다.

`small_warehouse_harmonic.world`의 `<world name="default">` 바로 다음에 추가한다.

```xml
<plugin
  filename="gz-sim-physics-system"
  name="gz::sim::systems::Physics">
  <engine>
    <filename>gz-physics-bullet-featherstone-plugin</filename>
  </engine>
</plugin>
<plugin
  filename="gz-sim-user-commands-system"
  name="gz::sim::systems::UserCommands"/>
<plugin
  filename="gz-sim-scene-broadcaster-system"
  name="gz::sim::systems::SceneBroadcaster"/>
<plugin
  filename="gz-sim-sensors-system"
  name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

각 system의 역할:

| system | 역할 |
|---|---|
| Physics | collision과 로봇 운동 계산 |
| UserCommands | TurtleBot3 create·remove·pose 명령 처리 |
| SceneBroadcaster | server의 scene을 Gazebo GUI에 전달 |
| Sensors | Camera와 LiDAR rendering·측정 생성 |

### Bullet Featherstone을 선택한 이유

Harmonic 8.11의 기본 DART로 원본을 실행하면 다음 로그가 반복됐다.

```text
Mesh construction from an SDF has not been implemented yet for dartsim.
The geometry element of collision [...] couldn't be created
```

warehouse의 collision이 DAE mesh이므로 이 상태에서는 선반이 보여도 로봇이 통과할 수 있다.
설치 이미지에는 Bullet Featherstone plugin이 있고, 같은 world를 이 engine으로 실행했을 때 mesh collision 생성 실패 로그가 사라졌다. [Gazebo Sim 8 문서](https://gazebosim.org/api/sim/8/physics.html)는 Bullet 계열 지원을 preliminary로 설명하므로 실제 접촉과 wheel dynamics 검증이 필수다.

**단순히 GUI에 모델이 보이는 것은 성공이 아니다. 실제 접촉 테스트까지 통과해야 한다.**

## 4. Ground와 Roof를 정적 모델로 고친다

원본을 Harmonic에서 처음 열면 다음 오류가 난다.

```text
Error Code 19: A link named link has invalid inertia.
Error Code 9: Failed to load a world.
```

문제가 확인된 파일:

| 파일 | 이유 |
|---|---|
| `models/aws_robomaker_warehouse_GroundB_01/model.sdf` | 오래된 inertia가 최신 유효성 검사를 통과하지 못함 |
| `models/aws_robomaker_warehouse_RoofB_01/model.sdf` | 오래된 inertia가 최신 유효성 검사를 통과하지 못함 |

바닥과 지붕은 움직일 물체가 아니다.
두 파일 각각의 `<model ...>` 바로 다음에 추가한다.

```xml
<static>true</static>
```

두 파일에서 전체 `<inertial>...</inertial>` 블록을 삭제한다.

수정 후 구조:

```xml
<model name="aws_robomaker_warehouse_GroundB_01">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      ...
    </collision>
    <visual name="visual">
      ...
    </visual>
  </link>
</model>
```

Roof도 같은 형태다.
**임의의 inertia 숫자를 새로 만들지 않는다. 정적 모델에는 동역학 inertia가 필요 없다.**

이 두 수정은 원본 `ros2` branch가 아니라 `practice/gazebo-harmonic` branch에만 저장한다.

## 5. Docker에서 외부 자산을 read-only로 마운트한다

`docker/compose.yaml`의 공통 environment에 추가한다.

```yaml
TURTLEBOT3_WORLD_DIR: ${TURTLEBOT3_WORLD_DIR:-/ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds}
GZ_SIM_RESOURCE_PATH: /opt/aws_warehouse/models
```

공통 volumes에 추가한다.

```yaml
- ../../forks/aws-robomaker-small-warehouse-world:/opt/aws_warehouse:ro
```

`:ro`는 container가 source와 license를 바꾸지 못하게 한다.
`GZ_SIM_RESOURCE_PATH` 덕분에 world의 다음 URI가 local model로 해석된다.

```xml
<uri>model://aws_robomaker_warehouse_ShelfE_01</uri>
```

Fuel 다운로드는 필요 없다.

## 6. 기존 launch가 world directory를 받게 한다

`turtlebot3_world.launch.py`는 현재 world 이름을 무조건 TurtleBot3 package의 `worlds/` 아래에서 찾는다.
파일 이름과 디렉터리를 분리해 받는다.

import에 `FindPackageShare`와 `PathJoinSubstitution`은 이미 있으므로 새 dependency는 없다.

```python
world_dir = LaunchConfiguration('world_dir')
world_name = LaunchConfiguration('world')
world = PathJoinSubstitution([world_dir, world_name])
```

`LaunchDescription`에 다음 인자를 추가한다.

```python
ld.add_action(DeclareLaunchArgument(
    'world_dir',
    default_value=PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'worlds',
    ]),
    description='Directory containing the selected world file',
))
```

기존 default world 동작은 바뀌지 않는다.

`docker/compose.yaml`의 sim 명령에는 `world_dir`를 전달한다.

```yaml
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
world_dir:=$${TURTLEBOT3_WORLD_DIR}
world:=$${TURTLEBOT3_WORLD}
gazebo_gui:=$${GAZEBO_GUI}
launch_rviz:=$${LAUNCH_RVIZ}
```

실제 YAML에서는 기존 folded command 한 줄 안에 이어 쓴다.

## 7. TurtleBot3 package를 다시 빌드한다

AWS fork는 `/opt/aws_warehouse`에 mount되므로 colcon build 대상이 아니다.
launch를 수정한 TurtleBot3 package만 다시 빌드한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

docker compose run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  cd /ws &&
  colcon build --symlink-install --packages-select turtlebot3_gazebo
'
```

## 8. Harmonic server만 먼저 검증한다

GUI와 ROS bridge를 붙이기 전에 world 자체 오류를 분리한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

docker compose run --rm shell bash -lc '
  timeout --signal=INT 15 \
    gz sim -s -r -v4 \
    /opt/aws_warehouse/worlds/small_warehouse/small_warehouse_harmonic.world
'
```

성공 로그에 다음 내용이 있어야 한다.

```text
Loaded [gz::physics::bullet_featherstone::Plugin]
Loaded system [gz::sim::systems::Physics]
Loaded system [gz::sim::systems::Sensors]
World [default] initialized
```

다음 내용은 없어야 한다.

```text
Unable to find uri[model://...]
A link named link has invalid inertia
couldn't be created
Failed to load a world
```

`gz sdf -k`만으로 끝내지 않는다.
bare `gz sdf`는 `model://` callback을 갖지 않아 유효한 resource path도 찾지 못할 수 있다.
실제 `gz sim -s`가 최종 검사다.

## 9. 전체 simulation을 실행한다

host terminal:

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

export TURTLEBOT3_WORLD_DIR=/opt/aws_warehouse/worlds/small_warehouse
export TURTLEBOT3_WORLD=small_warehouse_harmonic.world
export GAZEBO_GUI=true
export LAUNCH_RVIZ=true

docker compose up sim
```

다른 terminal의 개발 shell:

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose run --rm shell
```

container 안:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ros2 topic list | grep -E '(^/scan$|^/camera/|^/odom$|^/cmd_vel$)'
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
```

Simulation PATCH-01의 `waffle_pi_3d`를 완료했다면 `/calib/points`도 확인한다.

```bash
ros2 topic hz /calib/points
```

## 10. collision을 실제로 확인한다

warehouse가 보이는 것과 collision이 동작하는 것은 다르다.
로봇을 선반 앞에 놓은 상태로 낮은 속도를 발행한다.

```bash
ros2 topic pub --rate 10 --times 50 \
  --wait-matching-subscriptions 1 \
  /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

확인 항목:

| 관측 | 성공 기준 |
|---|---|
| Gazebo GUI | 로봇이 선반 표면을 관통하지 않음 |
| `/scan` 또는 `/calib/points` | 선반까지 거리가 줄어듦 |
| `/odom` | 접촉 후 전진 거리가 계속 증가하지 않음 |
| server log | mesh collision 생성 실패가 없음 |

즉시 정지:

```bash
ros2 topic pub --once \
  /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

Bullet Featherstone에서 TurtleBot3 wheel dynamics가 불안정하다면 engine만 숨겨 바꾸지 않는다.
그 경우 warehouse collision mesh를 단순 box collision으로 교체하고 기본 DART로 되돌리는 별도 변경이 필요하다.

## 11. 실습 시나리오를 배치한다

한 warehouse에서 시작 pose와 동적 cart 경로만 바꾼다.
world 파일을 시나리오마다 복제하지 않는다.

| scenario | 배치 | 검증 목적 |
|---|---|---|
| `static_shelf` | 중앙 통로에서 선반을 향해 주행 | LiDAR 감지와 기본 회피 |
| `blind_crossing` | 선반 끝에서 cart가 횡단 | 가려졌다 나타나는 동적 물체 대응 |
| `occlusion` | cart가 선반 뒤로 들어갔다 재등장 | Camera-LiDAR 인식·추적 연속성 |

Simulation PATCH-06의 단순 `/scan` 회피는 `static_shelf` 기준선으로 유지한다.
`blind_crossing`과 `occlusion`은 이후 perception·tracking 평가에 사용한다.

동적 cart는 기존 `obstacle1` system을 재사용한다.
wall-clock 대신 `UpdateInfo.simTime`을 사용하고, speed와 waypoint를 SDF에서 읽게 바꾼다.
같은 simulation time에 cart 위치가 같아야 한다.

## 12. 변경을 각 fork에 저장한다

AWS fork:

```bash
git -C forks/aws-robomaker-small-warehouse-world status --short
git -C forks/aws-robomaker-small-warehouse-world add \
  worlds/small_warehouse/small_warehouse_harmonic.world \
  models/aws_robomaker_warehouse_GroundB_01/model.sdf \
  models/aws_robomaker_warehouse_RoofB_01/model.sdf
git -C forks/aws-robomaker-small-warehouse-world commit \
  -m "feat: migrate warehouse assets to Gazebo Harmonic"
git -C forks/aws-robomaker-small-warehouse-world push \
  -u origin practice/gazebo-harmonic
```

TurtleBot3 fork:

```bash
git -C forks/turtlebot3_simulations add \
  turtlebot3_gazebo/launch/turtlebot3_world.launch.py
git -C forks/turtlebot3_simulations commit \
  -m "feat: load worlds from an external directory"
git -C forks/turtlebot3_simulations push
```

상위 Mobin repository에는 compose와 문서를 저장한다.
`forks/`는 독립 Git 저장소이므로 상위 commit에 nested source가 섞이지 않는다.

## 실패할 때 확인 순서

| 증상 | 원인 | 확인 |
|---|---|---|
| `Unable to find uri[model://...]` | model directory가 resource path에 없음 | `echo "$GZ_SIM_RESOURCE_PATH"` |
| `invalid inertia` | Ground/Roof 수정 누락 | 두 model의 `static`과 `inertial` 확인 |
| 모델은 보이나 통과함 | DART가 mesh collision을 만들지 못함 | Bullet load 로그와 collision 실패 로그 확인 |
| GUI만 안 보임 | X11·DISPLAY 문제 | Simulation PATCH-00의 GUI 진단 수행 |
| Robot create가 대기함 | world server load 실패 | 가장 먼저 나온 Error Code 확인 |
| texture만 누락됨 | DAE의 상대 texture 경로 또는 mount 누락 | model 전체 디렉터리가 유지됐는지 확인 |

container에 `rg`가 없으므로 위 검사는 `grep`을 사용한다.

**최소 이식 범위는 원본 자산 보존, 두 정적 모델 수정, 네 Harmonic system 추가, Bullet collision 검증이다.**
