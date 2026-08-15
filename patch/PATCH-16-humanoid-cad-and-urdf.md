# PATCH-16: 자체 Humanoid CAD와 URDF 제작

- 작성일: 2026-08-15
- 선행 조건: PATCH-15 요구사항·single-joint rig 통과
- 대상: 향후 `humanoid/cad/`, `humanoid/description/`, `humanoid/tools/`
- 결론: **CAD assembly를 외형 그림이 아니라 link·joint·질량·충돌의 기준 원본으로 만든다. visual mesh, 단순 collision, mass·center of mass·inertia를 분리해 URDF/Xacro로 내보내고 RViz와 physics 검사까지 통과시킨다.**

## 개념

| 항목 | 의미 | 잘못되면 생기는 문제 |
|---|---|---|
| link | 움직이지 않는 하나의 강체 묶음 | 부품이 따로 떨어지거나 관절 수가 틀림 |
| joint | 두 link의 회전축·위치·범위 | 다리가 엉뚱한 축으로 움직임 |
| visual | 화면에 보이는 상세 형상 | physics보다 rendering 성능에 영향 |
| collision | 접촉 계산용 단순 형상 | 발이 바닥에 박히거나 simulation이 느림 |
| inertial | mass, center of mass, inertia tensor | robot이 떨거나 넘어지고 학습 결과가 틀어짐 |

[ROS 2 URDF physical property 문서](https://docs.ros.org/en/humble/Tutorials/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html)는 simulation link마다 inertial이 필요하며 zero에 가까운 inertia가 model 붕괴를 만들 수 있음을 설명한다.

## 1. 계획 디렉터리

```text
humanoid/
├── cad/
│   ├── README.md
│   ├── exports/step/
│   └── drawings/
├── description/
│   ├── urdf/mobin_humanoid.urdf.xacro
│   ├── meshes/visual/
│   ├── meshes/collision/
│   ├── config/joint_limits.yaml
│   └── launch/display.launch.py
├── tools/
│   ├── export_description.py
│   └── validate_description.py
└── tests/
    └── test_description.py
```

Onshape를 사용하면 [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot)의 export 흐름을 재사용한다. [Onshape Mates](https://cad.onshape.com/help/Content/Assembly/mates.htm)와 [Mate Connector](https://cad.onshape.com/help/Content/Assembly/assembly_mate_connector.htm)는 관절 자유도와 local axis를 정의하는 공식 참고 자료다. [Mass Properties](https://cad.onshape.com/help/Content/View/mass_properties_tool.htm)는 material density, center of mass, inertia 확인에 사용한다. 다른 CAD를 사용하면 STEP assembly를 보관하고 exporter만 바꾼다. 두 exporter를 동시에 유지하지 않는다.

## 2. Raspberry Pi 5를 CAD에 먼저 삽입한다

[Raspberry Pi 공식 hardware 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#schematics-and-mechanical-drawings)는 Pi 5의 mechanical drawing과 STEP 파일을 제공한다. 보드 외곽을 직접 다시 그리지 않고 공식 STEP을 CAD assembly의 reference component로 삽입한다.

[Pi 5 mechanical drawing](https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf)의 보드 크기는 약 `85 mm × 58 mm`다. 도면은 치수가 참고용이며 모든 부품과 제조 공차를 나타내지 않으므로 **최종 출력 전 실제 보드로 connector·높이·나사 간섭을 측정한다.**

| CAD envelope | 확인할 공간 |
|---|---|
| PCB와 mounting hole | standoff, M2.5급 fastener, 나사 머리, 절연 간격 |
| Active Cooler | heatsink·blower 높이, 흡기와 배기 통로, fan cable |
| USB·Ethernet·USB-C | connector 삽입 길이와 cable 굽힘 반경 |
| MIPI·PCIe FFC | connector 잠금 장치에 손이 닿고 ribbon이 꺾이지 않는 공간 |
| microSD | torso를 전부 분해하지 않고 교체할 방향 |
| GPIO·UART | MCU cable connector, strain relief, 탈착 공간 |
| power converter | Pi용 5 V rail, fuse, motor rail과의 물리적 분리 |
| 질량 | Pi, cooler, cable, converter를 torso mass·center of mass에 포함 |

Pi 5는 `5 V / 5 A` 전원 조건을 지원하며, 지속적인 추론 부하는 thermal throttling을 만들 수 있다. [Raspberry Pi 냉각 자료](https://www.raspberrypi.com/news/heating-and-cooling-raspberry-pi-5/)를 기준으로 첫 기체에도 Active Cooler와 airflow를 포함한다.

AI HAT을 쓸 가능성이 있으면 빈 PCB 높이만 남기지 않는다. [공식 AI HAT 조립 문서](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html)는 spacer, GPIO stacking header, PCIe ribbon, HAT 및 heatsink가 Pi 5 위에 쌓이는 구조를 보여 준다. Pi 5에는 외부용 single-lane PCIe connector 하나가 있으므로 AI HAT과 NVMe를 동시에 사용한다고 가정하지 않는다.

| compute 후보 | 기계 설계 결정 |
|---|---|
| Raspberry Pi 5 | 첫 prototype 기본값, 공식 STEP과 실제 보드로 검증 |
| Pi 5 + AI HAT+ | PATCH-18 vision benchmark가 필요성을 증명할 때 stack envelope 추가 |
| Compute Module 5 | torso가 Pi 5를 수용하지 못할 때만 carrier PCB와 함께 재설계 |
| Jetson Orin Nano | vision policy가 Pi 5 경로의 deadline을 넘을 때 power·cooling·torso를 다시 설계 |

[Compute Module 5 datasheet](https://pip.raspberrypi.com/categories/944/raspberry-pi-compute-module-5/documents/RP-008180-DS/cm5-datasheet.pdf?disposition=inline)는 module 크기를 `40 mm × 55 mm`로 정의한다. 더 작지만 carrier PCB가 필요하므로 첫 prototype에는 넣지 않는다.

**PATCH-18의 실제 policy benchmark가 끝나기 전 torso 내부 mounting plate를 최종 고정하지 않는다.** 첫 prototype은 Pi 5용 plate 하나만 만들고, accelerator가 필요하다는 측정 결과가 생기면 해당 board 기준으로 revision한다.

## 3. CAD assembly 규칙

| 규칙 | 이유 |
|---|---|
| 한 link 안의 움직이지 않는 부품은 rigid group | URDF 강체와 일치 |
| joint axis와 zero pose를 CAD mate로 명시 | axis를 URDF에서 다시 추측하지 않음 |
| 좌우 이름은 `left_*`, `right_*`로 고정 | mirror 부품과 controller mapping 구분 |
| 모든 치수는 mm, export 후 URDF는 m | 1000배 scale 오류 방지 |
| 재질·밀도를 실제 출력물과 fastener에 지정 | mass property 정확도 확보 |
| battery, Pi 5, MCU, cable 질량 포함 | torso center of mass 누락 방지 |
| bearing·shaft·insert 교체 공간 확보 | 조립 후 service 가능 |
| cable bend radius와 joint 최대각 동시 확인 | 움직임 중 단선 방지 |

좌우 mirror 형상도 joint axis 부호와 encoder 방향을 따로 검토한다. 형상이 대칭이어도 actuator 설치 방향은 같지 않을 수 있다.

## 4. Link·joint 표를 먼저 만든다

CAD export 전에 `humanoid/description/config/kinematic_tree.yaml`에 다음 정보를 둔다.

| field | 예시 |
|---|---|
| `parent`, `child` | `pelvis`, `left_hip_yaw_link` |
| `joint_name` | `left_hip_yaw_joint` |
| `axis` | `[0, 0, 1]` |
| `origin_xyz` | meter |
| `origin_rpy` | radian |
| `lower`, `upper` | radian |
| `velocity` | rad/s |
| `effort` | N·m |
| `encoder_sign`, `zero_offset` | 실물 mapping |

이 표, CAD mate, URDF의 joint 이름·axis·limit가 자동 검사에서 일치해야 한다.

## 5. Visual과 collision을 분리한다

| 출력 | 형상 | 목적 |
|---|---|---|
| visual | decimate한 실제 외형 mesh | RViz·rendering |
| collision | box, capsule, cylinder, convex hull | 빠르고 안정적인 contact |

발바닥 collision은 평평하고 좌우 크기가 실제 sole과 맞아야 한다. 관절 주변 collision은 정상 range에서 이웃 link를 막지 않도록 확인한다. 상세 나사산·케이블·logo는 collision에 넣지 않는다.

visual mesh를 그대로 collision에 쓰는 option은 초기 확인에만 허용한다. 최종 asset은 단순 collision을 별도 저장한다.

## 6. Mass와 inertia를 CAD에서 가져온다

각 link별로 다음 표를 export한다.

| 값 | 단위·기준 |
|---|---|
| mass | kg |
| center of mass | link frame 기준 m |
| `ixx`, `iyy`, `izz` | kg·m² |
| `ixy`, `ixz`, `iyz` | kg·m² |
| source revision | CAD version 또는 document ID |

조건:

- inertia tensor는 center of mass 기준
- 모든 diagonal 값은 양수
- 전체 link mass 합과 조립품 실측 질량 차이를 기록
- battery·cable처럼 CAD가 놓치기 쉬운 질량은 실측 보정 근거를 남김
- 임의의 `0.001` 값을 모든 link에 복사하지 않음

시제품 조립 뒤 저울과 balance test로 mass·center of mass를 다시 확인하고 CAD material을 갱신한다.

## 7. URDF/Xacro를 생성한다

`mobin_humanoid.urdf.xacro`에는 다음을 포함한다.

| 요소 | 내용 |
|---|---|
| link | visual, collision, inertial |
| joint | parent, child, origin, axis, limit, dynamics |
| frames | `base_link`, `imu_link`, foot frames, camera frame 후보 |
| transmission/control | PATCH-17에서 사용할 joint interface 자리 |

generated URDF를 직접 수정하지 않는다. 변경은 CAD, export config, Xacro source 중 원인이 있는 곳에 반영한 뒤 다시 생성한다.

## 8. 정적 검사

```bash
xacro humanoid/description/urdf/mobin_humanoid.urdf.xacro \
  > /tmp/mobin_humanoid.urdf
check_urdf /tmp/mobin_humanoid.urdf
```

검사 script는 최소 다음을 실패 처리한다.

- link마다 유효한 mass·inertia가 없음
- joint axis 길이가 0 또는 정규화되지 않음
- limit의 lower가 upper보다 큼
- mesh 파일 누락 또는 scale 오류
- link·joint 이름 중복
- kinematic tree root가 하나가 아님
- 좌우 joint 목록 또는 limit가 의도 없이 다름

## 9. RViz와 운동 범위 검사

```bash
ros2 launch mobin_humanoid_description display.launch.py
```

RViz에서 joint state publisher slider를 최소·중립·최대로 움직인다.

| 검사 | 통과 조건 |
|---|---|
| zero pose | 발바닥이 같은 높이·방향 |
| joint direction | 이름과 실제 회전 방향 일치 |
| mesh origin | 회전할 때 link가 joint 축에서 분리되지 않음 |
| range | cable·housing 간섭 전 limit |
| TF | torso, IMU, foot frame 방향 일관 |

RViz는 dynamics를 검증하지 않는다. 여기서 보인다는 이유만으로 Isaac Lab asset이 완성된 것은 아니다.

## 10. 제작 도면과 revision

| 산출물 | 저장 내용 |
|---|---|
| STEP | 전체 assembly와 교환 가능한 중립 형식 |
| STL/3MF | 실제 출력 orientation·revision |
| drawing | shaft, bearing, insert, fastener 중요 치수 |
| BOM | part number, 수량, 질량, 공급처 |
| assembly guide | 조립 순서, torque, cable route |
| change log | CAD revision과 URDF/USD 호환 여부 |

직접 만든 CAD의 license는 공개 전에 명시한다. 외부 STEP·mesh는 원본 license가 허용하는 경우만 배포한다.

## 11. 완료 조건

- CAD rigid group·joint mate와 URDF link·joint가 1:1 대응
- visual과 단순 collision mesh 분리
- 모든 simulated link에 CAD 근거 mass·center of mass·inertia 존재
- `xacro`, `check_urdf`, 자동 정적 검사 통과
- RViz zero pose·joint 방향·limit·TF 확인
- 실제 출력물 mass와 CAD 합계 차이 기록 및 허용 범위 결정
- STEP, print file, drawing, BOM, assembly revision 연결
- 출처와 license가 불명확한 mesh 없음
- Pi 5 STEP, Active Cooler, connector cable, power converter를 포함한 torso 간섭 검사 통과
- 실제 Pi 5로 mounting hole·connector·microSD·airflow 접근성 확인
- PATCH-18 benchmark에서 선택한 compute board와 CAD revision 일치

**PATCH-16의 URDF가 PATCH-17 hardware interface와 PATCH-18 Isaac Lab USD의 공통 기준이다.**
