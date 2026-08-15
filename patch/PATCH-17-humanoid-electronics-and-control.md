# PATCH-17: Humanoid 전장과 ROS 2 제어 계층

- 작성일: 2026-08-15
- 선행 조건: PATCH-15 actuator rig, PATCH-16 URDF
- 대상: 향후 `humanoid/hardware/`, `humanoid/control/`, `humanoid/firmware/`
- 결론: **Raspberry Pi 5는 ROS 2·state estimation·policy·logging을 담당하고, MCU 또는 smart-servo bus controller는 정해진 주기의 actuator I/O·watchdog·limit를 담당한다. ROS 2와 실물 사이에는 하나의 `ros2_control` hardware interface만 둔다.**

## 개념

Linux가 실행되는 Raspberry Pi 5는 학습 policy와 ROS 2에는 적합하지만 motor의 전류·position loop를 항상 정확한 주기로 실행하는 안전 controller로 간주하지 않는다.

| 계층 | 담당 |
|---|---|
| Raspberry Pi 5 | ROS 2 node, IMU/state estimation, policy inference, rosbag, UI |
| MCU/servo controller | actuator command, encoder read, hard limit, watchdog |
| power/safety circuit | fuse, main switch, emergency stop, voltage·current 보호 |

Arduino-compatible MCU는 선택지다. 사용하는 smart actuator가 안정적인 bus SDK와 controller를 제공하면 불필요한 Arduino layer를 추가하지 않는다.

## 1. Hardware architecture를 확정한다

```text
policy / trajectory
        ↓
ros2_control controller_manager
        ↓
MobinHumanoidSystem::write()
        ↓ serial/CAN/servo bus
MCU or bus controller → actuators
        ↑ encoder/current/temp/status
MobinHumanoidSystem::read()
        ↓
/joint_states + diagnostics
```

[ros2_control Jazzy 문서](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)는 controller manager가 hardware `read()`→controller update→hardware `write()` 순서로 실행하는 구조를 정의한다. 다관절 humanoid는 하나의 통신 채널과 여러 joint를 가진 `System` component가 적합하다.

## 2. 전장 설계 자료

`humanoid/hardware/`에 다음 문서를 만든다.

| 파일 | 내용 |
|---|---|
| `power_budget.md` | rail별 평균·peak current, regulator 여유, fuse |
| `wiring.md` | connector, wire gauge, pin, cable length, ground |
| `bom.csv` | 제조사 part number와 대체 가능 여부 |
| `interfaces.md` | bus, baud rate, packet, timeout, update rate |
| `safety.md` | emergency stop이 실제 전력을 끊는 범위 |

전원 계산은 actuator stall current를 모두 단순 합산한 값과 정상 보행 측정값을 구분한다. Pi 5 전원 rail과 motor rail을 분리하고 ground·EMI·brownout 대책을 회로 review에 포함한다.

## 3. MCU protocol을 작게 유지한다

최소 packet:

| 방향 | 값 |
|---|---|
| Pi→MCU | sequence, monotonic timestamp, joint target, control mode, enable |
| MCU→Pi | sequence, joint position·velocity·current·temperature, fault, battery |

규칙:

- CRC 또는 transport 자체 오류 검출 사용
- sequence 누락과 오래된 command 거부
- command timeout이면 torque off 또는 정의한 safe pose 동작
- joint hard limit와 current·temperature limit는 MCU 쪽에서도 적용
- enable은 power-on 기본 false

메시지 schema가 고정되기 전 micro-ROS와 custom serial을 둘 다 구현하지 않는다. 필요한 topic이 적고 지연 측정이 쉬우면 custom binary protocol, ROS message 통합이 더 중요하면 micro-ROS 중 하나를 선택한다.

[micro-ROS Arduino](https://github.com/micro-ROS/micro_ros_arduino)는 ESP32를 지원하고 Apache-2.0이지만, 공식 README도 production·safety use에 준비되지 않았다고 밝힌다. 사용하더라도 emergency stop과 watchdog의 유일한 안전 계층으로 삼지 않는다.

## 4. URDF에 ros2_control interface를 추가한다

PATCH-16 Xacro에서 simulation과 real hardware plugin만 parameter로 바꾼다.

```xml
<ros2_control name="MobinHumanoidSystem" type="system">
  <hardware>
    <plugin>mobin_humanoid_hardware/MobinHumanoidSystem</plugin>
  </hardware>
  <joint name="left_knee_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

실제 actuator가 제공하지 않는 effort를 추정값인데 측정값처럼 노출하지 않는다. 없다면 interface를 빼거나 `estimated_effort` diagnostic으로 구분한다.

## 5. C++ hardware interface를 구현한다

계획 구조:

```text
humanoid/control/mobin_humanoid_hardware/
├── include/mobin_humanoid_hardware/mobin_humanoid_system.hpp
├── src/mobin_humanoid_system.cpp
├── config/controllers.yaml
├── plugin_description.xml
└── test/test_packet_codec.cpp
```

행동을 바꾸는 핵심 함수:

| 함수 | 역할 |
|---|---|
| `on_init()` | URDF joint와 protocol joint mapping 검증 |
| `on_configure()` | port 연결, actuator 상태 읽기 |
| `on_activate()` | 현재 자세를 command 초기값으로 사용해 jump 방지 |
| `read()` | encoder·fault를 state interface에 반영 |
| `write()` | limit·freshness 검사 후 command packet 전송 |
| `on_deactivate()` | zero/hold/torque-off 중 정한 안전 동작 수행 |

`read()`나 `write()` 오류를 무시하고 마지막 command를 계속 보내지 않는다. controller manager에 error를 반환하고 MCU watchdog이 작동해야 한다.

## 6. Python과 C++ 역할

| 기능 | 언어 |
|---|---|
| policy inference·experiment orchestration | Python |
| state estimator prototype | Python, 주기 미달 시 C++ |
| `ros2_control` SystemInterface | C++ |
| packet codec와 hardware timing | C++ |
| CAD·URDF 검증 script | Python |

같은 hardware driver를 Python과 C++로 중복 작성하지 않는다. 사용자 요구의 두 언어는 서로 다른 적합한 계층에서 사용한다.

## 7. Hardware 없이 먼저 검사한다

| 단계 | 대상 | 통과 조건 |
|---:|---|---|
| 1 | packet codec unit test | encode/decode, CRC, sequence 오류 검출 |
| 2 | fake hardware | controller load·activate·deactivate 정상 |
| 3 | actuator 1개 rig | direction, offset, limit, timeout, temperature |
| 4 | 한쪽 다리 | 동시 bus rate와 전원 안정성 |
| 5 | 전신을 지지대에 고정 | 모든 joint mapping과 emergency stop |
| 6 | 낮은 자세·safety tether | balance controller 검증 |

## 8. 상태와 시간 계약

| 값 | 기준 |
|---|---|
| joint position | rad, URDF zero와 같은 방향 |
| joint velocity | rad/s |
| effort/current | 단위와 측정·추정 여부 명시 |
| IMU | REP-103 축 방향, sensor timestamp |
| control timestamp | Pi 수신 시각과 MCU sample 시각 둘 다 가능하면 저장 |
| dropped packet | 누적 diagnostic counter |

Pi와 MCU clock이 다르면 offset·drift를 측정한다. 수신 시각만 sensor 측정 시각처럼 쓰지 않는다.

## 9. 완료 조건

- power budget·wiring·pinout·fuse·emergency stop 문서 review
- protocol schema 하나 선택, sequence·CRC·timeout 구현
- URDF joint와 hardware mapping 1:1 자동 검사
- `MobinHumanoidSystem` lifecycle과 `read()`·`write()` 오류 경로 검증
- fake hardware, single-joint, one-leg, tethered whole-body 순서 통과
- power-on 기본 disable, 통신 끊김과 과전류·과열에서 안전 상태 확인
- Python policy와 C++ hardware driver 사이 단위·timestamp 계약 기록

**실물 관절이 안전하게 같은 명령 계약을 따를 때만 PATCH-18의 학습 policy를 연결한다.**
