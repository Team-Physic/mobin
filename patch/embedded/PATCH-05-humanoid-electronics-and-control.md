# Embedded PATCH-05: Humanoid 전장과 ROS 2 제어 계층

- 작성일: 2026-08-15
- 선행 조건: Embedded PATCH-03 actuator rig, Embedded PATCH-04 URDF
- 대상: 향후 `humanoid/hardware/`, `humanoid/control/`, `humanoid/firmware/`
- 결론: **Raspberry Pi 5는 ROS 2·state estimation·policy·logging을 담당하고, MCU 또는 smart-servo bus controller는 정해진 주기의 actuator I/O·watchdog·limit를 담당한다. ROS 2와 실물 사이에는 하나의 `ros2_control` hardware interface만 둔다.**

## 개념

Linux가 실행되는 Raspberry Pi 5는 학습 policy와 ROS 2에는 적합하지만 motor의 전류·position loop를 항상 정확한 주기로 실행하는 안전 controller로 간주하지 않는다.

Pi 5 선택은 Simulation PATCH-12의 policy benchmark를 통과한다는 조건부 결정이다. 통과하지 못하면 policy model을 줄이거나 accelerator·compute board를 바꾸며, MCU의 actuator loop와 watchdog 책임은 바뀌지 않는다.

| 계층 | 담당 |
|---|---|
| Raspberry Pi 5 | ROS 2 node, IMU/state estimation, policy inference, rosbag, UI |
| MCU/servo controller | actuator command, encoder read, hard limit, watchdog |
| power/safety circuit | fuse, main switch, emergency stop, voltage·current 보호 |

Arduino-compatible MCU는 선택지다. 사용하는 smart actuator가 안정적인 bus SDK와 controller를 제공하면 불필요한 Arduino layer를 추가하지 않는다.

## Embedded

### MCU와 RTOS

| 개념 | 쉬운 설명 | 잘못 설계했을 때 |
|---|---|---|
| task | RTOS scheduler가 실행 순서를 관리하는 작업 단위 | 한 task가 오래 막혀 motor update 지연 |
| interrupt/ISR | hardware event가 발생하면 일반 task보다 먼저 실행하는 짧은 handler | ISR의 blocking으로 전체 지연 증가 |
| queue/message buffer | task 또는 ISR 사이에 소유권이 명확한 data를 전달 | 공유 전역변수 race와 frame 손실 |
| mutex | 여러 task가 같은 자원을 동시에 바꾸지 못하게 보호 | priority inversion과 deadlock |
| deadline | command·sensor 결과가 완료되어야 하는 마지막 시각 | 평균 주기는 맞아도 간헐적 제어 실패 |
| jitter | 실제 실행 주기가 목표 주기에서 흔들리는 정도 | actuator command 간격 불규칙 |
| watchdog | software가 멈추거나 command가 오래되면 reset 또는 torque-off | 마지막 command가 무기한 유지 |
| stack high-water mark | task stack이 가장 많이 사용된 지점 | stack overflow로 임의 crash |

RTOS 첫 실습은 [Zephyr Getting Started](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)의 build·flash와 [Twister test runner](https://docs.zephyrproject.org/latest/develop/twister/index.html)를 사용한다. Zephyr의 `native_sim`·QEMU test를 hardware test로 확장한다. Yahboom vendor firmware가 FreeRTOS 기반이면 [FreeRTOS task·queue·notification 문서](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/00-Developer-docs)를 읽되 **두 RTOS로 같은 firmware를 중복 구현하지 않는다.**

### Hardware interface

| interface | 역할 | 첫 실습 |
|---|---|---|
| GPIO/PWM | enable, direction, motor duty 같은 단순 신호 | emergency input과 motor enable |
| ADC | battery·current 같은 analog voltage sampling | raw count→V/A 변환과 calibration |
| I2C | address가 있는 저속 board-level sensor bus | IMU register read와 timeout |
| SPI | 별도 clock·chip-select를 쓰는 고속 board-level bus | loopback 또는 한 sensor transaction |
| UART | byte stream 기반 MCU↔Pi 통신 | framed packet, sequence, CRC, timeout |
| CAN | 여러 node가 공유하는 차동 bus | 두 node 또는 Linux `vcan`에서 ID·timeout 시험 |

Linux CAN application은 hardware별 character driver를 만들기 전에 [SocketCAN 공식 문서](https://docs.kernel.org/networking/can.html)를 따른다. UART·I2C·SPI도 kernel과 vendor가 이미 제공하는 driver를 먼저 사용한다.

### 신입·1~2년차 설계 범위

| 직접 설계·측정할 것 | 재사용할 것 | 전문가 review가 필요한 것 |
|---|---|---|
| task rate·priority·queue와 timeout 표 | RTOS scheduler와 vendor HAL | hard real-time·기능 안전 주장 |
| version·type·sequence·timestamp·length·CRC packet | 검증된 CRC·serialization 구현 | bootloader·secure OTA 전체 |
| parser corruption·drop·reorder test | official peripheral driver | custom motor power electronics |
| p50·p95·max latency, jitter, deadline miss | ros2_control lifecycle | battery BMS·EMI/EMC·고전류 PCB |
| GDB/SWD로 crash 위치와 stack 확인 | OpenOCD와 board debug probe | 양산용 board bring-up 단독 승인 |

신입은 `native_sim`에서 packet·state machine test를 만든 뒤 single-joint rig로 옮긴다. 1~2년차는 logic analyzer/SWD를 사용해 deadline miss의 원인을 task, bus, actuator 응답으로 분리하고 regression test를 남긴다.

### GitHub에 남길 증거

| 문서 | 필요한 증거 |
|---|---|
| `docs/embedded/05_firmware_architecture.md` | task, priority, period, queue, ISR, watchdog diagram |
| `docs/embedded/05_protocol.md` | byte order, frame, state machine, timeout, version compatibility |
| `docs/embedded/05_timing_report.md` | board·firmware commit, 측정 도구, p50·p95·max·miss |
| `docs/embedded/05_power_and_fault_report.md` | voltage/current/temperature와 disconnect·corruption 결과 |

GitHub Actions에서는 native simulation과 parser test를 실행하고 JUnit·map file·test log를 artifact로 보관한다. 물리 board를 사용하지 않은 CI 결과를 HIL 통과로 표시하지 않는다.

### SW 실습

| 실습 | 입력·방법 | 산출물 | 통과 조건 |
|---|---|---|---|
| firmware state machine | 정상·timeout·CRC 오류·sequence 누락 fixture | native simulation test | fault마다 지정된 disable·torque-off 상태 전이 |
| packet codec | encode/decode와 corrupted frame | unit test·protocol 문서 | byte order·version·length·CRC 검사 통과 |
| RTOS timing 설계 | task period·priority·queue·ISR budget | firmware architecture 표 | blocking 경로와 deadline owner 명시 |
| ros2_control fake hardware | URDF joint와 simulated transport | lifecycle·mapping test | configure·activate·read·write·deactivate 통과 |
| diagnostics·CI | parser fuzz case, map file, native test | JUnit·log·artifact | 물리 HIL과 구분된 SW regression 결과 보존 |

### HW 실습

| 실습 | 필요한 장비 | 측정값·산출물 | 통과 조건 |
|---|---|---|---|
| 전원·배선 | 전원공급기, fuse, emergency stop, multimeter | rail별 idle·peak 전류와 voltage drop | brownout 없이 보호장치가 의도한 전력을 차단 |
| MCU bring-up | MCU board, debug probe, UART/CAN adapter | firmware version, boot log, pin·clock 확인 | power-on 기본 disable과 통신 확인 |
| timing 계측 | logic analyzer 또는 oscilloscope, SWD | loop period·jitter·p95·max·deadline miss | 목표 주기와 timeout 기준 충족 |
| single-joint HIL | actuator 1개, encoder, 하중 지그 | direction·offset·limit·current·temperature | mapping 일치, fault에서 안전 상태 전이 |
| 단계 확장 | 한쪽 다리→지지대 고정 전신→safety tether | bus 부하·전원·동시 update 결과 | 각 단계 통과 뒤에만 다음 단계 진행 |
| 물리 fault injection | cable 분리, frame corruption, 과열·과전류 모사 | watchdog·fault code·정지시간 | Pi process와 무관하게 MCU·power 계층이 제한 수행 |

**CI는 SW 오류를 빠르게 찾고, HIL은 전기·timing·actuator 반응을 검증한다. 두 결과를 같은 `pass`로 합치지 않는다.**

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

Pi 5 rail은 board, Active Cooler, camera와 PCIe accelerator를 실제 장착한 조합의 peak current로 산정한다. CPU benchmark가 빨라도 thermal throttling이나 motor 가속 시 brownout이 발생하면 통과가 아니다.

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

Embedded PATCH-04 Xacro에서 simulation과 real hardware plugin만 parameter로 바꾼다.

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

**실물 관절이 안전하게 같은 명령 계약을 따를 때만 Simulation PATCH-12의 학습 policy를 연결한다.**
