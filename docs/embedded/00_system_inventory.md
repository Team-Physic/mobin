# Yahboom·Raspberry Pi 5 System Inventory

- 확인일: 2026-09-02
- 상태: 보유 장비 식별 단계, 실물 bring-up 미완료
- 목적: vendor 제품과 실제 보유 장비를 구분하고, ROS 2·Edge AI·MCU 책임 경계를 고정한다.

## 1. 상태 표기

- **확인**: 사용자 출력, 실물 확인 또는 공식 제품 문서로 확인됨.
- **제품군 사양**: 공식 제품군에는 명시됐지만 실제 board revision에서 아직 확인하지 않음.
- **미확인**: 실물 label, 사진, 명령 출력 또는 주문 옵션 확인이 필요함.
- **제안**: 향후 통합 목표이며 현재 동작을 주장하지 않음.

## 2. 보유 장비

| 구성 | 현재 식별 | 상태 | 근거·남은 확인 |
|---|---|---|---|
| Robot | Yahboom ESP32 MicroROS Robot Car | 확인 | [공식 제품](https://category.yahboom.net/collections/embedded-series/products/microros-esp32), SKU `6000200573` |
| Robot option | Standard 또는 Vision | 미확인 | Camera 보유는 확인됐지만 주문 옵션·본체 label을 확인하지 않음 |
| Main MCU | ESP32-S3 계열 MicroROS control board | 제품군 사양 | 실제 chip은 board label 또는 `esptool.py chip_id`로 확인 |
| MCU module | `ESP32-S3-WROOM-1U-N4R2` | 제품군 사양 | [Yahboom control-board 사양](https://category.yahboom.net/products/microros-board)의 제품군 답변 기준 |
| Motors | 310 encoder motor ×4 | 제품군 사양 | Connector, gear ratio, encoder CPR 미확인 |
| Battery | 7.4 V battery | 제품군 사양 | Capacity, connector, measured voltage 미확인 |
| IMU | Onboard 6-axis IMU | 제품군 사양 | Chip model, frame convention, rate 미확인 |
| LiDAR | Kit에 포함된 TOF LiDAR | 보유 확인·모델 미확인 | 실제 label 필요. MS200이라고 단정하지 않음 |
| Camera | Kit에 포함된 camera | 보유 확인·모델 미확인 | Resolution, transport, FPS, lens 미확인 |
| Compute | Raspberry Pi 5 4 GB | 확인 | `free -h`: total memory `4.0 GiB` |
| Logic analyzer | 보유 | 모델 미확인 | Model, channel count, maximum sample rate 확인 필요 |

## 3. 제품 기본 구조

보유 제품의 공식 기본 구조는 Raspberry Pi를 사용하지 않는다.

```text
PC virtual machine
  ROS 2 Humble · decision nodes · visualization
             │
             │ Wi-Fi UDP / micro-ROS
             ▼
Yahboom ESP32-S3 control board
  motor PWM · encoder · IMU · LiDAR · battery monitoring
             │
             ▼
      310 encoder motors ×4
```

이 구조를 먼저 실행하는 이유는 vendor hardware, firmware, motor direction, encoder와 sensor가 정상인지 확인하기 위해서다. 이 단계가 없으면 Pi 5 통합 실패가 Pi 설정 문제인지 기존 robot 문제인지 분리할 수 없다.

## 4. 제안하는 Pi 5 통합 구조

```text
Camera · LiDAR
      │
      ▼
Raspberry Pi 5 4 GB
  ROS 2 · YOLO inference · Camera–LiDAR association
  tracking · CPA semantic safety · logging
      │
      │ serial 또는 Wi-Fi UDP / micro-ROS
      ▼
ESP32-S3 control board
  encoder read · motor output · command timeout · watchdog
      │
      ▼
310 encoder motors ×4
```

이 구조는 **제안**이다. Pi 5에서 micro-ROS agent와 현재 `mobin` node가 실물 interface로 동작한 증거는 아직 없다.

## 5. 현재 저장소 가정과 실물의 차이

기존 [Embedded PATCH-00](../../patch/embedded/PATCH-00-yahboom-hardware-bringup.md)은 `MicroROS-Pi5` 제품을 전제로 작성됐다. 실제로 확인된 제품은 PC virtual machine을 기본 controller로 사용하는 `MicroROS-ESP32` 계열이다.

따라서 다음을 그대로 주장하지 않는다.

- Pi 5가 vendor image로 robot을 이미 제어한다.
- LiDAR가 MS200이다.
- Camera가 `MicroROS-Pi5` bundle과 동일하다.
- 현재 URDF, topic, TF와 실제 robot interface가 일치한다.

PATCH를 다시 쓰기 전에 실물 inventory와 vendor baseline 결과를 먼저 확보한다.

## 6. Bring-up 체크리스트

### A. 전원 인가 전

- [ ] Robot, control board, Pi 5, camera, LiDAR의 전체 사진을 촬영한다.
- [ ] Serial number, Wi-Fi credential, 집 내부 등 민감정보를 공개 사진에서 가린다.
- [ ] Board silkscreen과 revision을 기록한다.
- [ ] Camera와 LiDAR label을 기록한다.
- [ ] Battery capacity, nominal voltage, connector polarity를 확인한다.
- [ ] 바퀴를 바닥에서 띄우고 물리 power switch에 손이 닿는 상태로 둔다.

### B. Vendor baseline

- [ ] Yahboom 제공 firmware·VM·ROS 2 version과 download checksum을 기록한다.
- [ ] PC VM에서 micro-ROS agent가 ESP32와 연결되는지 확인한다.
- [ ] Node, topic, message type, QoS, publish rate를 기록한다.
- [ ] 각 motor의 전진 방향과 encoder sign을 확인한다.
- [ ] `cmd_vel=0`에서 모든 motor가 정지하는지 확인한다.
- [ ] Camera frame과 LiDAR scan을 각각 저장한다.
- [ ] 60초 rosbag과 bring-up 영상을 남긴다.

### C. Pi 5 inventory

- [x] RAM: 4 GB (`free -h` total `4.0 GiB`)
- [ ] Pi model과 board revision을 기록한다.
- [ ] OS, kernel, architecture를 기록한다.
- [ ] Power supply와 cooling 구성을 기록한다.
- [ ] ROS 2 distribution과 middleware를 기록한다.
- [ ] Camera·LiDAR를 Pi에 연결했을 때 device path와 rate를 기록한다.

### D. Pi 5 통합

- [ ] Pi 5에서 micro-ROS agent를 실행한다.
- [ ] Vendor baseline과 같은 motor·encoder·sensor interface를 재현한다.
- [ ] Simulation topic과 실물 topic의 adapter 필요 여부를 기록한다.
- [ ] 기존 YOLO node를 고정 input으로 먼저 실행한다.
- [ ] 300 frames에서 FPS, p50/p95 latency, peak RSS를 측정한다.
- [ ] Tracking→CPA safety output을 motor command gate에 연결한다.
- [ ] Process 종료와 command timeout에서 motor stop latency를 측정한다.

## 7. 안전 경계

- Vendor motor PID와 전체 firmware를 첫 단계에서 다시 작성하지 않는다.
- 첫 motor test는 바퀴를 띄운 상태에서 수행한다.
- Software emergency stop만 믿지 않고 물리 power-off 경로를 준비한다.
- ESP32 timeout·watchdog가 검증되기 전 자율주행을 바닥에서 실행하지 않는다.
- Wi-Fi 또는 serial 단절 시험은 정상 정지 명령 검증 후 수행한다.
- 실물 시험 결과와 simulation 결과를 같은 `pass`로 합치지 않는다.

## 8. 다음에 기록할 실제 값

| 항목 | 값 |
|---|---|
| Robot option | 미확인 |
| Control-board revision | 미확인 |
| ESP32 chip/module | 미확인 |
| Camera model | 미확인 |
| LiDAR model | 미확인 |
| Motor gear ratio·encoder CPR | 미확인 |
| Battery capacity | 미확인 |
| Pi OS·kernel | 미확인 |
| ROS 2 distribution | 미확인 |
| micro-ROS transport | 미확인 |
| Vendor baseline commit/image | 미확인 |
