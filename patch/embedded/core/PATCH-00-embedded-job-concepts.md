# Embedded PATCH-00: 임베디드 채용공고 개념 기반 실습 재설계

- 작성일: 2026-08-19
- 대상: `patch/embedded/` 전체 Embedded 트랙
- 결론: **로봇·Humanoid 프로젝트를 capstone으로 두고, 그 앞에 일반 임베디드 개발자 채용공고에서 반복 등장하는 7개 core skill을 작은 board·firmware·Linux·debug·test 실습으로 쪼갠다.**
- 최종 목표: 동작하는 prototype이 아니라 **실제 판매 가능한 제품 수준**까지 설계·인증·양산·검증을 경험한다. 기능 동작은 시작일 뿐이고, 제품 판매는 DFM·인증·test·reliability·문서·비용 gate를 통과해야 한다.

## 임베디드 개발자가 하는 일

임베디드 개발자는 PC에서 돌아가는 프로그램만 만드는 사람이 아니라, **특정 장치 안에서 software와 hardware가 정해진 전원·메모리·시간·통신 제약 안에 동작하게 만드는 사람**이다. 예를 들어 motor가 명령을 받으면 정해진 시간 안에 움직이고, sensor가 오류를 보내면 안전하게 멈추고, 전원이 꺼졌다 켜져도 같은 상태로 복구되는 것까지 책임진다.

### 실제 업무

| 일 | 구체적인 예 |
|---|---|
| datasheet·schematic 읽기 | GPIO가 어느 pin인지, UART baud rate, ADC range, 전원 허용 범위 확인 |
| firmware 작성 | C/C++로 register·driver·제어 logic 구현 |
| RTOS/Linux 설정 | task priority, watchdog, device tree, systemd service |
| build·flash | cross-compile 후 board에 firmware 올리기 |
| debugging | GDB, logic analyzer, oscilloscope로 crash·timing·전기 신호 확인 |
| test | unit test, fault injection, HIL로 재현 가능한 통과 조건 작성 |
| 문서화 | pin map, protocol, timing, power budget, fault case 기록 |
| 형상관리 | Git commit, CI, board·firmware version 추적 |

### 계층

```text
hardware
  ↓
bootloader
  ↓
RTOS 또는 Linux kernel
  ↓
device driver / HAL / BSP
  ↓
middleware
  ↓
application
```

| 직무 | 주로 다루는 계층 |
|---|---|
| firmware 개발자 | MCU, RTOS, driver, 제어 loop |
| Embedded Linux 개발자 | kernel, device tree, driver, BSP, systemd |
| board bring-up 엔지니어 | bootloader, clock, memory, power, peripheral 초기화 |
| robot control 개발자 | ROS 2, micro-ROS, actuator interface, state machine |

공고마다 경계가 다르다. 신입은 위 전체를 한 번에 전문가 수준으로 하지 않는다. 이 프로젝트에서는 **ESP32에서 MCU/RTOS, Raspberry Pi 5에서 Embedded Linux, test board에서 hardware debugging**을 나눠서 실제로 반복한다.

## 참조한 채용공고

아래 공고는 2026-08-19에 확인했다. 공고 상태와 문구는 바뀔 수 있으므로 링크 원문을 다시 확인한다.

### 국내 공고

| 공고 | 반복 요구 개념 | 링크 |
|---|---|---|
| 에이치오피 임베디드 개발자 | C, Python, SLAM, embedded hardware, firmware | https://www.jobkorea.co.kr/Recruit/GI_Read/49687448 |
| 로보톰 임베디드 로봇 제어 | C/C++, ROS 2, LiDAR, CAN, BLE, OTA | https://www.jobkorea.co.kr/Recruit/GI_Read/49350280 |
| D.Hive 임베디드 firmware | MCU, RTOS, sensor driver, ROS 2 bridge, power, UART/CAN, OTA, 계측 | https://www.wanted.co.kr/wd/366867 |
| Polaris3D Embedded Linux | kernel/system log, C++, driver·IPC·network, GDB·strace·perf, BSP | https://www.wanted.co.kr/wd/307655 |
| 아르비젼 Embedded software | MCU firmware, UART·SPI·I2C·CAN driver | https://www.wanted.co.kr/wd/358905 |
| 아그모 Embedded AI | edge model 최적화, sensor fusion, data lifecycle | https://www.wanted.co.kr/wd/371576 |

### 해외 공고

| 공고 | 반복 요구 개념 | 링크 |
|---|---|---|
| Synkriom Embedded Software Engineer | Embedded C/C++, MCU, RTOS/bare-metal, UART/SPI/I2C/CAN/BLE/Wi-Fi/MQTT | https://www.dice.com/job-detail/71c16c51-7ef4-4c0d-b343-78411db601e2 |
| Tata Technologies FreeRTOS Engineer | Embedded C/C++, ECU, SPI/I2C/UART/CAN/ADC/PWM device driver | https://www.tata.com/careers/jobs/jobdetails?jobId=895746&company=Tata%20Technologies%20Europe&jobTitle=FreeRTOS%20Engineer%20-%20Embedded%20Software%20(Automotive%20ECU)&location=Seattle%2C%20United%20States |
| Yukti Embedded Firmware Engineer | ARM MCU, RTOS, UART/SPI/I2C/CAN/MQTT/TCP-IP/BLE/Wi-Fi | https://www.hirist.tech/j/embedded-firmware-engineer-c-c-1651175 |
| Wabtec Embedded Software Engineer | C/C++, RTOS·bare-metal, TCP/IP·UDP·RS232/RS422·HDLC·SPI·I2C·CAN | https://careers.wabtec.com/fr/job/embedded-software-engineer-in-west-melbourne-fl-united-states-jid-3762 |
| INVICTOSOFT Senior Embedded Firmware Developer | Embedded C/C++, RTOS, device driver, bootloader, BSP, middleware | https://www.hirist.tech/j/senior-embedded-firmware-developer-rtos-microcontroller-1648562 |
| DMC Entry Level Embedded Engineer | C, C++, Python, FreeRTOS, Zephyr, embedded system | https://www.dice.com/job-detail/b2265ded-5d97-46ea-b9d9-79539592b470 |
| Michigan Talent Embedded Software | FreeRTOS·Embedded Linux, I2C/SPI/UART/GPIO driver | https://jobs.mitalent.org/job-seeker/job-details/JobCode/398922053 |
| HCLTech Junior Embedded Software Integration | Embedded C/C++, RTOS task/scheduling, system-level debugging | https://www.monster.com.vn/job/junior-fresher-engineer-embedded-software-integration-hcltech-vietnam-ho-chi-minh-62964888 |
| Arlo Senior Staff Embedded Firmware Engineer | Embedded C, modern C++, HAL, bootloader, BSP, system bring-up | https://www.tealhq.com/job/senior-staff-embedded-firmware-engineer_7ea1a3270353f1d734077cb21166b2bd665f4 |

위 공고에서 추출한 개념을 아래 7개 core skill로 묶었다. 공고 문장 자체는 저작권이 있을 수 있으므로 저장소에 복사하지 않고 요구역량만 요약한다.

## 개념 설명 : MCU

MCU는 작은 컴퓨터 하나가 아니라, CPU·flash·RAM·GPIO·UART·I2C·SPI·ADC·timer 같은 입출력 장치가 한 chip에 들어 있는 제어용 chip이다. Linux가 돌아가는 Raspberry Pi 5와 달리, MCU는 보통 하나의 firmware만 실행하고 전원·motor·sensor 같은 짧은 주기 작업을 맡는다.

| 구분 | MCU | Linux가 있는 SoC |
|---|---|---|
| 예 | ESP32, STM32, Raspberry Pi Pico | Raspberry Pi 5 |
| OS | bare-metal 또는 RTOS | Linux |
| 강점 | 고정된 주기, 빠른 wakeup, 작은 전력 | ROS 2, vision, logging, 복잡한 network |
| 이 프로젝트 역할 | encoder·motor·watchdog·안전 신호 | ROS 2, policy, camera, log |

이 PATCH는 가지고 있는 ESP32 보드를 기본 MCU로 사용한다. ESP32는 Wi-Fi·Bluetooth가 있는 MCU지만, 첫 실습에서는 UART·GPIO·ADC·PWM·timer와 FreeRTOS를 쓰는 firmware부터 시작한다.

## 보유 board와 Arduino/ESP32 구분

보유 board:

| board | 종류 | 이 트랙에서 역할 |
|---|---|---|
| ESP32 dev board | 32-bit MCU | core PATCH-01~04 기본 실습 |
| Raspberry Pi 5 | Linux SoC | core PATCH-05 Embedded Linux |
| Arduino Limited Edition (UNO 계열) | 8-bit MCU | 단순 GPIO·초저전력·analog 확인용 |

Arduino와 ESP32는 서로 대체재가 아니라 목적이 다르다.

| 구분 | Arduino UNO 계열 | ESP32 |
|---|---|---|
| MCU | ATmega328P, 8-bit | Xtensa LX6 / RISC-V, 32-bit |
| clock | 16 MHz | 240 MHz |
| RAM | 2 KB | 320+ KB |
| flash | 32 KB | 4 MB+ |
| Wi-Fi / BLE | 없음 | 내장 |
| OS | super loop | FreeRTOS |
| 적합 | 단순 on/off, 초저전력, analog | micro-ROS, Wi-Fi/BLE, RTOS, 다중 센서 |

이 트랙은 micro-ROS·RTOS·주변장치 driver를 목표로 하므로 기본 MCU는 ESP32다. Arduino UNO 계열은 Wi-Fi·RTOS·micro-ROS가 불가하므로 core 실습에는 쓰지 않고, 단순 GPIO·전력·analog 확인이 필요할 때만 선택한다.

## 채용공고에서 반복되는 개념

2026-08-19에 확인한 일반 임베디드 공고에서 반복되는 표현을 프로젝트 skill로 바꾼다.

| 채용공고 표현 | 실제 뜻 | 이 로드맵의 실습 |
|---|---|---|
| Embedded C/C++ | PC C/C++이 아니라 memory·register·interrupt·linker를 고려한 구현 | PATCH core 1 |
| MCU, ARM, bare-metal | OS 없이 startup·clock·GPIO·timer·ISR를 직접 실행 | PATCH core 1 |
| RTOS, FreeRTOS, Zephyr | task·scheduler·queue·mutex·timer·watchdog | PATCH core 2 |
| UART, SPI, I2C, CAN, USB, BLE | bit·frame·clock·address·arbitration·timeout이 있는 통신 | PATCH core 3 |
| device driver, HAL, BSP | hardware를 일관된 API로 노출하고 board를 OS가 부팅하게 만드는 계층 | PATCH core 3·4 |
| Embedded Linux, device tree, kernel | Linux가 있는 SoC에서 GPIO·bus·driver·boot를 다루는 일 | PATCH core 4 |
| logic analyzer, oscilloscope, GDB, JTAG/SWD | software만이 아니라 전기 신호·timing·crash 위치까지 보는 debugging | PATCH core 5 |
| bootloader, firmware update, OTA | 부팅과 안전한 firmware 교체 | PATCH core 1·5 |
| power, ADC, PWM, motor control | analog·전력·actuator를 안전하게 제어 | PATCH core 6 |
| ROS 2, micro-ROS | Linux와 MCU 사이 robot middleware 연결 | 프로젝트 적용 |
| CI, unit test, HIL | hardware 없이 빠르게 검증하고, hardware가 있는 test는 구분 | PATCH core 5·6 |
| DFM, DFT, BOM | 공장에서 조립·검사 가능한 설계 | PATCH core 8 |
| EMC, ESD, 인증 | 판매 허용을 위한 규제·시험 | PATCH core 8 |
| manufacturing test, reliability | 양산 불량 선별과 실제 사용 환경 내구성 | PATCH core 8 |

핵심 원칙: **한 번에 robot 전체를 만들지 않는다. 개념마다 2~4시간 lab으로 끝내고, 결과를 측정값·사진·log·test로 남긴다.**

## Embedded core 트랙

```text
PATCH-00  job concept mapping
PATCH-01  ESP32 연결·serial 확인
PATCH-02  C/MCU bare-metal foundation
PATCH-03  RTOS task·timing·watchdog
PATCH-04  peripheral driver와 protocol
PATCH-05  Embedded Linux·BSP·Pi 5
PATCH-06  debug·test·CI·HIL
PATCH-07  soldering·board·power·safety
PATCH-08  제품화·양산·인증
```

실제 core lab 파일:

| 단계 | 파일 |
|---|---|
| PATCH-01 | [PATCH-01-prerequire-esp32-connection.md](PATCH-01-prerequire-esp32-connection.md) |
| PATCH-02 | [PATCH-02-esp32-c-mcu-baremetal.md](PATCH-02-esp32-c-mcu-baremetal.md) |
| PATCH-03 | [PATCH-03-esp32-rtos-timing-watchdog.md](PATCH-03-esp32-rtos-timing-watchdog.md) |
| PATCH-04 | [PATCH-04-peripheral-driver-protocol.md](PATCH-04-peripheral-driver-protocol.md) |
| PATCH-05 | [PATCH-05-pi5-embedded-linux-bsp.md](PATCH-05-pi5-embedded-linux-bsp.md) |
| PATCH-06 | [PATCH-06-debug-test-ci-hil.md](PATCH-06-debug-test-ci-hil.md) |
| PATCH-07 | [PATCH-07-board-soldering-rtos-hil.md](PATCH-07-board-soldering-rtos-hil.md) |
| PATCH-08 | [PATCH-08-product-engineering-and-certification.md](PATCH-08-product-engineering-and-certification.md) |

기존 Yahboom·Humanoid PATCH는 이 core 뒤의 **프로젝트 적용**으로 둔다. core lab은 ESP32 하나와 Raspberry Pi 5, 작은 test board로 시작하고, 실물 Yahboom이 없어도 대부분 진행할 수 있다.

## PATCH-02: C/MCU bare-metal foundation

| lab | 목표 | 통과 조건 |
|---|---|---|
| MCU 개발환경 | ESP-IDF build·flash | `hello_world`와 blink binary가 flash됨 |
| register/GPIO | datasheet 기반 pin mux, output, pull-up 설정 | button이 LED를 제어 |
| timer/PWM | hardware timer와 PWM duty 변경 | 주파수·duty를 scope로 확인 |
| interrupt/ISR | GPIO·timer interrupt와 volatile flag | button debounce 없이도 event 손실 기록 |
| memory map/linker | `.text`, `.data`, `.bss`, stack, heap 위치 확인 | map file에서 symbol 주소 설명 |
| C memory trap | `const`, `volatile`, `static`, pointer, alignment | 의도한 bug를 재현하고 원인 기록 |

첫 MCU는 보유한 ESP32를 기본으로 한다. Yahboom에 붙은 ESP32 firmware를 바로 수정하지 않고, 별도 ESP32 dev board에서 기초 lab을 먼저 익힌다.

참고: [ESP-IDF Get Started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html), [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

## PATCH-03: RTOS task·timing·watchdog

| lab | 목표 | 통과 조건 |
|---|---|---|
| task 생성 | 1~100 ms 주기의 task 3개 | 지정 우선순위로 실행 |
| queue | ISR→task, task→task data 전달 | overflow·block timeout 처리 |
| mutex/semaphore | 공유 자원 보호와 event 동기화 | race가 재현되지 않음 |
| timer | software/hardware timer callback | deadline·jitter 기록 |
| watchdog | task hang·command timeout 감지 | enable off 또는 reset |
| timing 측정 | GPIO toggle + logic analyzer | p50·p95·max jitter, deadline miss 구분 |

RTOS 실습은 보유한 ESP32와 ESP-IDF FreeRTOS 하나로만 진행한다. Zephyr는 채용공고에서 등장하는 이름으로만 확인하고, 같은 firmware를 두 RTOS로 중복 구현하지 않는다.

참고: [ESP-IDF FreeRTOS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html), [Zephyr getting started](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

## PATCH-04: peripheral driver와 protocol

| bus | lab | 통과 조건 |
|---|---|---|
| UART | TX→RX loopback, frame, CRC, timeout | corruption·drop 검출 |
| I2C | OLED/EEPROM/GPIO expander address scan | NACK·bus hang 복구 |
| SPI | loopback과 한 sensor read | clock polarity·phase, CS 동작 |
| CAN | `vcan` 두 process 후 실제 CAN transceiver | ID·timeout·재전송 확인 |
| GPIO/ADC/PWM | button, potentiometer, LED, motor-enable simulation | unit 변환과 안전 기본값 |

driver는 vendor HAL을 먼저 사용하고, register부터 다시 쓰는 것은 특정 bug를 재현할 때만 한다. protocol 문서에는 byte order, bit field, sequence, timeout, error recovery를 남긴다.

## PATCH-05: Embedded Linux·BSP·Pi 5

| lab | 목표 | 통과 조건 |
|---|---|---|
| Pi 5 GPIO | `libgpiod`, `gpiodetect`, `/dev/gpiochip*` | button/LED를 userspace에서 제어 |
| device tree overlay | `uart1-pi5`, `i2c1-pi5`, `spi` overlay | `/dev/ttyAMA*`, `/dev/i2c-*`, `/dev/spidev*` 생성 |
| bus loopback | UART·I2C·SPI loopback | 각 bus에서 echo/read 성공 |
| SocketCAN | `vcan0`와 실제 CAN HAT | `candump`/`cansend` 동작 |
| boot/service | systemd service로 node 자동 시작 | 재부팅 뒤 interface 복원 |
| cross-compile | host에서 ESP32 binary 빌드 | 같은 source가 CI와 board에서 재현 |

Raspberry Pi 5는 RP1이 GPIO를 관리하므로 legacy `RPi.GPIO` 대신 `gpiod`를 사용한다.

참고: [Raspberry Pi 공식 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html), [libgpiod](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/about/), [SocketCAN](https://docs.kernel.org/networking/can.html)

## PATCH-06: debug·test·CI·HIL

| lab | 목표 | 통과 조건 |
|---|---|---|
| GDB/OpenOCD | breakpoint, memory, call stack, register 확인 | crash 위치를 source line까지 찾음 |
| SWD/JTAG | ESP32 USB-JTAG 또는 외부 USB-UART debug | firmware를 중단·재개하고 flash |
| logic analyzer | GPIO toggle, UART/I2C/SPI frame | timing diagram과 protocol decode |
| unit test | parser·CRC·state machine fixture | 오류 입력마다 pass/fail |
| CI | build·native test·static check | GitHub Actions에서 재현 |
| HIL | Pi 5 ↔ MCU ↔ test board | 소프트웨어·전기·timing을 함께 검증 |

hardware를 쓰지 않은 CI 결과와 HIL 결과를 같은 `pass`로 합치지 않는다.

## PATCH-07: soldering·board·power·safety

저전력 board를 먼저 만들어 다음을 검증한다.

| 검사 | 방법 | 통과 조건 |
|---|---|---|
| 전원 short | 전원 인가 전 continuity | 3.3 V·5 V·GND rail 간 short 없음 |
| cold joint/bridge | 확대경·multimeter | pad·lead wetting, 인접 pin 분리 |
| e-stop chain | NC switch·LED | software와 무관하게 enable off |
| MCU watchdog | Pi process 중단 | 정해진 timeout 뒤 안전 상태 |
| motor enable simulation | motor 대신 LED | 정상 command·fault에서 기대 동작 |

motor power stage는 이 단계를 통과한 뒤 Embedded PATCH-05의 power budget과 single-joint rig에서만 추가한다.

## 프로젝트 적용 순서

core lab을 통과하면 기존 프로젝트 PATCH를 다음 순서로 연결한다.

```text
core PATCH-01~07
   ↓
Yahboom bring-up·safety·RL Sim2Real
   ↓
Humanoid reference·CAD·URDF
   ↓
Humanoid electronics·ros2_control·HIL
   ↓
core PATCH-08 제품화·양산·인증
```

이 순서는 모든 core skill을 한 번에 익히고 시작하라는 뜻이 아니다. board가 준비된 lab부터 병렬로 진행할 수 있지만, motor를 실제로 구동하는 단계는 `core PATCH-07`의 safety chain과 `Embedded PATCH-05`의 power budget을 통과한 뒤에만 허용한다. 제품 판매를 주장하려면 마지막 `core PATCH-08`의 DFM·인증·양산 test·reliability·문서·비용 gate까지 증거가 있어야 한다.

## GitHub 증거

| 문서 | 기록 |
|---|---|
| `docs/embedded/core/00_job_concepts.md` | 채용공고 skill과 lab mapping |
| `docs/embedded/core/01_c_mcu.md` | register·memory·linker·interrupt 실험 |
| `docs/embedded/core/02_rtos_timing.md` | task table, priority, jitter, watchdog |
| `docs/embedded/core/03_protocol.md` | UART/I2C/SPI/CAN frame과 fault |
| `docs/embedded/core/04_pi5_bsp.md` | overlay, `/dev` node, boot/service log |
| `docs/embedded/core/05_debug_test_hil.md` | GDB·logic analyzer·CI·HIL 결과 |
| `docs/embedded/core/06_board_power.md` | soldering 검사, power, e-stop, fault |
| `docs/embedded/core/08_product_engineering.md` | DFM/DFT, firmware release, cost |
| `docs/embedded/core/08_certification.md` | KC/CE/FCC/RoHS 체크리스트 |
| `docs/embedded/core/08_manufacturing_test.md` | ICT/FCT·calibration·burn-in |
| `docs/embedded/core/08_reliability.md` | 온습도·진동·배터리 cycle |

모든 실측 결과에는 board revision, firmware commit, tool, measurement setup을 함께 기록한다.
