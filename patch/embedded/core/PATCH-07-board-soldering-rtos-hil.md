# Embedded Core PATCH-07: 납땜 보드·Raspberry Pi 5 HAT·RTOS HIL bring-up

- 작성일: 2026-08-19
- 선행 조건: Embedded PATCH-00의 Yahboom interface 이해, Embedded PATCH-05의 MCU/RTOS architecture 이해
- 대상: 향후 `humanoid/hardware/board_lab/`, `humanoid/firmware/`, `docs/embedded/06_*`
- 결론: **motor를 직접 납땜해 구동하기 전에 저전력 digital·UART·I2C·SPI·CAN·emergency-stop 신호만 있는 Raspberry Pi 5 HAT와 MCU test board를 먼저 만든다. 이 board에서 납땜, Linux bus, RTOS task·watchdog·packet·fault를 측정하고 통과한 뒤 motor power stage로 확장한다.**

## 검증한 공개 자료

| 자료 | 확인한 내용 | 이 PATCH에서의 사용 |
|---|---|---|
| [Adafruit Guide To Excellent Soldering](https://learn.adafruit.com/adafruit-guide-excellent-soldering) | 납땜 인두 온도, rosin-core solder, flux, 냉납·bridge·pad 손상 판별 | 첫 soldering kit와 검사 기준 |
| [Raspberry Pi 5 공식 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) | Pi 5의 GPIO는 RP1이 관리하며 legacy `RPi.GPIO`가 아니라 `libgpiod`/`gpiod` 사용 | Pi 5 GPIO·UART·I2C·SPI loopback |
| [ESP-IDF FreeRTOS 문서](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html) | task, queue, semaphore, timer, interrupt watchdog 구조 | Yahboom ESP32를 대체·보완하는 firmware 계약 |
| [FreeRTOS queue·timer 문서](https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/00-TaskHandle.html) | task priority, queue, timer, watchdog의 표준 의미 | firmware task 표와 timeout 설계 |
| [micro-ROS 문서](https://micro.ros.org/docs/) | MCU와 ROS 2 사이 transport·message·lifecycle | MCU↔Pi 통신을 micro-ROS로 확장할 때 사용 |
| [SocketCAN 공식 문서](https://docs.kernel.org/networking/can.html) | Linux에서 CAN network interface를 쓰는 표준 경로 | Pi 5 CAN HAT과 `vcan` test |
| [Zephyr CAN counter sample](https://docs.zephyrproject.org/latest/samples/drivers/can/counter/README.html) | 채용공고 개념 확인용 CAN sample | ESP32 CAN 실습 전 `vcan`으로 동작 이해 |

납땜은 화상·연기·ESD 위험이 있는 작업이다. 환기가 되는 곳에서 rosin-core solder를 쓰고, acid-core plumbing solder를 사용하지 않는다. 인두가 뜨거울 때 손과 cable을 멀리하고, 전원을 끊은 board에서만 측정한다.

## Embedded

### 개념

| 개념 | 쉬운 설명 | 이 PATCH에서 확인하는 것 |
|---|---|---|
| cold joint / bridge | 납이 pad와 lead에 제대로 wetting되지 않거나 인접 pin끼리 붙은 불량 | continuity·확대 사진·multimeter |
| ESD | 정전기로 반도체가 손상되는 현상 | wrist strap·grounded mat·포장 규칙 |
| level shifter | 3.3 V와 5 V 신호를 서로 안전하게 변환 | Pi 5 GPIO와 peripheral 사이 voltage 확인 |
| RP1 | Raspberry Pi 5에서 GPIO·UART·SPI·I2C 등을 묶는 peripheral controller | pin mux와 `/dev/gpiochip*` 이해 |
| pinctrl overlay | Linux device tree에서 GPIO pin 기능을 고르는 설정 | `uart1-pi5`, `i2c1-pi5` 등 |
| loopback | 출력을 자신의 입력으로 되돌려 경로를 검사 | UART TX→RX, SPI MOSI→MISO |
| task deadline | RTOS task가 끝나야 하는 최대 시각 | 주기·jitter·watchdog 측정 |
| watchdog | software가 멈추거나 명령이 오래되면 안전 상태로 전환 | MCU firmware와 Pi side stale command |
| HIL | 실제 hardware를 제어 loop에 연결해 software·전기·timing을 함께 검사 | motor 전에는 digital I/O와 bus로 축소 |

RTOS 사용이 실시간을 보장하지 않는다. task priority, ISR 처리 시간, queue 차단, 최악 실행 시간을 측정해야 `deadline miss`를 주장할 수 있다.

### 직접 설계·검증할 것 / 재사용할 것 / 제외할 것

| 직접 설계·측정 | 재사용 | 이 단계에서 제외 |
|---|---|---|
| board schematic·soldering·visual/electrical test | Raspberry Pi official pinout과 vendor driver | motor power stage 직접 설계 |
| Pi 5 GPIO·UART·I2C·SPI loopback | `libgpiod`, Linux kernel driver | custom kernel driver |
| RTOS task·queue·timer·watchdog firmware | ESP-IDF FreeRTOS HAL와 sample | 양산용 bootloader·OTA |
| sequence·length·CRC·timeout packet | 검증된 CRC library | custom 암호화 |
| e-stop·enable·fault LED 안전 chain | vendor motor driver의 enable interface | 기능 안전 인증 |
| `vcan`/CAN HAT 두 node 통신 | SocketCAN과 MCP2515/transceiver | multi-node CAN arbitration 최적화 |

신입 범위는 board를 만들고 loopback·packet·watchdog을 계측해 root cause를 남기는 것이다. 1~2년차는 CAN/UART fault를 재현하고 task deadline miss를 logic analyzer 또는 logic analyzer/oscilloscope로 task·bus·driver 구간으로 분리한다.

## 계획 파일

```text
humanoid/hardware/board_lab/
├── pi5_hat/
│   ├── pi5_hat.sch
│   ├── pi5_hat.kicad_pcb
│   ├── bom.csv
│   └── test_points.md
humanoid/firmware/
├── zephyr_pico_can/
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── boards/
│   └── src/main.c
└── esp32_freertos/
    ├── CMakeLists.txt
    ├── sdkconfig.defaults
    └── main/app_main.c
docs/embedded/
├── 06_soldering_report.md
├── 06_pi5_bus_report.md
├── 06_rtos_timing_report.md
├── 06_protocol_report.md
└── 06_hil_report.md
```

`bom.csv`에는 part number, package, 3.3 V/5 V 여부, 대체 가능 여부를 기록한다. 비밀번호·token은 저장하지 않는다.

## 1. 저전력 test HAT를 먼저 정의한다

첫 board는 motor driver를 넣지 않는다. 다음 신호만 포함한다.

| 기능 | 부품 예 | 검사 |
|---|---|---|
| emergency stop | NC switch, pull-up, LED | 누르면 Pi GPIO와 MCU GPIO가 함께 low |
| motor enable simulation | LED + series resistor | firmware watchdog가 enable를 끊는지 확인 |
| UART | 3.3 V UART header 또는 level shifter | Pi↔MCU framed packet |
| I2C | I2C OLED/GPIO expander | address scan과 register read |
| SPI | SPI loopback 또는 sensor | MISO echo와 clock 확인 |
| CAN | MCP2515/MCP2562 또는 SN65HVD230 | 두 node CAN ID 전송 |
| power monitor | 전압 divider + ADC 또는 INA219 | idle·fault 전류 |

이유: 납땜 불량과 firmware 결함을 motor의 큰 전류와 섞으면 원인 분리가 어렵다. board가 검증된 뒤에만 motor rail을 별도 board 또는 개정판으로 확장한다.

## 2. 납땜과 board 검사를 한다

조립 순서:

1. 낮은 부품부터 resistor·header·capacitor를 납땜한다.
2. connector와 큰 부품은 나중에 조립한다.
3. 각 단계마다 확대 사진과 continuity를 남긴다.

검사 항목:

| 검사 | 방법 | 통과 조건 |
|---|---|---|
| 전원 short | 전원 인가 전 multimeter continuity | 3.3 V·5 V·GND rail 사이 short 없음 |
| cold joint | 밝은 조명·확대경 육안 | pad와 lead가 매끈한 fillet |
| bridge | 인접 pin continuity | 의도하지 않은 연결 없음 |
| diode·LED 방향 | silkscreen과 datasheet | 전원 인가 후 예상 LED만 점등 |
| connector 극성 | 1:1 pinout sheet | Pi 5 40-pin과 1번 pin 일치 |

결과는 `docs/embedded/06_soldering_report.md`에 사진과 함께 저장한다.

## 3. Raspberry Pi 5 bus를 Linux에서 검사한다

Pi 5에서는 legacy `RPi.GPIO`를 쓰지 않는다. `/boot/firmware/config.txt` overlay와 `gpiod`/`libgpiod`를 사용한다.

```bash
cat /proc/device-tree/model
gpiodetect
gpioinfo | head -40
ls /dev/ttyAMA* /dev/ttyS* /dev/i2c-* /dev/spidev* 2>/dev/null
```

| bus | 최소 검사 |
|---|---|
| GPIO | `gpioset`/`gpioget`으로 LED on/off와 e-stop 입력 |
| UART | TX→RX loopback 후 `pyserial`/`minicom` echo |
| I2C | `i2cdetect -y <bus>`로 address 확인 |
| SPI | MOSI→MISO loopback, 1 MHz부터 시작 |
| CAN | `ip link add dev vcan0 type vcan`, 이후 HAT에서 `can0` 확인 |

Pi 5 overlay는 공식 문서와 `dtoverlay -h`로 현재 image에 존재하는 이름을 확인한다. 온라인 핀맵의 예전 이름을 그대로 넣지 않는다.

## 4. MCU에 RTOS firmware를 올린다

첫 RTOS target은 둘 중 하나만 선택한다.

| 선택 | 이유 |
|---|---|
| 보유 ESP32 + ESP-IDF FreeRTOS | 별도 Pico 구매 없이 UART·GPIO·ADC·PWM·CAN 실습 |
| Yahboom ESP32 + vendor firmware | core lab 통과 뒤 실제 motor 제어 계층 재현 |

첫 RTOS 실습은 ESP32 + ESP-IDF FreeRTOS 하나로 진행한다. Zephyr나 Pico로 같은 firmware를 중복 구현하지 않는다.

| task | 주기 | 우선순위 | 안전 동작 |
|---|---:|---:|---|
| command_rx | event | 높음 | CRC·sequence·timeout 검사 |
| control_loop | 1~10 ms | 높음 | enable를 갱신하고 deadline miss 기록 |
| telemetry_tx | 20~100 ms | 중간 | state·fault·counter 전송 |
| led_diag | 100 ms | 낮음 | task liveness 표시 |
| watchdog | 10 ms | 최고 | command timeout이면 enable off |

ESP32는 ESP-IDF `hello_world`를 flash한 뒤 `xTaskCreate`, `xTimerCreate`, `esp_task_wdt`를 사용해 같은 다섯 task를 구성한다.

## 5. Pi↔MCU packet과 fault를 만든다

최소 frame은 Embedded PATCH-05보다 작게 유지한다.

| 방향 | field |
|---|---|
| Pi→MCU | `magic`, `sequence`, `timestamp_ms`, `mode`, `enable`, `crc16` |
| MCU→Pi | `magic`, `sequence`, `state`, `fault`, `loop_count`, `crc16` |

검사:

| fault | 기대 동작 |
|---|---|
| CRC 오류 | 수신 frame 폐기, 오류 counter 증가 |
| sequence 누락 | 누락 event 기록, 오래된 command 거부 |
| command timeout | MCU watchdog이 enable off |
| UART 단선 | Pi가 timeout을 감지하고 safe state로 전환 |
| board 재기동 | power-on 기본 enable false |

Python/Pi 쪽은 `docs/embedded/06_protocol_report.md`에 encode/decode test vector를 남긴다. C++ 구현은 control 주기나 latency가 Python에서 부족할 때만 추가한다.

## 6. CAN HIL과 motor enable simulation을 수행한다

1. Linux `vcan0`으로 두 process의 CAN ID·timeout test를 만든다.
2. 실제 CAN HAT를 두 node에 연결해 `candump`/`cansend`로 검증한다.
3. 마지막에 motor 대신 LED를 enable에 연결해 다음을 확인한다.

| 시험 | 통과 조건 |
|---|---|
| 정상 command | 1 kHz 이하 지정 주기로 LED/state 갱신 |
| e-stop 누름 | software 상태와 무관하게 enable LED off |
| MCU watchdog | Pi process가 멈춰도 정해진 timeout 뒤 off |
| CAN bus error | frame loss가 기록되고 stale command가 유지되지 않음 |

이 단계를 통과한 뒤 motor driver는 전원·fuse·전류 제한을 별도로 넣은 다음 revision에서만 추가한다.

## SW 실습

| 실습 | 입력·방법 | 산출물 | 통과 조건 |
|---|---|---|---|
| GPIO/bus loopback | Pi 5 board와 `gpiod` | `06_pi5_bus_report.md` | GPIO·UART·I2C·SPI 각각 echo/read 성공 |
| RTOS task table | ESP-IDF FreeRTOS firmware | `06_rtos_timing_report.md` | priority·period·deadline·watchdog 명시 |
| packet codec | encode/decode, corrupt frame | test vector | CRC·sequence·length 오류 검출 |
| fault simulation | UART 단선·packet 손상·Pi kill | log·fault counter | 지정 safe state 전이 |
| CAN vcan test | 두 process | capture log | ID·timeout·재전송 정책 확인 |

## HW 실습

| 실습 | 필요한 장비 | 측정값·산출물 | 통과 조건 |
|---|---|---|---|
| 납땜 board 검사 | soldering iron, multimeter, 확대경 | continuity·사진 | short·bridge 없음 |
| Pi 5 loopback | 점퍼선 또는 test header | bus log | 3.3 V level과 pin mapping 일치 |
| MCU RTOS timing | logic analyzer/oscilloscope | period·jitter·p95·max | deadline miss 없거나 원인 식별 |
| e-stop chain | NC switch·LED·즉시 전원 차단 | off까지 시간 | software 경유 없이 enable off |
| CAN HIL | CAN transceiver 2개, termination | candump | 실제 bus에서 양방향 전송 |

**motor를 넣기 전까지 이 PATCH의 HW 통과는 전류·토크·기계적 안전을 검증하지 않는다. motor 단계는 Embedded PATCH-05의 single-joint rig와 power budget이 선행되어야 한다.**

## 완료 조건

- `pi5_hat` board가 납땜되고 continuity·short 검사 통과
- Pi 5에서 GPIO·UART·I2C·SPI loopback이 기록으로 남음
- ESP-IDF FreeRTOS task 5종이 deadline·jitter와 함께 기록됨
- Pi↔MCU packet이 CRC·sequence·timeout fault를 통과
- `vcan`과 실제 CAN HAT 양방향 시험 완료
- e-stop과 MCU watchdog이 motor 대신 enable LED를 안전하게 끊음
- 모든 실측값에 board revision·firmware commit·측정 도구가 함께 기록됨
