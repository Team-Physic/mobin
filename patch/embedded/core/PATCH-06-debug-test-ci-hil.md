# Embedded Core PATCH-06: debug·test·CI·HIL

- 선행 조건: Core PATCH-02~05에서 firmware·bus·Linux 결과 존재
- 목표: crash와 timing을 software log가 아니라 GDB·logic analyzer·HIL로 찾고, hardware 없이 재현되는 test는 CI로 돌린다.
- 결론: CI 통과와 HIL 통과를 같은 `pass`로 합치지 않는다.

## 개념

| 도구 | 보는 것 |
|---|---|
| GDB | breakpoint, call stack, memory, register |
| OpenOCD/JTAG/SWD | target CPU를 중단·재개하고 flash |
| logic analyzer | 여러 GPIO·UART·I2C·SPI의 digital timing |
| oscilloscope | 전압 파형, rise/fall, analog noise |
| unit test | 함수·parser·state machine의 고정 입력 |
| CI | build·native test·static check를 재현 |
| HIL | 실제 board·sensor·actuator를 포함한 통합 시험 |

## 1. ESP32 debug 연결

ESP32-S3는 USB-JTAG를 쓸 수 있다. 일반 ESP32는 USB-UART bootloader와 별도 JTAG probe가 필요할 수 있다.

```bash
idf.py openocd
```

다른 terminal:

```bash
idf.py gdb
```

GDB에서:

```text
break app_main
continue
bt
info registers
x/16xw 0x3ffb0000
```

crash 시 `idf.py monitor`의 backtrace와 `.elf` symbol을 연결해 source line까지 찾는다.

## 2. logic analyzer

최소 4 channel로 측정한다.

| channel | 신호 |
|---|---|
| 0 | control task GPIO toggle |
| 1 | command packet TX |
| 2 | enable GPIO |
| 3 | watchdog 또는 LED |

기록:

```text
control period: target 5 ms
p50/p95/max jitter
UART frame width·baud
enable off latency after fault
```

## 3. unit test

CRC·packet parser·state machine은 PC에서 `cmake` 또는 host test로 돌린다.

```c
void test_invalid_crc_is_rejected(void) {
    frame_t f = {0};
    f.magic = MAGIC;
    f.seq = 1;
    f.len = 4;
    f.crc = compute_crc(&f, f.len);
    assert(validate(&f));

    f.crc ^= 0x01;
    assert(!validate(&f));
}
```

통과 조건: 정상·CRC 오류·길이 오류·sequence 누락·timeout 각각 pass/fail이 고정 fixture로 재현된다.

## 4. CI

GitHub Actions에서 firmware build와 host test를 실행한다.

```yaml
jobs:
  firmware:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: espressif/esp-idf-ci-action@v1
        with:
          esp_idf_version: v5.3
          target: esp32
          path: humanoid/firmware/lab03_protocol
```

CI artifact:

```text
build log
unit test result
map file
static check log
```

CI에서는 board timing·전원·실제 bus를 검증하지 않는다. 그 항목은 HIL에서만 pass로 기록한다.

## 5. HIL

```text
Pi 5 ↔ UART/USB ↔ ESP32 ↔ test board
```

시험 matrix:

| fault | software 기대 | hardware 측정 |
|---|---|---|
| UART 단선 | timeout, error log | enable off까지 시간 |
| packet CRC 오류 | 폐기·counter 증가 | motor/LED 안전 상태 |
| ESP32 hang | Pi가 stale command 정지 | watchdog가 enable off |
| Pi process kill | MCU command timeout | enable off |
| e-stop | 모든 계층 우회 | 즉시 power/enable 차단 |

## SW 산출물

```text
docs/embedded/core/05_debug_test_hil.md
.github/workflows/embedded-core.yml
humanoid/firmware/*/test/
```

## 완료 조건

- GDB로 의도한 crash 위치를 source line까지 추적
- logic analyzer로 control jitter와 fault latency 기록
- CRC·packet·state machine unit test가 CI에서 통과
- GitHub Actions에서 ESP32 build 재현
- Pi 5↔ESP32↔test board HIL fault matrix 통과
- CI 결과와 HIL 결과가 문서에서 구분됨
