# Embedded Core PATCH-01: Prerequire — ESP32 보드 연결과 serial 확인

- 목표: ESP32 dev board를 노트북에 연결하고, ESP-IDF가 firmware를 flash·monitor할 수 있는 `/dev/ttyUSB*` 또는 `/dev/ttyACM*` port를 확보
- 선행: ESP32 보드, data 전송 가능한 USB 케이블, ESP-IDF 설치
- 결론: 보드를 연결하지 않은 채 firmware 실습을 시작하지 않는다. 먼저 `dmesg`·`lsusb`·`esptool`로 board와 USB-UART chip이 보이는지 확인한다.

## 1. 어떤 board인지 먼저 확인

보드 silkscreen 또는 구매 페이지에서 다음을 확인한다.

| 항목 | 확인 예 |
|---|---|
| MCU | `ESP32-WROOM-32`, `ESP32-S3-WROOM-1`, `ESP32-C3` |
| USB connector | USB-C 또는 Micro-USB |
| USB-UART chip | `CP2102`, `CH340`, `FTDI`, 없음 |
| 전원 | USB로 전원 공급 여부, 외부 `5V`/`3V3` pin |

ESP32-S3 일부 board는 native USB-JTAG/Serial을 제공해 `/dev/ttyACM0`으로 잡힌다.

## 2. 노트북과 연결

USB data 케이블로 board의 USB port를 노트북에 연결한다.

```bash
sudo dmesg -w
```

연결 중 새 log를 보며 다음을 확인한다.

```text
cp210x converter now attached to ttyUSB0
ch341-uart ... ttyUSB0
cdc_acm ... ttyACM0
```

USB port 목록:

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

charge-only 케이블은 USB data가 없어 port가 나타나지 않는다. 케이블이 data 전송용인지 확인한다.

## 3. USB-UART가 없는 bare module

board에 USB connector가 없으면 3.3 V USB-UART adapter를 쓴다.

```text
ESP32 TX  → USB-UART RX
ESP32 RX  → USB-UART TX
ESP32 GND → USB-UART GND
```

**USB-UART 5 V 출력을 ESP32 3.3 V pin에 직접 연결하지 않는다.** board가 5 V input pin을 제공하면 datasheet를 확인하고 그 pin에만 연결한다.

## 4. serial port 권한

```bash
id
sudo usermod -aG dialout $USER
```

`dialout` group 적용은 재로그인 뒤 확인한다.

```bash
groups | tr ' ' '\n' | grep dialout
```

## 5. flash port 확인

### 5.1 ESP-IDF 설치

```bash
sudo apt install -y git wget flex bison gperf python3 python3-venv \
  python3-pip cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

mkdir -p ~/esp
cd ~/esp
git clone -b v5.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

`esp32`는 대상 chip이다. 보드가 `esp32s3`이면 `./install.sh esp32s3`로 바꾼다.

### 5.2 환경 로드

`export.sh`는 shell 세션마다 한 번 실행한다. 이 파일이 하는 일:

| 설정 | 값 |
|---|---|
| `IDF_PATH` | `~/esp/esp-idf` |
| `PATH` | ESP-IDF python venv와 toolchain 경로 추가 |
| python | ESP-IDF 전용 가상환경 활성화 |

```bash
source ~/esp/esp-idf/export.sh
```

로드 확인:

```bash
idf.py --version
which esptool.py
printenv IDF_PATH
```

`source`는 현재 shell에서만 유효하다. 새 terminal을 열면 다시 실행한다.

### 5.3 chip과 port 확인

```bash
esptool.py chip_id
```

port를 직접 지정할 수 있다:

```bash
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyACM0 monitor
```

## 6. 연결 상태 점검표

| 점검 | 통과 조건 |
|---|---|
| USB 연결 | `dmesg`에 USB-UART attach log |
| port node | `/dev/ttyUSB*` 또는 `/dev/ttyACM*` 존재 |
| chip 확인 | `esptool.py chip_id`가 ESP32 type·MAC 출력 |
| monitor | reset log 또는 `hello_world` 출력 |
| 안전 | board에 손상된 pin·과열·연기 없음 |

## 산출물

```text
docs/embedded/core/00_prerequire.md
docs/embedded/core/00_prerequire_serial.log
```

기록할 값:

```text
board model
USB-UART chip
port path
ESP-IDF version
chip type
MAC address
```

## 완료 조건

- ESP32 board가 USB로 노트북에 연결됨
- `/dev/ttyUSB*` 또는 `/dev/ttyACM*` 확인
- `esptool.py chip_id` 성공
- `idf.py -p <port> monitor`로 reset log 확인
- board·chip·port·IDF version이 문서에 기록됨
