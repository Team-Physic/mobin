# Embedded Core PATCH-05: Raspberry Pi 5 Embedded Linux·BSP·bus

- 선행 조건: Raspberry Pi 5, Linux, ESP32 firmware build 이해
- 목표: Linux가 GPIO·UART·I2C·SPI·CAN을 어떻게 노출하는지 device tree·`/dev`·systemd로 확인
- 결론: Pi 5는 legacy `RPi.GPIO`가 아니라 RP1과 `libgpiod`로 GPIO를 다룬다.

## 개념

| 개념 | 쉬운 설명 |
|---|---|
| SoC | CPU와 주요 주변장치가 한 chip에 있는 Linux용 processor |
| RP1 | Pi 5에서 GPIO·UART·SPI·I2C 등을 관리하는 peripheral controller |
| device tree | hardware 구성을 kernel에 설명하는 data |
| overlay | device tree 일부를 boot 시 추가·변경 |
| pinctrl | pin이 어떤 기능(UART/I2C/SPI/GPIO)을 할지 정하는 계층 |
| `/dev/gpiochip*` | userspace가 GPIO chip에 접근하는 node |
| SocketCAN | Linux에서 CAN interface를 network처럼 쓰는 API |
| systemd service | 부팅 시 node·script를 자동 시작 |
| cross-compile | host에서 target board용 binary를 build |

## 1. Pi 5 GPIO 확인

```bash
cat /proc/device-tree/model
gpiodetect
gpioinfo | head -60
ls -l /dev/gpiochip*
```

`RPi.GPIO`는 Pi 5와 호환되지 않는다. `gpiod`/`libgpiod`를 사용한다.

```bash
gpioset gpiochip4 17=1
gpioget gpiochip4 17
```

chip 번호와 line 번호는 `gpioinfo`로 확인한다. button/LED를 연결하고 입력·출력을 검사한다.

## 2. device tree overlay

사용 가능한 overlay를 확인한다.

```bash
dtoverlay -a | grep -E 'uart|i2c|spi'
```

`/boot/firmware/config.txt`에 필요한 overlay를 추가한다.

```text
dtoverlay=uart1-pi5
dtoverlay=i2c1-pi5
dtoverlay=spi0-1cs
```

재부팅 뒤 node를 확인한다.

```bash
ls -l /dev/ttyAMA* /dev/ttyS* /dev/i2c-* /dev/spidev* 2>/dev/null
```

## 3. UART·I2C·SPI loopback

| bus | loopback 방법 | 확인 |
|---|---|---|
| UART | TX→RX 점퍼 | `minicom` 또는 Python serial echo |
| I2C | 실제 sensor/EEPROM | `i2cdetect -y 1` |
| SPI | MOSI→MISO 점퍼 | `spidev_test` echo |

```bash
i2cdetect -y 1
```

UART는 baud·parity·stop bit를 ESP32와 같게 맞추고 packet을 주고받는다.

## 4. SocketCAN

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip -details link show vcan0
candump vcan0
```

다른 terminal에서:

```bash
cansend vcan0 123#1122334455667788
```

실제 CAN HAT가 있으면 `can0` interface로 바꾸고 termination resistor를 확인한다.

## 5. systemd service

```bash
sudo tee /etc/systemd/system/mobin-uart.service >/dev/null <<'EOF'
[Unit]
Description=Mobin UART bridge
After=multi-user.target

[Service]
ExecStart=/usr/bin/python3 /opt/mobin/uart_bridge.py
Restart=on-failure
User=swlinux

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now mobin-uart.service
systemctl status mobin-uart.service
```

재부팅 뒤 service가 자동 시작되고 device node가 다시 나타나는지 확인한다.

## 6. ESP32 cross-compile

ESP32 binary는 ESP-IDF가 host에서 build하므로 별도 cross toolchain 설치가 필요 없다.

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

Pi 5에서도 같은 ESP-IDF project를 build할 수 있지만, 실습은 host에서 build하고 flash한다. build log에는 IDF version과 commit을 남긴다.

## SW 산출물

```text
docs/embedded/core/04_pi5_bsp.md
config/pi5/overlays.txt
code/scripts/pi5/uart_bridge.py
```

## 완료 조건

- `gpiodetect`/`gpioinfo`에서 GPIO chip·line 확인
- button/LED를 `gpioset`/`gpioget`으로 제어
- UART·I2C·SPI overlay 적용 후 `/dev` node 생성
- UART·SPI loopback 성공, I2C address 확인
- `vcan0` 또는 `can0`에서 CAN frame 전송
- systemd service가 재부팅 뒤 복원
- Pi 5 model·OS version·overlay 설정 기록
