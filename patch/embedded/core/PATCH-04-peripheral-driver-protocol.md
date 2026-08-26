# Embedded Core PATCH-04: UART·I2C·SPI·CAN peripheral driver와 protocol

- 선행 조건: Core PATCH-02·03의 ESP-IDF build와 RTOS task 이해
- 목표: 통신 bus를 register가 아니라 driver로 다루고, frame·timeout·fault를 검증
- 결론: protocol 문서 없이 driver를 쓰면 byte order·timeout·error recovery가 흩어진다. 먼저 frame을 고정한다.

## 개념

| bus | 특징 | ESP32 첫 실습 |
|---|---|---|
| UART | TX/RX 두 선, byte stream | USB-UART loopback |
| I2C | SDA/SCL, address, NACK | OLED 또는 EEPROM |
| SPI | MOSI/MISO/SCLK/CS, 전이중 | loopback 또는 sensor |
| CAN | differential bus, ID·arbitration | MCP2515 module 또는 Pi 5 `vcan` |
| GPIO/ADC/PWM | 단순 I/O·analog·duty | button·potentiometer·LED |

## 1. 공통 packet

```text
magic(2) | sequence(2) | length(1) | payload(N) | crc16(2)
```

규칙:

- byte order는 little-endian으로 고정
- CRC 오류·length 불일치·sequence 누락은 폐기
- timeout 후 안전 상태로 전환
- 모든 field와 error code를 protocol 문서에 기록

CRC는 검증된 library를 사용하고 직접 만든 algorithm을 production처럼 쓰지 않는다.

## 2. UART

ESP32 UART driver를 사용한다.

```c
#include "driver/uart.h"

uart_config_t cfg = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};
uart_driver_install(UART_NUM_1, 2048, 2048, 0, NULL, 0);
uart_param_config(UART_NUM_1, &cfg);
uart_set_pin(UART_NUM_1, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
```

검사:

| case | 기대 |
|---|---|
| TX→RX loopback | 보낸 byte 그대로 수신 |
| wrong CRC | frame 폐기, error counter 증가 |
| length mismatch | frame 폐기 |
| 수신 timeout | stale command가 유지되지 않음 |

## 3. I2C

```c
#include "driver/i2c.h"

i2c_config_t cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA_PIN,
    .scl_io_num = SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
};
i2c_param_config(I2C_NUM_0, &cfg);
i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
```

검사:

- `i2cdetect` 또는 ESP-IDF scan으로 address 확인
- 존재하지 않는 address에 NACK가 발생하는지 확인
- bus hang 뒤 timeout·reinit이 동작하는지 확인

## 4. SPI

```c
#include "driver/spi_master.h"

spi_bus_config_t bus = {
    .mosi_io_num = MOSI_PIN,
    .miso_io_num = MISO_PIN,
    .sclk_io_num = SCLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};
spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
```

첫 검사는 MOSI→MISO loopback. clock polarity·phase를 datasheet와 맞추고 CS가 transaction 동안 low인지 확인한다.

## 5. CAN

ESP32는 CAN controller가 없으므로 MCP2515 SPI module 또는 Pi 5 SocketCAN을 쓴다.

Linux 먼저:

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
candump vcan0
cansend vcan0 123#DEADBEEF
```

실제 CAN module이 있으면 ESP32 SPI로 MCP2515를 제어하거나 Pi 5 CAN HAT를 사용한다. CAN에서는 두 node, termination resistor, ID, timeout을 확인한다.

## 6. GPIO/ADC/PWM safety simulation

button, potentiometer, LED를 motor 대신 사용해 다음을 확인한다.

```text
potentiometer → ADC → normalized command
button → emergency stop
LED → motor enable simulation
```

ADC raw count를 전압·물리량으로 변환할 때 offset과 calibration 계수를 기록한다.

## SW 산출물

```text
humanoid/firmware/lab03_protocol/
docs/embedded/core/03_protocol.md
docs/embedded/core/03_bus_capture/
```

## 완료 조건

- 공통 packet의 CRC·length·sequence test 통과
- UART loopback과 fault 주입 결과 기록
- I2C address scan과 NACK 복구 확인
- SPI loopback과 clock mode 확인
- `vcan0` 또는 실제 CAN에서 두 node 전송
- GPIO/ADC/PWM motor-enable simulation이 fault에서 off
