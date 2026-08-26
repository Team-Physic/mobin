# Embedded Core PATCH-02: ESP32 C/MCU bare-metal 기초

- 선행 조건: ESP32 dev board, USB-UART 케이블, ESP-IDF 설치
- 목표: MCU가 boot부터 GPIO·timer·interrupt·memory·linker까지 어떻게 실행되는지 실제 firmware로 확인
- 결론: vendor 예제를 flash하는 데서 멈추지 않고, `main.c`, map file, register, ISR를 직접 읽고 측정한다.

## 개념

| 개념 | 쉬운 설명 |
|---|---|
| MCU | CPU·flash·RAM·GPIO·timer·UART 등이 한 chip에 있는 제어용 chip |
| bare-metal | Linux 같은 OS 없이 startup code가 hardware를 직접 초기화 |
| register | MCU 주변장치를 제어하는 특정 주소의 bit 집합 |
| GPIO | 범용 입출력 pin |
| timer | 주기마다 event를 만들거나 PWM을 만드는 hardware |
| ISR | interrupt가 발생하면 일반 코드보다 먼저 실행되는 짧은 handler |
| volatile | compiler가 매번 memory에서 다시 읽게 하는 한정자 |
| linker/map file | code·data가 실제 flash/RAM 어느 주소에 배치되는지 보여주는 파일 |

## 장비와 핀 정의

보드마다 LED·button pin이 다르다. 먼저 보드 silkscreen과 datasheet에서 실제 GPIO를 확인한다. 아래는 예시이며 `sdkconfig`와 `menuconfig`에서 바꾼다.

```text
LED_GPIO  = GPIO2
BUTTON_GPIO = GPIO0
PWM_GPIO  = GPIO4
```

## 1. ESP-IDF 개발환경

```bash
idf.py --version
idf.py create-project lab01_blink
cd lab01_blink
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

보드가 ESP32-S3이면 `set-target esp32s3`을 사용한다. flash port는 `ls /dev/ttyUSB* /dev/ttyACM*`로 확인한다.

## 2. GPIO register와 LED

`main.c`에서 LED를 켜고 끈다.

```c
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_2

void app_main(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

통과 조건: LED가 0.5초 주기로 깜빡이고 `idf.py monitor` log가 정상 출력된다.

## 3. button과 pull-up

```c
#define BTN_GPIO GPIO_NUM_0

gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
gpio_set_pull_mode(BTN_GPIO, GPIO_PULLUP_ONLY);

while (1) {
    int level = gpio_get_level(BTN_GPIO);
    gpio_set_level(LED_GPIO, !level);
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

button을 누르면 LED가 켜지고, 뗐을 때 bounce로 LED가 순간적으로 흔들리는지 확인한다.

## 4. timer와 PWM

ESP-IDF `ledc` driver로 PWM duty를 바꾼다.

```c
#include "driver/ledc.h"

ledc_timer_config_t timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK,
};
ledc_timer_config(&timer);

ledc_channel_config_t ch = {
    .gpio_num = 4,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
};
ledc_channel_config(&ch);

for (int duty = 0; duty < 1023; duty += 64) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(50));
}
```

oscilloscope 또는 logic analyzer가 있으면 `freq_hz`와 duty를 측정한다.

## 5. GPIO interrupt와 volatile flag

```c
#include "freertos/queue.h"

static QueueHandle_t gpio_evt_queue;

static void IRAM_ATTR btn_isr(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void app_main(void) {
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_set_intr_type(BTN_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_GPIO, btn_isr, (void *)BTN_GPIO);

    uint32_t gpio_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            gpio_set_level(LED_GPIO, !gpio_get_level(LED_GPIO));
        }
    }
}
```

ISR에서 `printf`, delay, 긴 함수를 호출하지 않는다. queue를 통해 main task로 event를 넘긴다.

## 6. memory map과 linker

```bash
idf.py build
ls build/*.map
nm -C build/lab01_blink.elf | grep app_main
idf.py size
```

map file에서 다음을 찾는다.

| symbol | 어느 section | 의미 |
|---|---|---|
| `app_main` | `.flash.text` 또는 `.iram0.text` | 실행 code 위치 |
| 전역 변수 | `.dram0.data`/`.bss` | 초기화·비초기화 RAM |
| static 변수 | `.data`/`.bss` | file scope 유지 |

`volatile` 변수를 일반 변수로 바꿨을 때 compiler 최적화가 어떻게 바뀌는지 map/disassembly로 비교한다.

## 7. C memory trap 재현

다음을 의도적으로 만들고 원인을 기록한다.

| bug | 예 | 예상 증상 |
|---|---|---|
| uninitialized pointer | `int *p; *p = 1;` | crash 또는 임의 주소 write |
| stack overflow | 깊은 recursion | reboot 또는 corrupt |
| missing `volatile` | ISR flag를 일반 변수로 | flag가 update되지 않음 |
| signed/unsigned | `uint8_t` loop overflow | 무한 loop 또는 잘못된 범위 |
| alignment | `char buf[4]; int *p=(int*)buf;` | unaligned access 문제 |

각 bug는 `idf.py monitor`의 backtrace와 disassembly를 함께 남긴다.

## SW 산출물

```text
humanoid/firmware/lab01_blink/
docs/embedded/core/01_c_mcu.md
docs/embedded/core/01_c_mcu_bugs.md
```

## 완료 조건

- `hello_world`와 blink binary flash 성공
- button→LED 제어와 PWM duty 변화 기록
- GPIO interrupt가 queue로 event를 전달
- map file에서 `app_main`과 변수 section 주소 설명
- 5개 C memory bug를 재현하고 backtrace 저장
- board model, GPIO pin, ESP-IDF commit 기록
