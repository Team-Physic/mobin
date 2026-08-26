# Embedded Core PATCH-03: ESP32 FreeRTOS task·timing·watchdog

- 선행 조건: Core PATCH-02 GPIO·interrupt·ESP-IDF build 완료
- 목표: RTOS task·queue·mutex·timer·watchdog뿐 아니라 scheduler·context switch·priority inversion·ISR-safe·stack overflow까지 실제 측정값으로 확인
- 결론: `vTaskDelay`가 정확한 deadline을 보장하지 않는다. priority와 blocking 경로를 정하고 GPIO toggle·logic analyzer로 측정한다.

## RTOS를 중점 실습하는 이유

임베디드 공고에서 `RTOS, FreeRTOS, task/scheduling`이 반복 등장한다. firmware 개발자는 main loop만 돌리는 코드와 달리, 여러 task가 언제 실행되고 언제 멈추는지, ISR과 task 사이 data를 어떻게 넘기는지, deadline을 놓쳤을 때 어떻게 안전하게 멈추는지를 정해야 한다. 이 PATCH가 Embedded core에서 가장 큰 비중을 차지한다.

## 개념

| 개념 | 쉬운 설명 |
|---|---|
| task | scheduler가 실행 순서를 관리하는 작업 단위 |
| priority | 숫자가 높은 task가 먼저 실행될 우선순위 |
| preemption | 높은 priority task가 낮은 task를 중단하고 실행 |
| scheduler | ready task 중 가장 높은 priority를 고르는 부분 |
| context switch | CPU register·stack을 저장하고 다른 task로 전환 |
| tick | RTOS 시간의 기본 단위, `configTICK_RATE_HZ` 기준 |
| idle task | ready task가 없을 때 돌아가는 최저 priority task |
| blocking | queue·mutex·delay를 기다리며 CPU를 내놓는 상태 |
| ISR context | interrupt 안에서 실행, blocking API 사용 불가 |
| queue | task·ISR 사이 data를 안전하게 전달 |
| mutex | 공유 자원을 동시에 바꾸지 못하게 보호 |
| semaphore | event·허가 수를 세고 task를 깨움 |
| software timer | RTOS tick 기준 callback |
| watchdog | task hang이나 command timeout을 감지해 안전 동작 |
| jitter | 실제 실행 시각이 목표 주기에서 흔들리는 정도 |

## task table

| task | 주기 | priority | 하는 일 |
|---|---:|---:|---|
| cmd_rx | event | 8 | UART packet 수신·검증 |
| control | 5 ms | 7 | enable 갱신과 상태 전이 |
| telemetry | 100 ms | 5 | 상태 log 전송 |
| led | 250 ms | 3 | alive LED |
| watchdog | 10 ms | 9 | command timeout 감지 |

우선순위는 숫자로만 정하지 않는다. 각 task가 blocking되는 API와 최악 실행 시간을 표로 남긴다.

task priority를 정할 때 `period`, `deadline`, `worst-case execution time`을 함께 기록한다. 예:

```text
control: period=5ms deadline=5ms wcet=0.3ms -> priority 7
cmd_rx : event deadline=10ms wcet=0.2ms -> priority 8
```

## 1. task 생성

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void control_task(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        gpio_set_level(GPIO_NUM_4, 1);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
        gpio_set_level(GPIO_NUM_4, 0);
    }
}

void app_main(void) {
    xTaskCreate(control_task, "control", 4096, NULL, 7, NULL);
    xTaskCreate(led_task, "led", 2048, NULL, 3, NULL);
}
```

`vTaskDelayUntil`을 사용하면 고정 주기에 더 가깝다. `vTaskDelay(5)`는 실행 시간만큼 늦어진다.

task를 삭제하거나 stack을 너무 작게 잡으면 panic이 난다. stack size는 `uxTaskGetStackHighWaterMark()`로 남은 여유를 확인해 정한다.

## 2. queue

```c
#include "freertos/queue.h"

QueueHandle_t q = xQueueCreate(16, sizeof(uint32_t));

// ISR
xQueueSendFromISR(q, &value, NULL);

// task
if (xQueueReceive(q, &value, pdMS_TO_TICKS(20)) == pdTRUE) {
    handle(value);
}
```

queue full·timeout·ISR context를 각각 test한다. ISR에서는 block time `0`만 쓴다.

### task notification

한 task를 깨우는 event 하나라면 queue보다 `xTaskNotify`가 가볍다.

```c
// ISR
vTaskNotifyGiveFromISR(task_handle, NULL);

// task
ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
```

queue는 여러 data를 보낼 때, notification은 단일 event일 때 사용한다.

## 3. mutex와 semaphore

```c
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

if (xSemaphoreTake(mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    shared_counter++;
    xSemaphoreGive(mutex);
}

SemaphoreHandle_t evt = xSemaphoreCreateBinary();
xSemaphoreGive(evt);
if (xSemaphoreTake(evt, pdMS_TO_TICKS(10)) == pdTRUE) { ... }
```

통과 조건:

- mutex 없이 공유 counter를 여러 task가 증가시키면 예상보다 작은 값이 재현된다.
- mutex 적용 뒤 race가 사라진다.
- semaphore로 task를 깨우고 timeout을 확인한다.

### priority inversion 재현

낮은 priority task가 mutex를 쥐고, 높은 priority task가 같은 mutex를 기다릴 때 중간 priority task가 계속 실행되면 높은 task가 오래 막힌다. ESP-IDF mutex는 priority inheritance가 기본이라 낮은 task가 잠시 높은 priority로 올라가 inversion이 줄어든다.

```c
// priority inheritance 확인
SemaphoreHandle_t m = xSemaphoreCreateMutex();
```

GPIO toggle로 높은 task의 대기 시간을 측정해 inheritance 적용 전·후를 비교한다.

### event group

여러 flag를 동시에 기다릴 때 사용한다.

```c
EventGroupHandle_t ev = xEventGroupCreate();
xEventGroupSetBits(ev, BIT0 | BIT1);
xEventGroupWaitBits(ev, BIT0 | BIT1, pdFALSE, pdTRUE, pdMS_TO_TICKS(50));
```

`pdTRUE`는 기다린 flag를 자동으로 clear한다.

## 4. software timer

```c
#include "freertos/timers.h"

TimerHandle_t t = xTimerCreate(
    "led", pdMS_TO_TICKS(100), pdTRUE, NULL, timer_cb);
xTimerStart(t, 0);
```

callback은 가볍게 유지하고 긴 일은 task에 넘긴다.

## 5. deadlock과 ISR-safe API

두 task가 서로의 mutex를 기다리면 deadlock이 난다. 아래 순서를 test로 재현하고 해결한다.

```text
task A: take M1 -> delay -> take M2
task B: take M2 -> delay -> take M1
```

해결은 모든 경로에서 같은 lock 순서를 쓰거나, 한 번에 하나의 lock만 쥔다.

ISR 안에서는 blocking API 금지다. `xQueueSendFromISR`, `xTaskNotifyFromISR`, `xSemaphoreGiveFromISR`처럼 `FromISR` 계열만 사용하고 block time을 `0`으로 둔다.

## 6. watchdog

ESP-IDF의 `esp_task_wdt`를 사용한다.

```c
#include "esp_task_wdt.h"

esp_task_wdt_config_t cfg = {
    .timeout_ms = 2000,
    .idle_core_mask = 0,
    .trigger_panic = true,
};
esp_task_wdt_init(&cfg);
esp_task_wdt_add(NULL);

while (1) {
    if (fresh_command) {
        esp_task_wdt_reset();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

Pi process가 command를 보내지 않으면 `fresh_command`가 false가 되고 watchdog이 reboot 또는 enable-off를 수행해야 한다.

## 7. stack overflow와 memory

```c
#include "esp_system.h"

// task 생성 뒤 남은 stack 확인
UBaseType_t uxTaskGetStackHighWaterMark(NULL);
```

통과 조건:

- control task stack을 의도적으로 작게 잡아 overflow를 재현한다.
- overflow panic log에서 어느 task가 어디서 넘쳤는지 확인한다.
- 정상 stack에서 high water mark가 일정 이상 남는지 기록한다.

## 8. scheduler 설정

```c
// sdkconfig
CONFIG_FREERTOS_HZ=1000
CONFIG_FREERTOS_USE_TIME_SLICING=y
```

같은 priority task가 있으면 time slicing으로 번갈아 실행된다. tick rate를 바꾸면 `pdMS_TO_TICKS`의 분해능이 바뀐다. 변경 뒤 timing을 다시 측정한다.

## 9. timing 측정

control task 시작 직전에 GPIO를 high, 끝나면 low로 toggle한다.

```text
logic analyzer/oscilloscope → GPIO4
목표 주기: 5 ms
기록: p50, p95, max jitter, deadline miss
```

`idf.py monitor` timestamp만으로 real-time을 판정하지 않는다. 반드시 GPIO toggle을 hardware로 측정한다.

측정 표:

```text
목표 주기: 5 ms
p50      : 5.02 ms
p95      : 5.11 ms
max      : 5.43 ms
deadline miss: 0
```

## SW 산출물

```text
humanoid/firmware/lab02_rtos/
docs/embedded/core/02_rtos_timing.md
docs/embedded/core/02_fault_cases.md
```

## 완료 조건

- task 5개가 지정 priority로 실행
- queue full·timeout·ISR 전달 test 통과
- mutex 미적용 race가 재현되고 적용 후 해결
- priority inversion이 GPIO toggle로 확인됨
- deadlock이 재현되고 lock 순서로 해결됨
- stack overflow panic log 확인
- watchdog이 command timeout에서 reboot 또는 enable-off
- GPIO toggle로 p50·p95·max jitter 기록
- task blocking 경로와 최악 실행 시간 표 기록
