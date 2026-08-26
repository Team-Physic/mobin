# Embedded Core PATCH-08: 제품 판매 수준 제품화·양산

- 선행 조건: Core PATCH-07 safety chain, 프로젝트 적용 prototype 완료
- 목표: 동작하는 prototype을 실제로 판매할 수 있는 제품으로 만드는 gate
- 결론: 기능 동작과 제품 판매는 다르다. DFM·인증·양산 test·reliability·문서·비용까지 통과해야 한다.

## 제품 판매에 필요한 gate

| gate | 질문 | 통과 증거 |
|---|---|---|
| spec | 누구에게, 몇 대, 어떤 환경, 가격 | requirement·acceptance criteria |
| DFM/DFT | 공장에서 조립·검사 가능한가 | BOM, test point, panel, 조립 순서 |
| firmware release | 안전하게 업데이트·롤백 가능한가 | bootloader, OTA, version, rollback |
| certification | 국내/해외 판매 허용 | KC·CE·FCC·RoHS 체크리스트 |
| manufacturing test | 대량 생산에서 불량을 골라내는가 | ICT/FCT·calibration·burn-in |
| reliability | 실제 사용 환경에서 고장나지 않는가 | 온습도·진동·배터리 cycle 결과 |
| quality | bug와 release가 추적되는가 | bug triage, changelog, beta field |
| service | 고객이 혼자 쓸 수 있고 고장 시 복구 가능한가 | 사용자 매뉴얼, service guide |

## 1. 제품 spec 정의

```text
target user  : 교육용/실내 로봇 사용자
sales volume : 100대 파일럿
environment  : 실내 0~40도, 상대습도 10~90%
price target : BOM + 조립 + test + A/S 마진
MTBF target  : 5000시간
```

판매 대상과 volume을 먼저 정한다. volume에 따라 PCB 공정, 인증 범위, test 자동화 수준이 달라진다.

## 2. Hardware DFM·DFT

| 항목 | 제품 기준 |
|---|---|
| PCB | test point, fiducial, panelization, edge clearance |
| BOM | vendor·MPN·lead time·alternate part |
| 조립 | 커넥터 실수 방지, cable 길이·고정 |
| DFM | 부품 방향, solder mask, thermal pad |
| DFT | JTAG/SWD, UART, 전원 rail test point |

prototype에서 jumper·빵판·임시 배선은 제거하고 제조 가능한 설계로 바꾼다.

## 3. Firmware production release

```c
// 필수 요소
bootloader
secure boot
OTA / firmware update
version + rollback
watchdog + brownout
flash wear
fault log / serial number
```

release마다 다음을 남긴다:

```text
firmware version
hardware revision
commit hash
flash 방법 / OTA 절차
rollback 절차
```

## 4. 인증

국내 판매는 KC가 기본이다. 무선·전기·배터리·환경 규제를 분리해 확인한다.

| 규제 | 대상 | 확인 항목 |
|---|---|---|
| KC 전파 | Wi-Fi/BLE | 전파인증, 시험성적서 |
| KC 전기안전 | 전원/충전기 | 절연, 온도, 보호회로 |
| 배터리 | Li-ion | KC 안전확인, 운송 |
| RoHS | 납 등 유해물질 | 성분 확인 |
| CE/FCC | 해외 판매 | EMC·RF 시험 |

EMC·ESD는 설계 단계에서부터 잡는다:

```text
ESD: 커넥터 입력 보호, GND 경로
EMC: decoupling, shielding, layout
```

## 5. Manufacturing test

```text
ICT  : 전원 rail, short, 주요 net
FCT  : 기능 test, sensor·motor·통신
calibration : IMU·모터·센서 초기값
burn-in : 연속 구동, reboot loop
```

각 board에 serial number를 붙이고 test 결과를 저장한다.

## 6. Reliability test

| test | 조건 | 통과 기준 |
|---|---|---|
| 온습도 | 0~40도, 10~90%RH | 동작·복구 확인 |
| 진동/낙하 | 포장 상태 낙하, 진동 | 기능 저하 없음 |
| 배터리 cycle | 충·방전 반복 | 용량·온도 한계 기록 |
| 장기 구동 | 72시간 연속 | reboot·fault 없음 |

실측 조건·시간·고장 위치를 기록한다.

## 7. Quality·release·service

```text
bug triage
release note / changelog
beta field test
사용자 매뉴얼
service guide / FAQ
```

고객 문의·불량을 분류해 firmware·hardware·사용 오류로 나누고, 재발 시 어느 gate를 보강할지 기록한다.

## 8. Cost·supply

| 항목 | 기록 |
|---|---|
| BOM cost | 부품별 단가·합계 |
| 조립·test cost | 공정 시간·불량률 |
| lead time | 주요 부품 입고 기간 |
| EOL/alternate | 단종 대체 부품 |

## 완료 조건

- requirement·acceptance criteria 문서
- DFM/DFT 반영 hardware revision
- firmware release·OTA·rollback 절차
- KC/CE/FCC/RoHS 체크리스트
- ICT/FCT·calibration·burn-in 결과
- reliability test 보고서
- bug triage·changelog·beta field 결과
- 사용자 매뉴얼·service guide

## 산출물

```text
docs/embedded/core/08_product_engineering.md
docs/embedded/core/08_certification.md
docs/embedded/core/08_manufacturing_test.md
docs/embedded/core/08_reliability.md
```
