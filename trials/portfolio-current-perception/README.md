# Gazebo·RViz 포트폴리오 증거

2026-09-07. 실제 구현으로 생성한 데이터와 renderer 출력이다. 이미지 내용·track 상태를 수작업 또는 생성형 이미지 도구로 수정하지 않는다. Renderer에서는 ID 가독성을 위해 텍스트의 showOnTop과 글자 높이의 0.35배 공백만 사용한다. Label 내용·위치는 원본 그대로다. 상세 색상 범례와 수록 원고는 [portfolio-selected-content.md](../../docs/portfolio-selected-content.md)에 있다.

## 수록 이미지 선택 기준

최종 수록 묶음은 `synchronized/`다. **359.800초, confirmed ID 98·99**, 12개 topic의 timestamp 완전 일치를 검사했다. 동일 입력의 실제 fusion 재계산에서도 annotation 차이 0 pixel·3D 관측 차이 0m였다. 다음 조건을 모두 확인한 뒤 선택한다.

- Gazebo 카메라 원본, segmentation label, Detection2D, LiDAR 원본, fusion 출력, track, marker의 **정수 ns timestamp 완전 일치**.
- 서로 다른 두 track이 실제 `CONFIRMED`이고, 각 마지막 관측 시각도 선택 시각과 일치한다. double→ns 변환의 반올림 오차 1ns만 허용한다.
- 두 track은 서로 다른 Detection3D와 각각 0.75m 이내에서 대응하며, 영상에도 사람 Detection2D가 둘 이상 있다.
- PointCloud·Marker·TF를 같은 시각에 고정 재생한 격리 ROS domain에서 RViz 렌더러로 PNG를 직접 저장한다. 반복 재생 중 tracker를 실행하거나 confirmation 횟수를 인위적으로 늘리지 않는다.
- 이미지와 3D를 눈으로 검사해 두 사람이 보이고 흰색 ID·confirmed 문구가 읽히는지 확인한다.

Gazebo 그림은 **시뮬레이터가 생성한 로봇 카메라 원본**이다. 데스크톱 Gazebo GUI의 다른 시각이나 외부 관찰 카메라 영상을 같은 시각인 것처럼 대체하지 않는다. CameraInfo는 최신 고정 calibration이고, TF는 선택 시각의 `odom <- sensor` 조회 결과다.

`snapshot.json`에 timestamp, confirmed ID, CDR topic/type 목록, 대기 시간을 기록한다. `*.cdr`은 ROS serialization 원본이다. `gazebo-camera.png`, `camera-association.png`, `rviz.png`를 한 묶음으로 사용한다. 날짜·시각만 같거나 wall-clock에 동시에 캡처했다는 이유로 동시각 데이터라고 판정하지 않는다.

## 실행 환경

- Container: `mobin-portfolio-capture`, image `tb3-jazzy-lab:local`, ROS 2 Jazzy·Gazebo Harmonic.
- Gazebo server는 `-s`로 실행. 최종 RViz export는 Xvfb `:98`의 가상 화면에서 실행해 데스크톱 창을 띄우지 않는다.
- 실험 ROS domain `87`, 표시 전용 frozen replay domain `88`, `GZ_PARTITION=mobin-portfolio-capture`.
- 모델 `waffle_pi_3d_large`, warehouse, 보행자 3명·seed 42, 시작 pose `(-2,-0.5,0)`.
- 로봇 이동 명령 없음. Navigation·patrol 비활성, perception·safety 활성.
- 최초 평가 구간은 Camera 30Hz·LiDAR 10Hz. 동시각 그림 수집을 위해 평가 종료 후 Camera 10Hz인 임시 SDF로 로봇을 원래 pose에 다시 생성했다. 실행 중 rate 변경만으로는 기존 8ms phase 차이가 남아 센서를 함께 초기화했다. 해당 그림은 별도 촬영 조건이며 초기 성능 수치와 결합하지 않는다.
- 사전학습 `yolo11s-seg.pt`, confidence 0.25, GPU device 0. GPU NVIDIA GTX 1050 4096MiB, driver 580.173.02.
- RViz/rviz_rendering 14.1.22. 렌더링은 표시용이며 추론 결과를 변경하지 않는다.
- 실제 fusion 설정은 [fusion-runtime.yaml](fusion-runtime.yaml)에 보존한다. Node의 `use_sim_time=false`와 별개로 fusion·tracker 계산은 sensor message timestamp를 사용한다. 표시 전용 RViz는 `use_sim_time=true`와 고정 `/clock`을 사용한다.

## 재현 절차

아래 경로는 container 내부 기준이다. 이미 실행된 평가의 파일을 덮어쓰지 않도록 새 증거 디렉터리를 사용한다. Source overlay와 모델은 먼저 빌드·설치된 환경이어야 한다. 현재 기록은 [gazebo.log](gazebo.log), [summary.json](summary.json)에 보존되어 있다.

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=mobin-portfolio-capture
export TURTLEBOT3_MODEL=waffle_pi_3d_large
ros2 launch mobile_robot_lab_python warehouse_patrol.launch.py \
  scenario:=multi_person pedestrian_count:=3 pedestrian_seed:=42 \
  run_navigation:=false run_patrol:=false run_evaluator:=true \
  run_perception:=true run_safety:=true gazebo_gui:=false launch_rviz:=false \
  publish_annotated_image:=true device:=0 confidence:=0.25 \
  evaluation_output_path:=/tmp/new-portfolio-run/summary.json \
  evaluation_duration_seconds:=60 evaluation_expected_person_count:=3 \
  x_pose:=-2.0 y_pose:=-0.5 yaw:=0.0
```

다른 terminal에서 동일 domain을 설정하고 수집한다. `capture`는 최대 wall 600초 대기 후 조건을 충족하지 못하면 실패로 종료한다. 이를 성공 사진으로 대체하지 않는다.

```bash
python3 /ws/trials/portfolio-current-perception/snapshot.py capture /tmp/new-portfolio-run/synchronized
python3 /ws/trials/portfolio-current-perception/snapshot.py verify /tmp/new-portfolio-run/synchronized
```

기존 보존 묶음의 재생은 격리 domain에서 실행한다. 원래 추적기는 이 domain에 실행하지 않는다. 별도 fusion 재계산 검사는 모든 출력 topic을 `/verification/*`로 remap해 원본 출력과 섞이지 않게 수행했다.

```bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
python3 /ws/trials/portfolio-current-perception/snapshot.py replay /ws/trials/portfolio-current-perception/synchronized
```

빌드 산출물은 `/tmp`에 생성한다. Xvfb는 아직 사용 중이지 않은 display 번호를 사용한다. 아래 `:98`은 이번 작업의 전용 번호다. 다른 작업이 사용하는 display에 연결하지 않는다.

```bash
cmake -S /ws/trials/portfolio-current-perception -B /tmp/portfolio-render-build
cmake --build /tmp/portfolio-render-build -j2
Xvfb :98 -screen 0 1920x1080x24 -nolisten tcp
```

다른 terminal에서 renderer를 실행한다. 출력 파일이 이미 있으면 추출기가 거부한다. 저장은 RViz 초기화 후 wall 15초에 수행되므로, 먼저 replay를 시작한다.

```bash
export DISPLAY=:98 ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa __NV_PRIME_RENDER_OFFLOAD=0
/tmp/portfolio-render-build/export_render /tmp/new-rviz.png \
  -d /ws/trials/portfolio-current-perception/synchronized/perception.rviz \
  --ros-args -p use_sim_time:=true
```

## 자료별 의미

| 자료 | 의미 | 사용 범위 |
|---|---|---|
| `synchronized/` | 같은 시각의 두 confirmed track과 headless 표시 | 최종 그림 |
| `capture.json`, `frame-00..08-*.png` | 앞서 수집한 9쌍 카메라 원본·annotation과 track 시간 기록 | 부분 관측·두 사람 예시의 보조 자료. 이전 RViz 그림과 동시각이라고 연결하지 않음 |
| `summary.json`, `roi-associations.csv`, `id-switch-events.json` | 이 실행의 첫 60 simulation초 평가 | TP310/FN20/FP64/IDSW15/IDF1 24.43%. 정지 로봇의 단일 실행이며 예전 고정 입력 비교와 개선율 비교 불가 |
| `metrics-table.json` | TP·FP·FN과 계산된 Precision·Recall·F1, 기존 ID switch·IDF1 | TN은 음성 객체 모집단이 없어 null. 원본 JSON 해시와 집계 단위 보존 |
| `fusion-runtime.yaml`, `environment.json` | 실행 설정·source와 모델·결과 해시 | 재현과 provenance |
| `export_render.cpp`, `CMakeLists.txt` | 기존 RViz renderer API를 호출하는 표시 도구 | production perception·tracking 로직 변경 없음 |
| `snapshot.py` | 동일 시각 선택·원본 serialization·고정 재생·계약 검사 | track 상태를 새로 만들거나 timestamp를 수정하지 않음 |

그림 선택은 포트폴리오 설명용 조건을 만족하는 순간을 고르는 것이다. 전체 실행의 성공률을 높게 보이도록 평가 구간을 바꾼 것이 아니며, 선택한 순간은 첫 60초 평가 구간 밖일 수도 있다. 정확한 시각과 평가 구간을 구분해 읽는다.

## 동시각 보존 검증 결과

[association-replay-check.json](synchronized/association-replay-check.json)은 저장된 원본 입력과 동일 production fusion 바이너리의 재계산 결과를 대조한다. 결과: timestamp=359800000000ns, annotation 차이=0 pixel, 3D detection 2개·위치 최대 절대차=0m. [render-readable.log](synchronized/render-readable.log)의 `readable track labels: 2`와 실제 PNG에서 두 ID·confirmed 가독성을 확인했다.

카메라 영상에서 두 인물이 보이지만 전신이 모두 들어온 장면은 아니다. 가까운 사람은 상체, 오른쪽 사람은 신체 일부가 영상 경계 밖이다. 부분 관측에서도 해당 순간 두 track이 confirmed였다는 예시이며, 모든 부분 가림 조건의 강건성 검증으로 일반화하지 않는다.

## 촬영 도구의 함수 변경

Production perception·tracking 함수는 이 자료 정리 단계에서 변경하지 않았다.

| 파일 위치 | 함수 또는 설정 | 변경 요약 |
|---|---|---|
| `trials/portfolio-current-perception/snapshot.py` | [stamp()](snapshot.py#L39) | 원본 header를 정수 ns로 읽어 부동소수점 시각 비교를 피함. MarkerArray도 동일 기준 사용 |
| `trials/portfolio-current-perception/snapshot.py` | [confirmed_pair()](snapshot.py#L45) | 같은 시각에 갱신된 두 confirmed track과 서로 다른 3D 관측의 거리 조건 검사 |
| `trials/portfolio-current-perception/snapshot.py` | [capture()](snapshot.py#L58) | 12개 topic의 완전 일치 시각을 선택하고 원본 CDR·PNG·TF·조건을 보존 |
| `trials/portfolio-current-perception/snapshot.py` | [replay()](snapshot.py#L136) | 저장 message를 고정 시각으로 반복 발행. Tracker를 재실행해 confirmation을 만드는 경로는 없음 |
| `trials/portfolio-current-perception/snapshot.py` | [verify()](snapshot.py#L150) | 보존된 timestamp·상태·3D 대응·TF·PNG 크기 계약을 assert로 검사 |
| `trials/portfolio-current-perception/snapshot.py` | [main()](snapshot.py#L169) | capture/replay/verify 모드 선택과 ROS node lifecycle 관리 |
| `trials/portfolio-current-perception/export_render.cpp` | [format_labels()](export_render.cpp#L22) | Track text의 showOnTop과 공백 폭만 조정해 점에 가려진 ID와 과도한 단어 간격 개선 |
| `trials/portfolio-current-perception/export_render.cpp` | [main()](export_render.cpp#L39) | Qt·ROS 인자를 분리하고 RViz 초기화 후 native renderer 결과만 PNG로 저장. 기존 출력 덮어쓰기 거부 |
| `trials/portfolio-current-perception/capture_window.py` | `main()` — **삭제됨** | 데스크톱 창을 올려 화면 픽셀을 캡처하던 경로를 제거. Camera는 ROS Image 저장, 3D는 native renderer 직접 저장으로 대체 |

## 10Hz 촬영용 모델 재현

전용 촬영 simulation에서만 수행한다. 원본 설치 모델을 수정하지 않고 임시 파일에 Camera 주기 한 항목만 바꾼다. 아래 서비스는 반드시 촬영용 `GZ_PARTITION`을 설정한 shell에서 호출한다.

```bash
python3 - <<'PY'
from pathlib import Path
source = Path('/ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi_3d_large/model.sdf')
text = source.read_text()
assert text.count('<update_rate>30</update_rate>') == 1
Path('/tmp/portfolio-camera-10hz.sdf').write_text(text.replace('<update_rate>30</update_rate>', '<update_rate>10</update_rate>'))
PY
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: true'
gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 5000 --req 'name: "waffle_pi_3d_large" type: MODEL'
ros2 run ros_gz_sim create -world default -name waffle_pi_3d_large -file /tmp/portfolio-camera-10hz.sdf -x -2.0 -y -0.5 -Y 0.0 -z 0.01
```

위 서비스와 spawn이 성공했는지 확인한 뒤 별도 terminal에서 `snapshot.py capture`를 시작하고 `pause: false`로 재개한다. 이 자료에서는 이후 139.71 wall초에 조건을 충족했다. 사람의 위치·track ID를 미리 지정한 장면이 아니므로 다음 실행의 선택 시각과 ID는 같다고 보장하지 않는다. **보존 묶음의 고정 재생**은 같은 공간 데이터를 재현한다.

## 최종 포트폴리오 반영 및 정리

실제 `ai-job-search/cv/ppt/presentation.tex`와 `presentation_tex.pdf`에 반영했다. 전체 18쪽 중 10쪽 개인 기여를 보강하고, 13쪽 동시각 시각화·범례, 14쪽 원인·수정, 15쪽 평가 표·지표 개념·한계를 추가했다. 원본 PNG·평가 JSON은 `assets/mobin`에도 복사했다. [빌드 확인 기록](portfolio-build-check.json)에 출력 해시를 보존한다.

촬영 전용 Xvfb 종료, X11 root 접근 허용 원복, 자동 삭제되는 `mobin-portfolio-capture` 컨테이너 종료를 완료했다. 공유 build/install 볼륨과 다른 작업 컨테이너는 정리 대상에서 제외했다. 위 simulation·render 빌드의 임시 파일은 전용 컨테이너 삭제로 함께 정리했다.
