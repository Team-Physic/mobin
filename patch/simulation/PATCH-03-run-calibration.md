# Simulation PATCH-03: Extrinsic 계산 실행 절차

## 이 PATCH에서 만드는 것

Simulation PATCH-02에서 기록한 5개 bag을 로컬 calibration fork로 build한 Jazzy 이미지로 처리해 `T_lidar_camera`를 계산한다.

```text
data/bags/pose-01..05
        │ preprocess
        ▼
data/results/calib.json + image/PLY 중간 결과
        │ initial_guess_manual
        ▼
results.init_T_lidar_camera
        │ calibrate
        ▼
results.T_lidar_camera
```

기본 Dockerfile은 `forks/direct_visual_lidar_calibration/docker/jazzy/Dockerfile`이다. Calibration C++ source를 실습 branch에서 수정한 뒤 이미지를 다시 build하면 변경이 실행 결과에 반영된다.

## 시작 조건

- `data/bags/pose-01`부터 `pose-05`까지 존재한다.
- 각 bag에 PointCloud2, Image, CameraInfo가 한 종류씩 들어 있다.
- host의 X11 GUI가 Docker에서 표시된다.
- Calibration fork의 `origin`과 `upstream`이 설정되어 있다.

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo
find data/bags -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort
git -C forks/direct_visual_lidar_calibration remote -v
git -C forks/direct_visual_lidar_calibration status --short --branch
```

Calibration fork source로 이미지를 build한다. Source를 수정하면 같은 명령을 다시 실행한다.

```bash
CALIB_COMMIT=$(git -C forks/direct_visual_lidar_calibration rev-parse HEAD)

docker build \
  --file forks/direct_visual_lidar_calibration/docker/jazzy/Dockerfile \
  --tag direct-visual-lidar-calibration:fork \
  --label "lab.direct_visual_lidar_calibration.commit=$CALIB_COMMIT" \
  forks/direct_visual_lidar_calibration

docker image inspect direct-visual-lidar-calibration:fork \
  --format '{{index .Config.Labels "lab.direct_visual_lidar_calibration.commit"}}'
```

### PCL build와 link 확인

PCL은 별도로 실행하는 server가 아니다. Calibration image 안에서 C++ library가 발견되고 `libdirect_visual_lidar_calibration.so`에 link됐는지 확인한다.

```bash
docker run --rm direct-visual-lidar-calibration:fork bash -lc '
  set -euo pipefail
  pkg-config --modversion pcl_common

  prefix=$(ros2 pkg prefix direct_visual_lidar_calibration)
  library="$prefix/lib/libdirect_visual_lidar_calibration.so"
  test -f "$library"

  ! ldd "$library" | grep -q "not found"
  ldd "$library" | grep -E "libpcl_(common|filters|surface)"
'
```

**통과 기준:** PCL version 한 줄과 `libpcl_common`, `libpcl_filters`, `libpcl_surface` 계열 link가 출력되고 명령 종료 code가 0이다.

| 결과 | 의미 | 다음 확인 |
|---|---|---|
| `Package PCL not found` 또는 CMake의 PCL 오류 | `libpcl-dev` 설치나 CMake 검색 실패 | Dockerfile의 `libpcl-dev` 설치와 image rebuild 확인 |
| `libpcl_*.so => not found` | compile은 됐지만 runtime linker가 library를 찾지 못함 | image 내부 package와 linker cache 확인 |
| version과 link 출력 정상 | PCL compile·link 환경 정상 | 아래 preprocess runtime 결과 확인 |
| `/calib/points`만 정상 | Gazebo와 ROS bridge 정상 | 아직 PCL 정상 여부는 판정할 수 없음 |

## 추가할 파일

```text
mobile-robot-calibration-repo/
├── data/results/.gitkeep
└── scripts/
    ├── check-calibration-bags.sh
    └── run-calibration.sh
```

별도 Python package, Compose service, wrapper library는 만들지 않는다. Docker 명령 두 개면 충분하다.

## 1. bag 검사 스크립트를 만든다

`scripts/check-calibration-bags.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BAGS_DIR=${1:-"$ROOT_DIR/data/bags"}
IMAGE=${CALIB_IMAGE:-direct-visual-lidar-calibration:fork}

test -d "$BAGS_DIR" || {
  echo "ERROR: bag directory not found: $BAGS_DIR" >&2
  exit 1
}

docker run --rm \
  -v "$BAGS_DIR:/bags:ro" \
  "$IMAGE" \
  bash -lc '
    set -euo pipefail
    count=0
    for bag in /bags/*; do
      test -f "$bag/metadata.yaml" || continue
      count=$((count + 1))
      info=$(ros2 bag info "$bag")
      echo "== $(basename "$bag") =="
      echo "$info"
      for expected in \
        "Topic: /calib/points | Type: sensor_msgs/msg/PointCloud2" \
        "Topic: /camera/image_raw | Type: sensor_msgs/msg/Image" \
        "Topic: /camera/camera_info | Type: sensor_msgs/msg/CameraInfo"; do
        grep -Fq "$expected" <<<"$info" || {
          echo "ERROR: $(basename "$bag") missing $expected" >&2
          exit 1
        }
      done
    done
    test "$count" -ge 5 || {
      echo "ERROR: expected at least 5 bags, found $count" >&2
      exit 1
    }
  '
```

실행 권한을 주고 검사한다.

```bash
chmod +x scripts/check-calibration-bags.sh
./scripts/check-calibration-bags.sh
```

이 검사는 topic과 type을 확인한다. `intensity` field는 bag metadata에 저장되지 않으므로 Simulation PATCH-01의 live 검사와 preprocess 로그에서 별도로 확인한다.

## 2. 단계별 실행 스크립트를 만든다

`scripts/run-calibration.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BAGS_DIR="$ROOT_DIR/data/bags"
RESULTS_DIR="$ROOT_DIR/data/results"
IMAGE=${CALIB_IMAGE:-direct-visual-lidar-calibration:fork}
STEP=${1:-}

usage() {
  echo "Usage: $0 {preprocess|initial|calibrate|viewer}"
}

test -n "$STEP" || { usage; exit 2; }
mkdir -p "$RESULTS_DIR"

docker_gui() {
  docker run --rm -it \
    --network host \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$BAGS_DIR:/bags:ro" \
    -v "$RESULTS_DIR:/results:rw" \
    "$IMAGE" "$@"
}

case "$STEP" in
  preprocess)
    "$ROOT_DIR/scripts/check-calibration-bags.sh" "$BAGS_DIR"
    test ! -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: data/results/calib.json already exists; move it aside before preprocessing" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration preprocess \
      /bags /results \
      --camera_info_topic /camera/camera_info \
      --image_topic /camera/image_raw \
      --points_topic /calib/points \
      --intensity_channel intensity \
      --visualize
    ;;
  initial)
    test -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: run preprocess first" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration \
      initial_guess_manual /results
    ;;
  calibrate)
    test -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: run preprocess and initial first" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration calibrate /results
    ;;
  viewer)
    test -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: no calibration result" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration viewer /results
    ;;
  *)
    usage
    exit 2
    ;;
esac
```

실행 권한을 준다.

```bash
chmod +x scripts/run-calibration.sh
```

`--user`를 쓰는 이유는 결과 파일이 root 소유가 되는 것을 막기 위해서다. NVIDIA GPU를 요구하지 않으며 기본값은 Mesa software rendering이다. NVIDIA override가 필요하면 host에서 `LIBGL_ALWAYS_SOFTWARE=0`과 Docker GPU 옵션을 별도로 추가하지만, 첫 실습에서는 CPU 경로로 충분하다.

## 3. preprocess를 실행한다

호스트에서 X11 접근을 열고 실행한다.

```bash
xhost +local:docker
./scripts/run-calibration.sh preprocess
```

로그에서 다음 선택을 확인한다.

```text
camera_info: /camera/camera_info
image:       /camera/image_raw
points:      /calib/points
intensity_channel: intensity
```

같은 로그에서 다음 값도 확인한다.

```text
lidar_points[0]: <0보다 큰 정수>
LiDAR FoV: <유한한 각도>[deg]
```

`LiDAR FoV`는 [`estimate_lidar_fov()`](../../forks/direct_visual_lidar_calibration/src/vlcal/common/estimate_fov.cpp#L53)가 PCL VoxelGrid와 ConvexHull을 실행한 결과다. 이 값이 유한하고 preprocessing이 `calib.json`, PLY, LiDAR image까지 저장하면 **현재 rosbag 경로에서 사용하는 PCL 처리가 완료된 것**이다.

| runtime 결과 | 판정 |
|---|---|
| `lidar_points[0]`이 0 또는 process abort | 입력 point, field 해석, 거리 filter부터 확인 |
| `LiDAR FoV: nan[deg]`, ConvexHull/Qhull 오류 | downsampling 후 point가 너무 적거나 cloud의 공간 분포가 hull 계산에 부적합 |
| FoV는 유한하지만 실제 sensor FoV와 크게 다름 | PCL 호출은 완료됐지만 입력 cloud나 scene coverage가 calibration에 부적합 |
| FoV와 중간 파일은 정상이나 final projection이 어긋남 | PCL 자체보다 intrinsic, timestamp, intensity, initial extrinsic, transform 방향 확인 |

topic 자동 탐지 `-a`를 쓰지 않는 이유는 bag에 `/tf`, `/clock` 등이 함께 있고 향후 image/point topic이 늘어났을 때 잘못 선택되는 것을 막기 위해서다.

처리가 끝나면 확인한다.

```bash
find data/results -maxdepth 1 -type f -printf '%f\n' | sort
```

최소한 다음 종류의 파일이 보여야 한다.

- `calib.json`
- 각 bag의 `.png`
- 각 bag의 `.ply`
- 각 bag의 LiDAR intensity/index image

### preprocess가 너무 많은 점을 버릴 때

도구의 기본 `--min_distance`는 1.0 m다. Simulation PATCH-02 물체는 2 m 이상에 있어 기본값을 그대로 쓴다. scene을 가까이 옮겼다면 임의로 여러 옵션을 바꾸지 말고 `--min_distance 0.3`만 추가한다.

## 4. manual initial guess를 만든다

```bash
./scripts/run-calibration.sh initial
```

GUI에서 다음 순서로 수행한다.

1. 3D cloud에서 분명한 모서리나 원통 끝점을 우클릭한다.
2. image에서 같은 물리 지점을 우클릭한다.
3. `Add picked points`를 누른다.
4. 가까운/중간/먼 깊이에서 최소 6쌍을 반복한다.
5. 한 평면 위 점만 고르지 않는다.
6. `Estimate`를 누른다.
7. `blend_weight`로 대략적인 projection을 확인한다.
8. 올바르게 겹치면 `Save`를 누른다.

도구의 수학적 최소치는 3쌍이지만, 클릭 오차와 잘못된 대응을 줄이기 위해 이 실습은 6쌍 이상을 사용한다.

SuperGlue 자동 matching은 이 PATCH에서 쓰지 않는다. 기본 Jazzy Dockerfile에 포함되지 않으며 별도 model·dependency의 license 검토까지 추가되기 때문이다.

저장 여부를 확인한다.

```bash
python3 - <<'PY'
import json
from pathlib import Path

data = json.loads(Path('data/results/calib.json').read_text())
value = data.get('results', {}).get('init_T_lidar_camera')
assert isinstance(value, list) and len(value) == 7, value
print(value)
PY
```

## 5. fine calibration을 실행한다

```bash
./scripts/run-calibration.sh calibrate
```

기본 `nid_bfgs` 설정부터 사용한다. 실패했다고 바로 histogram bin, optimizer, voxel size를 모두 바꾸지 않는다. 먼저 initial projection과 intensity image가 정상인지 확인한다.

계산이 끝나면 `calib.json`을 검사한다.

```bash
python3 - <<'PY'
import json
import math
from pathlib import Path

data = json.loads(Path('data/results/calib.json').read_text())
t = data.get('results', {}).get('T_lidar_camera')
assert isinstance(t, list) and len(t) == 7, t
assert all(math.isfinite(v) for v in t), t
qnorm = math.sqrt(sum(v * v for v in t[3:7]))
assert abs(qnorm - 1.0) < 1e-3, qnorm
print('T_lidar_camera =', t)
print('quaternion norm =', qnorm)
PY
```

배열 순서는 다음과 같다.

```text
[x, y, z, qx, qy, qz, qw]
```

transform 방향은 다음과 같다.

```text
p_lidar = T_lidar_camera * p_camera
```

즉, 이름만 보고 역변환으로 해석하지 않는다.

## 6. viewer로 정성 검증한다

```bash
./scripts/run-calibration.sh viewer
```

다음 항목을 전체 image에서 본다.

- 가까운 박스 모서리에 point projection이 맞는가
- 먼 panel에서도 한쪽으로 일정하게 밀리지 않는가
- 원통의 좌우 윤곽이 image 윤곽과 맞는가
- 화면 중앙만 맞고 가장자리에서 벌어지지 않는가
- 한 bag만 맞고 나머지 pose에서 어긋나지 않는가

중앙만 맞고 가장자리에서 벌어지면 camera intrinsic 또는 distortion을 먼저 의심한다. 모든 지점이 같은 방향으로 평행 이동하면 translation 또는 frame direction을 의심한다. 거리에 따라 오차 방향이 바뀌면 rotation을 의심한다.

## 7. 결과와 fork 변경을 확인한다

```bash
git -C forks/direct_visual_lidar_calibration status --short
git status --short
```

Calibration source를 수정했다면 첫 명령에 의도한 파일만 보여야 한다. 검증 후 해당 fork의 실습 branch에 commit·push한다. Calibration 결과는 `data/results/`에만 생성되어야 한다.

## 완료 조건

- `data/results/calib.json`이 존재한다.
- `results.init_T_lidar_camera`와 `results.T_lidar_camera`가 각각 7개 유한값이다.
- quaternion norm이 1에 가깝다.
- viewer에서 가까운/먼 물체와 image 가장자리까지 projection이 일관되다.
- raw bag은 수정되지 않았다.
- `forks/direct_visual_lidar_calibration`은 clean 상태다.

## 실패할 때 확인 순서

### `failed to determine point intensity channel`

Simulation PATCH-01로 돌아가 `PointCloud2.fields`를 확인한다. `--intensity_channel intensity` 이름과 field 이름이 정확히 같아야 한다.

### preprocess GUI가 열리지 않는다

```bash
echo "$DISPLAY"
ls -l /tmp/.X11-unix
xhost +local:docker
```

그 후 다시 실행한다. 데이터 처리가 문제인지 GUI만 문제인지 분리하려면 스크립트의 preprocess 명령에서 `--visualize`만 잠시 빼고 실행한다.

### initial guess 후 projection이 뒤집혀 보인다

`/camera/image_raw`, `/camera/camera_info`의 `frame_id`가 `camera_rgb_optical_frame`인지 확인한다. `T_lidar_camera`와 그 inverse를 임의로 번갈아 넣지 않는다.

### calibration이 큰 값으로 발산한다

optimizer option을 바꾸기 전에 다음을 확인한다.

1. 6쌍의 3D–2D 대응이 실제 같은 점인가
2. 가까운/중간/먼 점이 모두 포함됐는가
3. LiDAR intensity image에 구조와 texture가 보이는가
4. 5개 bag이 같은 고정 extrinsic으로 기록됐는가

## 이 PATCH에서 하지 않는 것

- calibration 소스 build
- SuperGlue 설치
- 결과를 URDF에 즉시 적용
- 정량 ground-truth 비교

transform 방향 변환과 정량 비교는 Simulation PATCH-04에서 한 번에 처리한다.
