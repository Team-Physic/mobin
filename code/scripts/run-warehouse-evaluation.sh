#!/usr/bin/env bash

# 서로 다른 pedestrian seed로 warehouse patrol simulation을 연속 실행해 일반화 성능을 측정한다.
# RUN_COUNT, SEED_START, RUN_TIMEOUT_SECONDS 환경변수로 실행 수·첫 seed·제한시간을 받는다.
# Seed별 evaluator JSON을 data/results/warehouse-unseen에 저장한 뒤 aggregate script로 집계한다.
# 한 seed가 timeout 또는 실패해도 다음 seed를 계속 실행하며 각 실행 사이 Compose service를 내린다.
# 주요 입력:
# - `RUN_COUNT`: 연속 실행 횟수. 기본값은 `20`이다.
# - `SEED_START`: 첫 pedestrian random seed. 기본값은 `1001`이며 실행마다 1씩 증가한다.
# - `RUN_TIMEOUT_SECONDS`: seed 하나의 최대 실행시간(초). 기본값은 `300`이다.
# - `YOLO_DEVICE` 등 Compose 환경변수는 호출한 shell 값을 그대로 사용한다.

set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
result_dir="${repo_dir}/data/results/warehouse-unseen"
run_count="${RUN_COUNT:-20}"
seed_start="${SEED_START:-1001}"
timeout_seconds="${RUN_TIMEOUT_SECONDS:-300}"
mkdir -p "${result_dir}"

cd "${repo_dir}/docker"
cleanup() {
  docker compose -f compose.yaml -f compose.nvidia.yaml down >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

for ((offset = 0; offset < run_count; ++offset)); do
  seed=$((seed_start + offset))
  output_path="/ws/data/results/warehouse-unseen/seed-${seed}.json"
  echo "[seed ${seed}] warehouse evaluation"
  PEDESTRIAN_RANDOM_SEED="${seed}" \
  EVALUATION_OUTPUT_PATH="${output_path}" \
  timeout --signal=INT --kill-after=15 "${timeout_seconds}" \
    docker compose -f compose.yaml -f compose.nvidia.yaml \
    up --force-recreate --abort-on-container-exit patrol || true
  cleanup
done

python3 "${repo_dir}/code/scripts/aggregate-warehouse-evaluation.py" \
  "${result_dir}"
