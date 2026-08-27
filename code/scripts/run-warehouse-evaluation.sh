#!/usr/bin/env bash
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
