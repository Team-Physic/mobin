#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import argparse
import glob
import json
import os


def aggregate(paths):
    runs = []
    for path in paths:
        with open(path, encoding='utf-8') as source:
            runs.append(json.load(source))
    if not runs:
        raise ValueError('평가 JSON이 없습니다.')
    latencies = [
        run['tracking_latency_p95_seconds'] for run in runs
        if run.get('tracking_latency_p95_seconds') is not None
    ]
    return {
        'run_count': len(runs),
        'collision_count': sum(run['collision_count'] for run in runs),
        'completion_rate': sum(
            bool(run['patrol_completed']) for run in runs) / len(runs),
        'mean_hota_at_0.75m': sum(
            run['hota_at_0.75m'] for run in runs) / len(runs),
        'mean_idf1': sum(run['idf1'] for run in runs) / len(runs),
        'mean_mota': sum(run['mota'] for run in runs) / len(runs),
        'minimum_person_distance': min((
            run['minimum_person_distance'] for run in runs
            if run['minimum_person_distance'] is not None), default=None),
        'mean_tracking_latency_p95_seconds': (
            sum(latencies) / len(latencies) if latencies else None),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results_dir')
    parser.add_argument('--output', default='summary.json')
    arguments = parser.parse_args()
    paths = sorted(glob.glob(os.path.join(arguments.results_dir, 'seed-*.json')))
    summary = aggregate(paths)
    output_path = os.path.join(arguments.results_dir, arguments.output)
    with open(output_path, 'w', encoding='utf-8') as output:
        json.dump(summary, output, indent=2, sort_keys=True)
        output.write('\n')
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
