#!/usr/bin/env python3
"""
Score a center-of-mass trajectory CSV against the desired path, using the same
metric the CMA optimizer minimizes (RMS cross-track distance + arc covered).

The point is an apples-to-apples comparison: tune the OPEN-LOOP gait with
`optimize_cma.py --objective cross_track`, then run the CLOSED-LOOP amplitude-
modulation tracker (snake_sim_trajectory_launch.py with trajectory_log_path)
and score its body_trajectory.csv here with the *identical* yardstick.

Both sides must be scored against the *same* desired path. Point
--trajectory-file at a CMA session's snapshotted trajectory.yaml to match a
tuning run exactly.

Usage:
  # score one or more body_trajectory.csv files against config/trajectory.yaml
  python3 scripts/score_path_following.py run/run_body_trajectory.csv

  # compare a closed-loop run vs an open-loop CMA best, same path snapshot:
  python3 scripts/score_path_following.py \
      closed_loop/cl_body_trajectory.csv \
      sweep_output/cma/run1/eval_003_02/eval_003_02_body_trajectory.csv \
      --trajectory-file sweep_output/cma/run1/trajectory.yaml

Dependencies: numpy, pandas (and cma, pulled in transitively via optimize_cma);
run it with the same venv as optimize_cma.py.
"""
import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import optimize_cma as oc


def _label(csv_path: Path) -> str:
    return csv_path.stem.replace('_body_trajectory', '') or csv_path.name


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Score COM trajectory CSV(s) against the desired path with '
                    'the same metric optimize_cma uses.')
    parser.add_argument('csv', type=Path, nargs='+',
                        help='One or more *_body_trajectory.csv files '
                             '(columns: time, com_x, com_y).')
    parser.add_argument('--objective', choices=['cross_track', 'time_aligned'],
                        default='cross_track',
                        help='Metric to report (default: cross_track), matching '
                             'optimize_cma --objective.')
    parser.add_argument('--trajectory-file', type=Path, default=None,
                        help='Desired-trajectory YAML to score against (default: '
                             'config/trajectory.yaml; point at a CMA session '
                             'snapshot to match a tuning run exactly).')
    parser.add_argument('--transient-seconds', type=float, default=10.0,
                        help='Initial settling to discard before scoring '
                             '(default: 10.0, matching the optimizer).')
    parser.add_argument('--min-progress', type=float, default=0.3,
                        help='cross_track: flag runs covering less than this '
                             'many metres along the path (default: 0.3).')
    parser.add_argument('--out', type=Path, default=None,
                        help='Optional CSV path to also write the table to.')
    args = parser.parse_args()

    if args.trajectory_file is not None:
        oc.TRAJECTORY_FILE = args.trajectory_file
    polyline, desired_times = oc.load_desired_trajectory()
    print(f'Desired path: {len(polyline)} points, from {oc.TRAJECTORY_FILE}',
          flush=True)
    print(f'Objective: {args.objective}  transient={args.transient_seconds}s',
          flush=True)

    rows = []
    for csv_path in args.csv:
        label = _label(csv_path)
        if not csv_path.exists():
            print(f'  {label}: ERROR file not found: {csv_path}', file=sys.stderr)
            continue
        try:
            if args.objective == 'cross_track':
                m = oc.compute_path_following_error(
                    csv_path, polyline, args.transient_seconds)
            else:
                m = oc.compute_tracking_error(
                    csv_path, polyline, desired_times, args.transient_seconds)
        except (ValueError, FileNotFoundError, KeyError) as e:
            print(f'  {label}: ERROR {e}', file=sys.stderr)
            continue
        m['label'] = label
        rows.append(m)

    if not rows:
        print('No scorable trajectories.', file=sys.stderr)
        return 1

    if args.objective == 'cross_track':
        hdr = (f'{"run":<24} {"rms_xtrack":>11} {"mean":>8} {"max":>8} '
               f'{"covered_m":>10} {"speed_m/s":>10} {"n":>6} {"floor":>6}')
    else:
        hdr = (f'{"run":<24} {"rms_dist":>11} {"mean":>8} {"max":>8} '
               f'{"dur_s":>8} {"n":>6}')
    print('\n' + hdr)
    print('-' * len(hdr))
    for m in rows:
        if args.objective == 'cross_track':
            floor = 'OK' if m['covered_arc_m'] >= args.min_progress else 'LOW'
            print(f'{m["label"]:<24} {m["rms_distance_m"]:>11.4f} '
                  f'{m["mean_distance_m"]:>8.4f} {m["max_distance_m"]:>8.4f} '
                  f'{m["covered_arc_m"]:>10.3f} {m["avg_speed_m_s"]:>10.4f} '
                  f'{m["n_samples"]:>6d} {floor:>6}')
        else:
            print(f'{m["label"]:<24} {m["rms_distance_m"]:>11.4f} '
                  f'{m["mean_distance_m"]:>8.4f} {m["max_distance_m"]:>8.4f} '
                  f'{m["duration_s"]:>8.2f} {m["n_samples"]:>6d}')

    if args.out is not None:
        cols = ['label', 'objective', 'n_samples', 'duration_s',
                'mean_distance_m', 'rms_distance_m', 'max_distance_m',
                'covered_arc_m', 'avg_speed_m_s']
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(cols)
            for m in rows:
                w.writerow([m['label'], args.objective]
                           + [m.get(c, '') for c in cols[2:]])
        print(f'\nWrote {args.out}', flush=True)

    return 0


if __name__ == '__main__':
    sys.exit(main())
