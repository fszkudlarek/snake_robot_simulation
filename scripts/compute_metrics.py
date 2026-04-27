#!/usr/bin/env python3
"""
Compute sidewinding motion metrics from a robot_body_logger CSV.

Metrics:
    S (straightness)  = ||P_K - P_0|| / sum_k ||P_{k+1} - P_k||   in [0, 1]
    v (speed, m/s)    = ||P_K - P_0|| / (t_K - t_0)
    J (objective)     = v * S^alpha

P_k are cycle-to-cycle samples of the COM: one sample per gait period T,
taken after discarding an initial transient. This removes gait oscillation
from the path-length estimate so S reflects trajectory curvature, not sway.

Usage:
    python3 scripts/compute_metrics.py <body_trajectory.csv> --period 5.0
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


# Below this net displacement, the straightness index is numerically
# unstable (division by a sum of tiny oscillations). Treat as non-moving.
MIN_NET_DISPLACEMENT_M = 0.05


def _sample_cycle_points(df: pd.DataFrame, period: float,
                         transient_cycles: float) -> pd.DataFrame:
    """Pick one row per gait cycle starting after the transient.

    Uses the row whose `time` is closest to t_start + k*T for each k.
    """
    t0 = float(df['time'].iloc[0]) + transient_cycles * period
    t_end = float(df['time'].iloc[-1])

    if t_end <= t0:
        raise ValueError(
            f'Not enough data after transient: t_end={t_end:.2f}s, '
            f't0={t0:.2f}s (transient_cycles={transient_cycles}, T={period}).'
        )

    n_cycles = int((t_end - t0) // period)
    if n_cycles < 2:
        raise ValueError(
            f'Only {n_cycles} full cycles after transient — need at least 2.'
        )

    targets = t0 + np.arange(n_cycles + 1) * period
    times = df['time'].to_numpy()
    # np.searchsorted + nearest-of-two gives the closest sample to each target
    idxs_right = np.searchsorted(times, targets)
    idxs_right = np.clip(idxs_right, 1, len(times) - 1)
    idxs_left = idxs_right - 1
    pick_right = (times[idxs_right] - targets) < (targets - times[idxs_left])
    idxs = np.where(pick_right, idxs_right, idxs_left)
    return df.iloc[idxs].reset_index(drop=True)


def compute_metrics(csv_path: Path, period: float,
                    transient_cycles: float = 2.0,
                    alpha: float = 2.0) -> dict:
    """Return {n_cycles, duration_s, net_disp_m, path_len_m, S, v, J}."""
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=['com_x', 'com_y']).reset_index(drop=True)
    if len(df) < 2:
        raise ValueError(f'{csv_path}: fewer than 2 valid COM samples.')

    samples = _sample_cycle_points(df, period, transient_cycles)
    pts = samples[['com_x', 'com_y']].to_numpy()
    times = samples['time'].to_numpy()

    segments = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    path_len = float(segments.sum())
    net_disp = float(np.linalg.norm(pts[-1] - pts[0]))
    duration = float(times[-1] - times[0])

    if net_disp < MIN_NET_DISPLACEMENT_M:
        straightness = 0.0
    else:
        straightness = net_disp / path_len if path_len > 0 else 0.0

    speed = net_disp / duration if duration > 0 else 0.0
    objective = speed * (straightness ** alpha)

    return {
        'n_cycles': len(samples) - 1,
        'duration_s': duration,
        'net_disp_m': net_disp,
        'path_len_m': path_len,
        'S': straightness,
        'v': speed,
        'J': objective,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Compute sidewinding motion metrics from a body trajectory CSV.')
    parser.add_argument('csv', type=Path, help='Path to body_trajectory.csv')
    parser.add_argument('--period', type=float, default=5.0,
                        help='Gait period T in seconds (default: 5.0)')
    parser.add_argument('--transient-cycles', type=float, default=2.0,
                        help='Number of initial gait cycles to discard (default: 2.0)')
    parser.add_argument('--alpha', type=float, default=2.0,
                        help='Straightness exponent in J = v * S^alpha (default: 2.0)')
    args = parser.parse_args()

    try:
        m = compute_metrics(args.csv, args.period,
                            args.transient_cycles, args.alpha)
    except (FileNotFoundError, ValueError) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1

    print(f'File:            {args.csv}')
    print(f'Cycles analyzed: {m["n_cycles"]} over {m["duration_s"]:.2f} s '
          f'(T={args.period}s, skipped {args.transient_cycles} cycles)')
    print(f'Net disp:        {m["net_disp_m"]:.4f} m')
    print(f'Path length:     {m["path_len_m"]:.4f} m')
    print(f'Straightness S:  {m["S"]:.4f}')
    print(f'Speed v:         {m["v"]:.4f} m/s')
    print(f'Objective J:     {m["J"]:.6f}   (= v * S^{args.alpha})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
