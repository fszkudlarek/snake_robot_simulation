#!/usr/bin/env python3
"""
Compute motion metrics from a robot_body_logger CSV.

Two gaits are supported:

SIDEWINDING (default):
    S (straightness)  = ||P_K - P_0|| / sum_k ||P_{k+1} - P_k||   in [0, 1]
    v (speed, m/s)    = ||P_K - P_0|| / (t_K - t_0)
    J = v * S^alpha

    P_k are cycle-to-cycle COM samples — sampling at the gait period removes
    lateral oscillation from the path-length estimate so S reflects trajectory
    curvature, not sway.

ROTATING:
    omega (angular speed, rad/s)  = |orientation(t_K) - orientation(t_0)| / (t_K - t_0)
    drift_speed (m/s)             = ||COM(t_K) - COM(t_0)|| / (t_K - t_0)
    R (rotation purity)           = (omega * L/2) / (omega * L/2 + drift_speed)   in [0, 1]
    J = omega * R^alpha

    Orientation is the head→tail body vector angle (atan2). Cycle-to-cycle
    sampling cancels the wave-induced wobble of the body line. Drift is the
    chord distance the COM travels — pure rotation has zero drift; R = 1.

Cycle-to-cycle sampling discards a configurable number of leading transient
cycles before measurement.

Usage:
    python3 scripts/compute_metrics.py <body_trajectory.csv> --period 5.0
    python3 scripts/compute_metrics.py <body_trajectory.csv> --gait rotating --period 5.0
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


# Below this net displacement, the straightness index is numerically
# unstable (division by a sum of tiny oscillations). Treat as non-moving.
MIN_NET_DISPLACEMENT_M = 0.05

# Below this angular speed, the rotation purity index is numerically unstable
# (drift dominates a near-zero rotational extremity speed). Treat as non-rotating.
MIN_OMEGA_RAD_PER_S = 1e-3

# CSV columns used for the rotating-gait orientation estimate. Must match
# the LINK_FRAMES list in robot_body_logger.py.
HEAD_FRAME = 'motor_with_onshape_mounting'
TAIL_FRAME = 'motor_with_no_arms'

GAITS = ('sidewinding', 'rotating')


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


def _compute_sidewinding(samples: pd.DataFrame, alpha: float) -> dict:
    pts = samples[['com_x', 'com_y']].to_numpy()
    times = samples['time'].to_numpy()

    segments = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    path_len = float(segments.sum())
    net_disp = float(np.linalg.norm(pts[-1] - pts[0]))
    duration = float(times[-1] - times[0])

    if net_disp < MIN_NET_DISPLACEMENT_M:
        S = 0.0
    else:
        S = net_disp / path_len if path_len > 0 else 0.0

    v = net_disp / duration if duration > 0 else 0.0
    J = v * (S ** alpha)

    return {
        'duration_s': duration,
        'net_disp_m': net_disp,
        'path_len_m': path_len,
        'S': S,
        'v': v,
        'J': J,
    }


def _compute_rotating(samples: pd.DataFrame, alpha: float) -> dict:
    times = samples['time'].to_numpy()
    com = samples[['com_x', 'com_y']].to_numpy()
    head = samples[[f'{HEAD_FRAME}_x', f'{HEAD_FRAME}_y']].to_numpy()
    tail = samples[[f'{TAIL_FRAME}_x', f'{TAIL_FRAME}_y']].to_numpy()

    body_vec = tail - head
    body_lengths = np.linalg.norm(body_vec, axis=1)
    body_length = float(body_lengths.mean())

    duration = float(times[-1] - times[0])

    # If the body has collapsed into a point, orientation is undefined.
    if body_length < 1e-3:
        return {
            'duration_s': duration,
            'net_rotation_rad': 0.0,
            'drift_distance_m': float(np.linalg.norm(com[-1] - com[0])),
            'body_length_m': body_length,
            'omega': 0.0,
            'drift_speed': 0.0,
            'R': 0.0,
            'J': 0.0,
        }

    orientation = np.unwrap(np.arctan2(body_vec[:, 1], body_vec[:, 0]))
    net_rotation = float(orientation[-1] - orientation[0])
    drift_distance = float(np.linalg.norm(com[-1] - com[0]))

    omega = abs(net_rotation) / duration if duration > 0 else 0.0
    drift_speed = drift_distance / duration if duration > 0 else 0.0

    half_body = body_length / 2.0
    rotational_extremity_speed = omega * half_body

    if omega < MIN_OMEGA_RAD_PER_S:
        R = 0.0
    else:
        denom = rotational_extremity_speed + drift_speed
        R = rotational_extremity_speed / denom if denom > 0 else 0.0

    J = omega * (R ** alpha)

    return {
        'duration_s': duration,
        'net_rotation_rad': net_rotation,
        'drift_distance_m': drift_distance,
        'body_length_m': body_length,
        'omega': omega,
        'drift_speed': drift_speed,
        'R': R,
        'J': J,
    }


def compute_metrics(csv_path: Path, period: float,
                    transient_cycles: float = 2.0,
                    alpha: float = 2.0,
                    gait: str = 'sidewinding') -> dict:
    """Dispatch to the per-gait metric. Returns a dict with at least:
        gait, n_cycles, duration_s, J
    plus gait-specific extras (S, v, net_disp_m, path_len_m for sidewinding;
    R, omega, drift_speed, drift_distance_m, body_length_m, net_rotation_rad
    for rotating).
    """
    if gait not in GAITS:
        raise ValueError(f'Unknown gait: {gait!r}; expected one of {GAITS}')

    df = pd.read_csv(csv_path)

    needed = ['com_x', 'com_y']
    if gait == 'rotating':
        needed += [f'{HEAD_FRAME}_x', f'{HEAD_FRAME}_y',
                   f'{TAIL_FRAME}_x', f'{TAIL_FRAME}_y']
    df = df.dropna(subset=needed).reset_index(drop=True)
    if len(df) < 2:
        raise ValueError(
            f'{csv_path}: fewer than 2 valid samples for gait={gait!r}.')

    samples = _sample_cycle_points(df, period, transient_cycles)
    n_cycles = len(samples) - 1

    if gait == 'sidewinding':
        gait_metrics = _compute_sidewinding(samples, alpha)
    else:  # rotating
        gait_metrics = _compute_rotating(samples, alpha)

    return {'gait': gait, 'n_cycles': n_cycles, **gait_metrics}


def _print_sidewinding(m: dict, alpha: float) -> None:
    print(f'Net disp:        {m["net_disp_m"]:.4f} m')
    print(f'Path length:     {m["path_len_m"]:.4f} m')
    print(f'Straightness S:  {m["S"]:.4f}')
    print(f'Speed v:         {m["v"]:.4f} m/s')
    print(f'Objective J:     {m["J"]:.6f}   (= v * S^{alpha})')


def _print_rotating(m: dict, alpha: float) -> None:
    print(f'Body length:     {m["body_length_m"]:.4f} m')
    print(f'Net rotation:    {m["net_rotation_rad"]:+.4f} rad '
          f'({np.degrees(m["net_rotation_rad"]):+.1f}°)')
    print(f'Drift distance:  {m["drift_distance_m"]:.4f} m')
    print(f'Angular ω:       {m["omega"]:.4f} rad/s')
    print(f'Drift speed:     {m["drift_speed"]:.6f} m/s')
    print(f'Rotation R:      {m["R"]:.4f}')
    print(f'Objective J:     {m["J"]:.6f}   (= ω * R^{alpha})')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Compute motion metrics from a body trajectory CSV.')
    parser.add_argument('csv', type=Path, help='Path to body_trajectory.csv')
    parser.add_argument('--period', type=float, default=5.0,
                        help='Gait period T in seconds (default: 5.0)')
    parser.add_argument('--transient-cycles', type=float, default=2.0,
                        help='Number of initial gait cycles to discard (default: 2.0)')
    parser.add_argument('--alpha', type=float, default=2.0,
                        help='Purity exponent in J = speed * purity^alpha (default: 2.0)')
    parser.add_argument('--gait', choices=list(GAITS), default='sidewinding',
                        help='Which gait metric to compute (default: sidewinding)')
    args = parser.parse_args()

    try:
        m = compute_metrics(args.csv, args.period,
                            args.transient_cycles, args.alpha, args.gait)
    except (FileNotFoundError, ValueError, KeyError) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1

    print(f'File:            {args.csv}')
    print(f'Gait:            {args.gait}')
    print(f'Cycles analyzed: {m["n_cycles"]} over {m["duration_s"]:.2f} s '
          f'(T={args.period}s, skipped {args.transient_cycles} cycles)')

    if args.gait == 'sidewinding':
        _print_sidewinding(m, args.alpha)
    else:
        _print_rotating(m, args.alpha)

    return 0


if __name__ == '__main__':
    sys.exit(main())
