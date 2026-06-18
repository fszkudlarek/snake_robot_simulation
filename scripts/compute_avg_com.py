#!/usr/bin/env python3
"""
Compute averaged COM positions and body orientation per sweep run.

For each run subdirectory under <sweep_dir>:
  1. Read <run>_body_trajectory.csv (segment x/y, com_x, com_y, time)
  2. Drop the first skip_cycles*period seconds as transient
  3. Use the next analysis_cycles*period seconds as the analysis window
  4. For each cycle h in 0..analysis_cycles-1, average over the window
       [skip_cycles*period + h*period, skip_cycles*period + (h+1)*period)
     producing:
       - mean COM (com_x, com_y)
       - mean body orientation: PCA principal-axis angle of segment
         positions per timestep, sign-flipped for branch continuity
         (so multi-revolution rotation is captured), then averaged over
         the cycle window
  5. Write per-run outputs into the run's directory:
       <run>_avg_com.csv  - h, t_start, t_end, com_x, com_y, orientation_rad
       <run>_avg_com.png  - top-down COM trail with the points + axis ticks

Usage:
    python3 scripts/compute_avg_com.py <sweep_dir> \
        [--skip-cycles 5] [--analysis-cycles 5] [--period 5.0]
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def _segment_columns(df: pd.DataFrame) -> list[tuple[str, str]]:
    """Return [(x_col, y_col), ...] for each body segment, excluding COM."""
    pairs = []
    for col in df.columns:
        if col == 'com_x' or not col.endswith('_x'):
            continue
        base = col[:-2]
        y_col = f'{base}_y'
        if y_col in df.columns:
            pairs.append((col, y_col))
    return pairs


def _continuous_orientation_series(df: pd.DataFrame,
                                   seg_cols: list[tuple[str, str]]) -> pd.Series:
    """Per-row body orientation (rad), branch-continuous across timesteps.

    Per row, fits the principal axis to the segment positions via the closed
    form of 2x2 PCA:  theta = 0.5 * atan2(2*Sxy, Sxx - Syy). Raw axis angles
    live in (-pi/2, pi/2] (axis is sign-ambiguous). To track multi-revolution
    rotation, each new sample is shifted by an integer multiple of pi to be
    closest to the previous one — assumes the per-sample rotation stays under
    pi/2 (always true at body_trajectory rates).

    Rows where any segment x/y is NaN get NaN orientation.
    """
    if not seg_cols:
        return pd.Series(np.nan, index=df.index)

    xs = df[[x for x, _ in seg_cols]].to_numpy(dtype=float)
    ys = df[[y for _, y in seg_cols]].to_numpy(dtype=float)

    valid = ~np.isnan(xs).any(axis=1) & ~np.isnan(ys).any(axis=1)
    raw = np.full(len(df), np.nan)

    if valid.any():
        sx, sy = xs[valid], ys[valid]
        mx = sx.mean(axis=1, keepdims=True)
        my = sy.mean(axis=1, keepdims=True)
        dx, dy = sx - mx, sy - my
        sxx = (dx * dx).sum(axis=1)
        syy = (dy * dy).sum(axis=1)
        sxy = (dx * dy).sum(axis=1)
        raw[valid] = 0.5 * np.arctan2(2 * sxy, sxx - syy)

    fixed = np.copy(raw)
    prev = None
    for i, a in enumerate(fixed):
        if math.isnan(a):
            continue
        if prev is None:
            prev = a
            continue
        delta = (a - prev) % math.pi
        if delta > math.pi / 2:
            delta -= math.pi
        fixed[i] = prev + delta
        prev = fixed[i]

    return pd.Series(fixed, index=df.index)


def process_run(traj_csv: Path, skip_cycles: float, analysis_cycles: int,
                period: float, write_outputs: bool = True,
                desired_path: np.ndarray | None = None) -> pd.DataFrame | None:
    """Compute averaged-COM + orientation rows for one run.

    Returns the resulting DataFrame (or None if no rows could be produced).
    When write_outputs is True, also writes <run>_avg_com.csv and the PNG.
    The caller can suppress writes when computing alternative configurations
    (e.g. a skip_cycles scan) without touching the persisted artifacts.

    When desired_path (an (N, 2) world-frame [x, y] polyline) is given, it is
    overlaid on the PNG as the path the run was tracking.
    """
    run_name = traj_csv.stem.replace('_body_trajectory', '')
    out_dir = traj_csv.parent

    df = pd.read_csv(traj_csv)
    com = df.dropna(subset=['com_x', 'com_y']).copy()
    if com.empty:
        print(f'  {run_name}: no COM samples, skipped', file=sys.stderr)
        return None

    seg_cols = _segment_columns(df)
    ori_series = _continuous_orientation_series(df, seg_cols)
    # Relative-to-initial orientation: subtract the first valid PCA angle so
    # the series starts at 0 at the very first body-trajectory sample. This
    # matters for downstream metrics that want to express world displacement
    # in the *initial* robot frame — at skip_cycles=0 the rotation applied
    # is then ~0, so the local-frame and world-frame plots agree.
    ori_first_valid = ori_series.dropna()
    ori_relative = (ori_series - ori_first_valid.iloc[0]
                    if not ori_first_valid.empty else ori_series)
    df = df.assign(_orientation_rad=ori_series,
                   _orientation_relative_rad=ori_relative)

    t0 = skip_cycles * period
    t1 = t0 + analysis_cycles * period
    t_max = com['time'].max()
    if t_max < t1:
        print(f'  {run_name}: trajectory ends at t={t_max:.2f}s, '
              f'need >= {t1:.2f}s. Will compute as much as possible.',
              file=sys.stderr)

    rows = []
    for h in range(analysis_cycles):
        t_start = t0 + h * period
        t_end = t_start + period
        com_window = com[(com['time'] >= t_start) & (com['time'] < t_end)]
        if com_window.empty:
            print(f'  {run_name}: window {h} [{t_start:.2f}, {t_end:.2f}) is empty',
                  file=sys.stderr)
            continue
        win = df[(df['time'] >= t_start) & (df['time'] < t_end)]
        ori_window = win['_orientation_rad'].dropna()
        ori_rel_window = win['_orientation_relative_rad'].dropna()
        rows.append({
            'h': h,
            't_start': t_start,
            't_end': t_end,
            'com_x': com_window['com_x'].mean(),
            'com_y': com_window['com_y'].mean(),
            'orientation_rad': ori_window.mean() if not ori_window.empty
                else float('nan'),
            'orientation_relative_rad': ori_rel_window.mean()
                if not ori_rel_window.empty else float('nan'),
        })

    if not rows:
        print(f'  {run_name}: no averaged points produced, skipped', file=sys.stderr)
        return None

    avg_df = pd.DataFrame(rows)

    if not write_outputs:
        return avg_df

    avg_csv = out_dir / f'{run_name}_avg_com.csv'
    avg_df.to_csv(avg_csv, index=False)

    # Top-down plot: full COM trail (gray), analysis window highlighted, and
    # the averaged points labeled by index. Equal aspect so geometry is honest.
    # A short line at each averaged point shows the principal axis orientation.
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect('equal')
    if desired_path is not None and len(desired_path) >= 1:
        ax.plot(desired_path[:, 0], desired_path[:, 1], '--', color='tab:blue',
                lw=1.2, alpha=0.7, zorder=0, label='desired path')
    ax.plot(com['com_x'], com['com_y'], '-', color='lightgray', lw=0.8,
            label='COM trail (full)')

    window_df = com[(com['time'] >= t0) & (com['time'] < t1)]
    if not window_df.empty:
        ax.plot(window_df['com_x'], window_df['com_y'], '-',
                color='tab:orange', lw=1.2,
                label=f'analysis window [{t0:.1f}s, {t1:.1f}s)')

    ax.plot(avg_df['com_x'], avg_df['com_y'], 'o', color='red', ms=8,
            label=f'{len(avg_df)} averaged points')
    for _, r in avg_df.iterrows():
        ax.annotate(f'{int(r["h"])}',
                    (r['com_x'], r['com_y']),
                    textcoords='offset points', xytext=(6, 6),
                    fontsize=9, color='red')

    # Orientation tick: a short segment along the principal axis at each
    # averaged COM point. Length scaled to the plot extent so it's visible
    # but doesn't dominate.
    x_range = com['com_x'].max() - com['com_x'].min()
    y_range = com['com_y'].max() - com['com_y'].min()
    tick_len = 0.05 * max(x_range, y_range, 1e-6)
    drew_orientation = False
    for _, r in avg_df.iterrows():
        if pd.isna(r.get('orientation_rad')):
            continue
        cx, cy, theta = r['com_x'], r['com_y'], r['orientation_rad']
        dx, dy = tick_len * math.cos(theta), tick_len * math.sin(theta)
        ax.plot([cx - dx, cx + dx], [cy - dy, cy + dy],
                '-', color='purple', lw=1.5,
                label='principal axis' if not drew_orientation else None)
        drew_orientation = True

    ax.set_title(f'{run_name}  '
                 f'(skip_cycles={skip_cycles}, '
                 f'analysis_cycles={analysis_cycles}, period={period})')
    ax.set_xlabel('com_x [m]')
    ax.set_ylabel('com_y [m]')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize='small')

    png_path = out_dir / f'{run_name}_avg_com.png'
    fig.savefig(png_path, dpi=120, bbox_inches='tight')
    plt.close(fig)

    print(f'  {run_name}: wrote {avg_csv.name} and {png_path.name}', flush=True)
    return avg_df


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Compute averaged COM positions per sweep run.')
    parser.add_argument('sweep_dir', type=Path,
                        help='Sweep output directory (contains per-run subdirs)')
    parser.add_argument('--skip-cycles', type=float, default=10.0,
                        help='Cycles of transient to skip (default: 10.0)')
    parser.add_argument('--analysis-cycles', type=int, default=8,
                        help='Number of averaged windows to compute (default: 8)')
    parser.add_argument('--period', type=float, default=5.0,
                        help='Controller period in seconds (default: 5.0)')
    args = parser.parse_args()

    if not args.sweep_dir.is_dir():
        print(f'ERROR: {args.sweep_dir} is not a directory', file=sys.stderr)
        return 1

    run_dirs = sorted(p for p in args.sweep_dir.iterdir() if p.is_dir())
    if not run_dirs:
        print(f'ERROR: no run subdirectories in {args.sweep_dir}', file=sys.stderr)
        return 1

    t0 = args.skip_cycles * args.period
    t1 = t0 + args.analysis_cycles * args.period
    print(f'Processing {len(run_dirs)} runs from {args.sweep_dir}')
    print(f'skip_cycles={args.skip_cycles}, '
          f'analysis_cycles={args.analysis_cycles}, '
          f'period={args.period}  -> '
          f'skip [0, {t0:.1f}s), use [{t0:.1f}s, {t1:.1f}s)')

    for run_dir in run_dirs:
        traj_csvs = sorted(run_dir.glob('*_body_trajectory.csv'))
        if not traj_csvs:
            print(f'  {run_dir.name}: no *_body_trajectory.csv, skipped',
                  file=sys.stderr)
            continue
        if len(traj_csvs) > 1:
            print(f'  {run_dir.name}: multiple body_trajectory CSVs, '
                  f'using {traj_csvs[0].name}', file=sys.stderr)
        process_run(traj_csvs[0], args.skip_cycles, args.analysis_cycles,
                    args.period)

    return 0


if __name__ == '__main__':
    sys.exit(main())
