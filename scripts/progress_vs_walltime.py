#!/usr/bin/env python3
"""
Plot how far the robot progresses ALONG the desired path as a function of wall
time, from a closed-loop run's body_trajectory.csv.

Purpose: choose --min-progress for optimize_cma. Each CMA eval runs for a fixed
wall budget (--wait-seconds), which buys a fixed amount of arc covered. This
chart shows that buildup; read the value at your budget and set --min-progress
to a small fraction of it (so the floor rejects only near-stationary gaits,
never a slow-but-real follower).

"Progress" here is computed exactly like optimize_cma's covered_arc_m: over the
post-transient window, max(arc projection) - min(arc projection) up to each
time. So the curve value at wall time T is the covered_arc a CMA eval with
--wait-seconds=T would record.

The CSV logs SIM time. Map it to wall time with --wall-duration (the measured
wall seconds the logged run covered) or --rtf (wall = sim / rtf). With neither,
the x-axis is sim time.

Usage:
  python3 scripts/progress_vs_walltime.py cl_body_trajectory.csv \
      --trajectory-file config/trajectory.yaml \
      --wall-duration 400 --budget-seconds 300 --out progress.png

Dependencies: numpy, pandas, matplotlib (run with the optimize_cma venv).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # headless
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
import optimize_cma as oc


def main() -> int:
    p = argparse.ArgumentParser(
        description='Plot arc covered along the path vs wall time, to pick '
                    'optimize_cma --min-progress.')
    p.add_argument('csv', type=Path,
                   help='Closed-loop body_trajectory.csv (time, com_x, com_y).')
    p.add_argument('--trajectory-file', type=Path, default=None,
                   help='Desired-path YAML (default: config/trajectory.yaml).')
    p.add_argument('--transient-seconds', type=float, default=10.0,
                   help='Sim seconds of settling to discard (default: 10.0, '
                        'matching the optimizer).')
    p.add_argument('--wall-duration', type=float, default=None,
                   help='Measured wall seconds the logged run covered; maps '
                        'sim->wall linearly. Preferred over --rtf.')
    p.add_argument('--rtf', type=float, default=None,
                   help='Real-time factor (wall = sim / rtf); used if '
                        '--wall-duration is omitted.')
    p.add_argument('--budget-seconds', type=float, default=300.0,
                   help='CMA --wait-seconds budget to mark (default: 300).')
    p.add_argument('--floor-frac', type=float, default=0.2,
                   help='Suggested min-progress = this fraction of the progress '
                        'reached at the budget (default: 0.2).')
    p.add_argument('--out', type=Path, default=Path('progress_vs_walltime.png'),
                   help='Output PNG (default: progress_vs_walltime.png).')
    args = p.parse_args()

    if args.trajectory_file is not None:
        oc.TRAJECTORY_FILE = args.trajectory_file
    polyline, _ = oc.load_desired_trajectory()

    df = pd.read_csv(args.csv).dropna(subset=['com_x', 'com_y'])
    df = df.reset_index(drop=True)
    if len(df) < 2:
        print(f'ERROR: {args.csv} has < 2 valid COM samples.', file=sys.stderr)
        return 1

    sim = df['time'].to_numpy()
    sim = sim - sim[0]
    com = df[['com_x', 'com_y']].to_numpy()
    _, s_at = oc._project_to_polyline(com, polyline)

    # Post-transient window, then progress = running (max - min) of the arc
    # projection — identical definition to optimize_cma.covered_arc_m.
    keep = sim >= args.transient_seconds
    if keep.sum() < 2:
        print(f'ERROR: < 2 samples after {args.transient_seconds}s transient.',
              file=sys.stderr)
        return 1
    sim_w = sim[keep]
    s_w = s_at[keep]
    progress = np.maximum.accumulate(s_w) - np.minimum.accumulate(s_w)

    # sim -> wall mapping
    if args.wall_duration is not None and sim[-1] > 0:
        scale = args.wall_duration / sim[-1]
        rtf = 1.0 / scale
        xmode = f'wall (--wall-duration {args.wall_duration:.0f}s, RTF~{rtf:.3f})'
    elif args.rtf is not None and args.rtf > 0:
        scale = 1.0 / args.rtf
        xmode = f'wall (--rtf {args.rtf})'
    else:
        scale = 1.0
        xmode = 'sim time (pass --wall-duration or --rtf for wall time)'
    x = sim_w * scale
    is_wall = scale != 1.0

    summary = (f'covered {progress[-1]:.3f} m over '
               f'{x[-1]:.0f}{"s wall" if is_wall else "s sim"} '
               f'(post-transient)')
    budget_prog = None
    if is_wall and args.budget_seconds <= x[-1]:
        budget_prog = float(np.interp(args.budget_seconds, x, progress))
    elif is_wall:
        print(f'NOTE: budget {args.budget_seconds:.0f}s exceeds the logged '
              f'{x[-1]:.0f}s wall — run longer to read the budget directly.',
              file=sys.stderr)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(x, progress, '-', color='#1f77b4', lw=1.6,
            label='arc covered along path')
    suggested = None
    if budget_prog is not None:
        suggested = args.floor_frac * budget_prog
        ax.axvline(args.budget_seconds, color='red', ls='--',
                   label=f'CMA budget {args.budget_seconds:.0f}s')
        ax.plot([args.budget_seconds], [budget_prog], 'o', color='red')
        ax.annotate(f'{budget_prog:.2f} m', (args.budget_seconds, budget_prog),
                    textcoords='offset points', xytext=(6, -12), color='red')
        ax.axhline(suggested, color='green', ls=':',
                   label=f'suggested min-progress ~ {suggested:.2f} m '
                         f'({args.floor_frac:.0%} of budget)')
    ax.set_xlabel('wall time [s]' if is_wall else 'sim time [s]')
    ax.set_ylabel('arc covered along path [m]')
    ax.set_title('Path progress vs ' + ('wall' if is_wall else 'sim') + ' time')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=9)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    plt.close(fig)

    print(f'x-axis: {xmode}')
    print(summary)
    if budget_prog is not None:
        print(f'progress at {args.budget_seconds:.0f}s budget: '
              f'{budget_prog:.3f} m')
        print(f'==> suggested --min-progress ~ {suggested:.2f} m '
              f'({args.floor_frac:.0%} of budget progress)')
    print(f'Saved chart to {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
