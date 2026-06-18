#!/usr/bin/env python3
"""
Per-epoch locomotion metrics for a CMA optimization session.

Unlike compute_sweep_metrics.py (which plots each metric against a swept grid
parameter), a CMA run has no single swept parameter — it evaluates a population
of candidates each generation ("epoch"). This script takes the *best* run of
each epoch and computes the same metrics, then plots them against the epoch so
you can see how the optimum's locomotion behavior evolves over the optimization.

For each generation in <cma_dir>/eval_log.csv:
  1. Select the evaluation with the minimum objective J (failed runs carry the
     J=1000 penalty, so they're naturally excluded). The within-generation row
     order maps positionally to evaluations/eval_<gen>_<index>.
  2. (Re)generate that run's avg-COM artifacts via compute_sweep_metrics.load_run
     and compute every metric in METRICS, plus the path-following metrics scored
     against the desired path (same yardstick as the CMA optimizer).

Outputs (in <cma_dir>):
    cma_summary.csv                            - epoch + best_eval + J + metrics
    cma_summary_<metric>_vs_epoch.png          - one chart per metric
    cma_summary_displacements_local_vs_epoch.png
    cma_summary_motion_vs_epoch.png

Usage (needs the scripts/.venv interpreter for the path-following metric):
    scripts/.venv/bin/python scripts/compute_cma_metrics.py <cma_dir> \
        [--skip-cycles 2] [--analysis-cycles 12] [--period 5.0] \
        [--transient-seconds 10.0] [-r]
"""
import argparse
import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

# compute_sweep_metrics lives next to this script; reuse its loaders, metric
# definitions, path-following scoring and plot styling rather than duplicating.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import compute_sweep_metrics as csm

EPOCH_COL = 'epoch'
# Axis label for the shared X axis on every chart.
csm.AXIS_LABELS.setdefault(EPOCH_COL, 'epoch (CMA generation)')


def select_best_runs(eval_log: pd.DataFrame, evaluations_dir: Path) -> list[dict]:
    """Per generation, pick the min-J evaluation and resolve its run directory.

    The global `eval` column repeats across resumes, so the row->directory
    mapping is positional: eval_<gen>_<i> is the i-th row of a generation. But a
    resume re-runs a generation from scratch, leaving the eval_log with more
    rows than directories while the on-disk dirs hold only the final attempt.
    Those final rows are the *last* n_dirs of the generation (verified by param
    match), so we map the tail positionally and pick min J among them.

    Returns dicts {'epoch', 'best_eval', 'J', 'dir'} sorted by epoch, skipping
    generations that have no directories on disk.
    """
    best = []
    for gen, group in eval_log.groupby('gen', sort=True):
        gen = int(gen)
        n_dirs = sum(1 for _ in evaluations_dir.glob(f'eval_{gen:03d}_*'))
        if n_dirs == 0:
            print(f'  gen {gen}: no evaluation dirs on disk — skipped',
                  file=sys.stderr)
            continue
        # Keep only the rows that have a matching directory (the final attempt).
        tail = group.tail(n_dirs).reset_index(drop=True)
        idx = int(tail['J'].idxmin())
        name = f'eval_{gen:03d}_{idx:02d}'
        run_dir = evaluations_dir / name
        if not run_dir.is_dir():
            print(f'  gen {gen}: {run_dir} not found — skipped', file=sys.stderr)
            continue
        best.append({
            EPOCH_COL: gen,
            'best_eval': name,
            'J': float(tail['J'].iloc[idx]),
            'dir': run_dir,
        })
    return best


def resolve_evaluations_dir(cma_dir: Path) -> Path | None:
    """Locate the directory holding the per-run eval_GGG_II subdirectories.

    Newer sessions nest them under <cma_dir>/evaluations/; older ones place
    them directly in <cma_dir>. Returns whichever contains eval_* dirs, or None.
    """
    nested = cma_dir / 'evaluations'
    if nested.is_dir() and any(nested.glob('eval_*_*')):
        return nested
    if any(cma_dir.glob('eval_*_*')):
        return cma_dir
    return None


def build_summary(cma_dir: Path, args: argparse.Namespace) -> pd.DataFrame | None:
    """Compute metrics for the best run of every epoch; write cma_summary.csv."""
    eval_log_path = cma_dir / 'eval_log.csv'
    evaluations_dir = resolve_evaluations_dir(cma_dir)
    if not eval_log_path.exists() or evaluations_dir is None:
        print(f'ERROR: {cma_dir} is not a CMA session '
              f'(needs eval_log.csv + eval_* run dirs)', file=sys.stderr)
        return None

    eval_log = pd.read_csv(eval_log_path)
    best_runs = select_best_runs(eval_log, evaluations_dir)
    if not best_runs:
        print(f'ERROR: no resolvable best runs in {cma_dir}', file=sys.stderr)
        return None

    # Desired path for the path-following metric (same resolution as the grid
    # sweep: explicit file, then <cma_dir>/trajectory.yaml, then the default).
    polyline, traj_src = csm.resolve_polyline(cma_dir, args.trajectory_file)
    if polyline is not None:
        print(f'Path-following metric scored against {traj_src}')

    rows = []
    for entry in best_runs:
        run = csm.load_run(entry['dir'], args.skip_cycles, args.analysis_cycles,
                           args.period, write_outputs=True,
                           desired_path=polyline)
        if run is None:
            print(f'  {entry["best_eval"]}: missing trajectory/params or no '
                  f'avg-COM points — skipped', file=sys.stderr)
            continue

        row = {EPOCH_COL: entry[EPOCH_COL], 'best_eval': entry['best_eval'],
               'J': entry['J']}
        for name, fn in csm.METRICS.items():
            try:
                row[name] = fn(run['avg_df'])
            except Exception as e:
                print(f'  {entry["best_eval"]}: metric {name} failed: {e!r}',
                      file=sys.stderr)
                row[name] = float('nan')
        if polyline is not None and run.get('traj_csv') is not None:
            row.update(csm.compute_path_metrics(
                run['traj_csv'], polyline, args.transient_seconds))
        rows.append(row)

    if not rows:
        print(f'ERROR: no usable runs in {cma_dir}', file=sys.stderr)
        return None

    summary_df = pd.DataFrame(rows).sort_values(EPOCH_COL).reset_index(drop=True)
    summary_csv = cma_dir / 'cma_summary.csv'
    summary_df.to_csv(summary_csv, index=False)
    print(f'Loaded {len(summary_df)} epochs from {cma_dir}')
    print(f'Wrote {summary_csv}')
    return summary_df


def plot_per_metric(summary_df: pd.DataFrame, cma_dir: Path) -> None:
    """One chart per metric: metric value vs epoch (best run of each epoch)."""
    metric_cols = list(csm.METRICS) + [c for c in csm.PATH_METRIC_KEYS
                                       if c in summary_df.columns]
    for metric in metric_cols:
        sub = summary_df[[EPOCH_COL, metric]].dropna().sort_values(EPOCH_COL)
        if sub.empty:
            continue
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(sub[EPOCH_COL], sub[metric], 'o-',
                color=csm.metric_color(metric), ms=4)
        ax.set_xlabel(csm.axis_label(EPOCH_COL))
        ax.set_ylabel(csm.axis_label(metric))
        ax.grid(True, alpha=0.3)

        png = cma_dir / f'cma_summary_{metric}_vs_epoch.png'
        fig.savefig(png, dpi=120, bbox_inches='tight')
        plt.close(fig)
        print(f'Wrote {png}')


def plot_combined_displacements(summary_df: pd.DataFrame, cma_dir: Path) -> None:
    """displacement_x_local and displacement_y_local as two series vs epoch."""
    disp_x_col, disp_y_col = 'displacement_x_local', 'displacement_y_local'
    if not {disp_x_col, disp_y_col}.issubset(summary_df.columns):
        return

    sub_x = summary_df[[EPOCH_COL, disp_x_col]].dropna().sort_values(EPOCH_COL)
    sub_y = summary_df[[EPOCH_COL, disp_y_col]].dropna().sort_values(EPOCH_COL)
    if sub_x.empty and sub_y.empty:
        return

    fig, ax = plt.subplots(figsize=(8, 5))
    if not sub_x.empty:
        ax.plot(sub_x[EPOCH_COL], sub_x[disp_x_col], 'o-',
                color=csm.metric_color(disp_x_col), ms=4,
                label=csm.legend_label(disp_x_col))
    if not sub_y.empty:
        ax.plot(sub_y[EPOCH_COL], sub_y[disp_y_col], 'o-',
                color=csm.metric_color(disp_y_col), ms=4,
                label=csm.legend_label(disp_y_col))
    ax.set_xlabel(csm.axis_label(EPOCH_COL))
    ax.set_ylabel(csm.axis_label('displacement_local'))
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize='small')

    png = cma_dir / 'cma_summary_displacements_local_vs_epoch.png'
    fig.savefig(png, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Wrote {png}')


def plot_stacked_motion(summary_df: pd.DataFrame, cma_dir: Path) -> None:
    """Combined displacements (top) + orientation change (bottom) vs epoch."""
    disp_x_col, disp_y_col = 'displacement_x_local', 'displacement_y_local'
    ori_col = 'orientation_change_deg'
    if not {disp_x_col, disp_y_col, ori_col}.issubset(summary_df.columns):
        return

    sub_x = summary_df[[EPOCH_COL, disp_x_col]].dropna().sort_values(EPOCH_COL)
    sub_y = summary_df[[EPOCH_COL, disp_y_col]].dropna().sort_values(EPOCH_COL)
    sub_ori = summary_df[[EPOCH_COL, ori_col]].dropna().sort_values(EPOCH_COL)
    if sub_x.empty and sub_y.empty and sub_ori.empty:
        return

    fig, (ax_disp, ax_ori) = plt.subplots(2, 1, sharex=True, figsize=(8, 8))
    if not sub_x.empty:
        ax_disp.plot(sub_x[EPOCH_COL], sub_x[disp_x_col], 'o-',
                     color=csm.metric_color(disp_x_col), ms=4,
                     label=csm.legend_label(disp_x_col))
    if not sub_y.empty:
        ax_disp.plot(sub_y[EPOCH_COL], sub_y[disp_y_col], 'o-',
                     color=csm.metric_color(disp_y_col), ms=4,
                     label=csm.legend_label(disp_y_col))
    if not sub_ori.empty:
        ax_ori.plot(sub_ori[EPOCH_COL], sub_ori[ori_col], 'o-',
                    color=csm.metric_color(ori_col), ms=4)

    ax_disp.set_ylabel(csm.axis_label('displacement_local'))
    ax_disp.grid(True, alpha=0.3)
    ax_disp.legend(loc='best', fontsize='small')
    ax_ori.set_xlabel(csm.axis_label(EPOCH_COL))
    ax_ori.set_ylabel(csm.axis_label(ori_col))
    ax_ori.grid(True, alpha=0.3)

    png = cma_dir / 'cma_summary_motion_vs_epoch.png'
    fig.savefig(png, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'Wrote {png}')


def process_session(cma_dir: Path, args: argparse.Namespace) -> int:
    """Full pipeline for one CMA session dir. Returns 0 / 1 like main()."""
    summary_df = build_summary(cma_dir, args)
    if summary_df is None:
        return 1
    plot_per_metric(summary_df, cma_dir)
    plot_combined_displacements(summary_df, cma_dir)
    plot_stacked_motion(summary_df, cma_dir)
    return 0


def _is_cma_session(d: Path) -> bool:
    return (d.is_dir() and (d / 'eval_log.csv').exists()
            and resolve_evaluations_dir(d) is not None)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Compute per-epoch metrics for the best run of each CMA '
                    'generation and plot them against the epoch.')
    parser.add_argument('cma_dir', type=Path,
                        help='CMA session directory (contains eval_log.csv and '
                             'evaluations/), or a parent of such dirs with -r.')
    parser.add_argument('--skip-cycles', type=float, default=2.0,
                        help='Cycles of transient to skip for the avg-COM '
                             'metrics (default: 2.0)')
    parser.add_argument('--analysis-cycles', type=int, default=12,
                        help='Number of averaged windows to compute (default: 12)')
    parser.add_argument('--period', type=float, default=5.0,
                        help='Controller period in seconds (default: 5.0)')
    parser.add_argument('--transient-seconds', type=float, default=10.0,
                        help='Warm-up discarded before scoring the path-following '
                             'metric. Default 10.0 matches optimize_cma.')
    parser.add_argument('--trajectory-file', type=Path, default=None,
                        help='Desired-path YAML for the path-following metric. '
                             'Default: <cma_dir>/trajectory.yaml, else '
                             'config/trajectory.yaml.')
    parser.add_argument('-r', '--recursive', action='store_true',
                        help='Treat <cma_dir> as a parent of multiple CMA '
                             'sessions and process each immediate subdirectory '
                             'that looks like a session.')
    args = parser.parse_args()

    if not args.cma_dir.is_dir():
        print(f'ERROR: {args.cma_dir} is not a directory', file=sys.stderr)
        return 1

    if not args.recursive:
        return process_session(args.cma_dir, args)

    targets = sorted(p for p in args.cma_dir.iterdir() if _is_cma_session(p))
    if not targets:
        print(f'ERROR: no CMA session subdirectories under {args.cma_dir}',
              file=sys.stderr)
        return 1

    print(f'Recursive mode: processing {len(targets)} session(s) under '
          f'{args.cma_dir}')
    failures = 0
    for sub in targets:
        print(f'\n=== {sub.name} ===')
        try:
            if process_session(sub, args) != 0:
                failures += 1
        finally:
            plt.close('all')

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
