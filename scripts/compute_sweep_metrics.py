#!/usr/bin/env python3
"""
Compute per-run averaged COM (delegating to compute_avg_com), aggregate
metrics across the sweep, and plot them vs the swept parameter.

For each run subdirectory under <sweep_dir>:
  1. Find <run>_body_trajectory.csv and call compute_avg_com.process_run()
     to (re)generate <run>_avg_com.csv and <run>_avg_com.png with the given
     skip_cycles / analysis_cycles / period.
  2. Load the resulting avg-COM points and the run's <run>_params.yaml
     (effective controller params, written by run_sweep.py).
  3. Compute one or more metrics from the avg-COM points (currently:
     distance from the first to the last averaged COM point).

Then identify which controller parameters changed across runs (i.e. are not
constant). For each (changing_param, metric) pair, write a summary CSV and
plot the metric against the parameter.

Outputs:
  Per-run (in each run subdirectory):
    <run>_avg_com.csv                            - from compute_avg_com
    <run>_avg_com.png                            - from compute_avg_com
  Sweep-wide (in <sweep_dir>):
    sweep_summary.csv                            - run_name + changing params + metrics
    sweep_summary_<metric>_vs_<param>.png        - chart per (metric, changing param)

Usage:
    python3 scripts/compute_sweep_metrics.py <sweep_dir> \
        [--skip-cycles 5] [--analysis-cycles 5] [--period 5.0]
"""
import argparse
import math
import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import yaml

# compute_avg_com lives next to this script.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from compute_avg_com import process_run as compute_avg_com_for_run


CONTROLLER_NODE_NAME = 'movement_controller_node'

# Float tolerance when deciding whether a parameter is "constant" across runs.
PARAM_EQ_ATOL = 1e-3
PARAM_EQ_RTOL = 1e-3

# When --scan-skip-cycles is enabled, repeat the metric computation for
# this many skip_cycles values, starting from the user-supplied --skip-cycles
# and stepping by SCAN_SKIP_STEP.
SCAN_SKIP_COUNT = 10
SCAN_SKIP_STEP = 0.1

# Controller params stored in radians that are easier to read in degrees.
# When one of these is the swept parameter, the summary CSV column and the
# plot X axis use degrees and the column name gets a "_deg" suffix.
ANGLE_PARAMS_DEG = {
    'A_v', 'A_h',
    'delta_phi_v', 'delta_phi_h', 'delta_phi_vh',
    'O_v', 'O_h',
}

# Derived parameters: name -> (source_param, divisor).
#   delta_phi_v = k_v * 0.8976
#   delta_phi_h = k_h * 1.0472
# Both relations are physical (driven by joint counts), so the constants are
# hardcoded rather than declared per-sweep. When a derived form is exposed,
# its source param is dropped from the plotted dimensions.
DERIVED_PARAMS = {
    'k_v': ('delta_phi_v', 0.8976),
    'k_h': ('delta_phi_h', 1.0472),
}
SOURCE_PARAMS = {src for src, _ in DERIVED_PARAMS.values()}
K_ALIAS = 'k'


def display_column(name: str) -> str:
    return f'{name}_deg' if name in ANGLE_PARAMS_DEG else name


def display_value(name: str, value):
    if value is None or name not in ANGLE_PARAMS_DEG:
        return value
    try:
        return math.degrees(float(value))
    except (TypeError, ValueError):
        return value


def augment_with_derived(params: dict) -> dict:
    """Return a copy of params with k_v / k_h derived from delta_phi_v / delta_phi_h."""
    out = dict(params)
    for name, (src, divisor) in DERIVED_PARAMS.items():
        val = params.get(src)
        if isinstance(val, (int, float)) and divisor != 0:
            out[name] = float(val) / divisor
    return out


def should_alias_k(runs: list[dict]) -> bool:
    """True if k_v and k_h are equal in every run — then we can show one 'k'."""
    for r in runs:
        kv = r['params'].get('k_v')
        kh = r['params'].get('k_h')
        if kv is None or kh is None:
            return False
        if not math.isclose(kv, kh, abs_tol=PARAM_EQ_ATOL, rel_tol=PARAM_EQ_RTOL):
            return False
    return True


def apply_alias_k(params: dict) -> dict:
    """Replace k_v and k_h with a single 'k' (= k_v) in a copy of params."""
    out = dict(params)
    if 'k_v' in out:
        out[K_ALIAS] = out.pop('k_v')
    out.pop('k_h', None)
    return out


def drop_redundant_sources(changing: list[str]) -> list[str]:
    """Drop delta_phi_v / delta_phi_h from `changing` when k_v / k_h / k is also there."""
    has_derived = (K_ALIAS in changing
                   or any(name in changing for name in DERIVED_PARAMS.keys()))
    if not has_derived:
        return list(changing)
    return [k for k in changing if k not in SOURCE_PARAMS]


def metric_distance_first_to_last(avg_df: pd.DataFrame) -> float:
    """Average per-cycle net straight-line displacement (m).

    |point[last] - point[first]| / (n_cycles - 1). Reflects the net drift
    rate over the analysis window — how far the COM moves *on average*
    each cycle when only the endpoints are considered.
    """
    if len(avg_df) < 2:
        return float('nan')
    valid = avg_df.dropna(subset=['com_x', 'com_y'])
    if len(valid) < 2:
        return float('nan')
    dx = valid['com_x'].iloc[-1] - valid['com_x'].iloc[0]
    dy = valid['com_y'].iloc[-1] - valid['com_y'].iloc[0]
    return math.hypot(dx, dy) / (len(valid) - 1)


def metric_distance_consecutive_sum(avg_df: pd.DataFrame) -> float:
    """Average per-cycle path length (m) — mean of consecutive distances."""
    if len(avg_df) < 2:
        return float('nan')
    valid = avg_df.dropna(subset=['com_x', 'com_y'])
    if len(valid) < 2:
        return float('nan')
    dx = valid['com_x'].diff().iloc[1:]
    dy = valid['com_y'].diff().iloc[1:]
    return float((dx ** 2 + dy ** 2).pow(0.5).mean())


def metric_orientation_change_rad(avg_df: pd.DataFrame) -> float:
    """Average per-cycle body orientation change (rad).

    (theta[last] - theta[first]) / (n_cycles - 1). Uses the
    branch-continuous PCA principal-axis angle written by compute_avg_com,
    so the value remains meaningful for multi-revolution turns.
    """
    if 'orientation_rad' not in avg_df.columns:
        return float('nan')
    valid = avg_df.dropna(subset=['orientation_rad'])
    if len(valid) < 2:
        return float('nan')
    return float((valid['orientation_rad'].iloc[-1]
                  - valid['orientation_rad'].iloc[0]) / (len(valid) - 1))


def metric_displacement_x(avg_df: pd.DataFrame) -> float:
    """Average per-cycle x displacement (m) of the COM."""
    if len(avg_df) < 2:
        return float('nan')
    valid = avg_df.dropna(subset=['com_x'])
    if len(valid) < 2:
        return float('nan')
    return float((valid['com_x'].iloc[-1] - valid['com_x'].iloc[0])
                 / (len(valid) - 1))


def metric_displacement_y(avg_df: pd.DataFrame) -> float:
    """Average per-cycle y displacement (m) of the COM."""
    if len(avg_df) < 2:
        return float('nan')
    valid = avg_df.dropna(subset=['com_y'])
    if len(valid) < 2:
        return float('nan')
    return float((valid['com_y'].iloc[-1] - valid['com_y'].iloc[0])
                 / (len(valid) - 1))


def _displacement_local(avg_df: pd.DataFrame) -> tuple[float, float]:
    """Average per-cycle world (dx, dy) rotated into the robot's initial frame.

    Uses orientation_relative_rad (which is 0 at t=0 by construction), so at
    skip_cycles=0 theta_0 ~ 0 and the local-frame components match the
    world-frame ones. For larger skip values the rotation reflects how much
    the robot has turned during the skipped phase. Falls back to the absolute
    orientation_rad when older avg_com.csv files don't have the relative
    column yet.
    """
    if len(avg_df) < 2:
        return float('nan'), float('nan')
    angle_col = ('orientation_relative_rad'
                 if 'orientation_relative_rad' in avg_df.columns
                 else 'orientation_rad')
    if angle_col not in avg_df.columns:
        return float('nan'), float('nan')
    valid = avg_df.dropna(subset=['com_x', 'com_y', angle_col])
    if len(valid) < 2:
        return float('nan'), float('nan')
    n_intervals = len(valid) - 1
    dx = (valid['com_x'].iloc[-1] - valid['com_x'].iloc[0]) / n_intervals
    dy = (valid['com_y'].iloc[-1] - valid['com_y'].iloc[0]) / n_intervals
    theta0 = float(valid[angle_col].iloc[0])
    cos_t, sin_t = math.cos(theta0), math.sin(theta0)
    return (dx * cos_t + dy * sin_t,    # local x: along initial body axis
            -dx * sin_t + dy * cos_t)   # local y: perpendicular to it


def metric_displacement_x_local(avg_df: pd.DataFrame) -> float:
    """X displacement (m) in the robot frame at the start of the analysis window."""
    return _displacement_local(avg_df)[0]


def metric_displacement_y_local(avg_df: pd.DataFrame) -> float:
    """Y displacement (m) in the robot frame at the start of the analysis window."""
    return _displacement_local(avg_df)[1]


# Add metrics here. Each takes the averaged-COM DataFrame and returns a float.
METRICS = {
    'distance_first_to_last': metric_distance_first_to_last,
    # 'distance_consecutive_sum': metric_distance_consecutive_sum,
    'orientation_change_rad': metric_orientation_change_rad,
    # 'displacement_x': metric_displacement_x,
    # 'displacement_y': metric_displacement_y,
    'displacement_x_local': metric_displacement_x_local,
    'displacement_y_local': metric_displacement_y_local,
}


def load_run(run_dir: Path, skip_cycles: float, analysis_cycles: int,
             period: float, write_outputs: bool = True) -> dict | None:
    """(Re)generate avg-COM artifacts, then load them with the run's params.

    Returns {'name', 'params', 'avg_df'} or None if the body trajectory or
    params YAML is missing, or if compute_avg_com produced no rows.
    """
    traj_csvs = sorted(run_dir.glob('*_body_trajectory.csv'))
    param_yamls = sorted(run_dir.glob('*_params.yaml'))
    if not traj_csvs or not param_yamls:
        return None

    avg_df = compute_avg_com_for_run(traj_csvs[0], skip_cycles, analysis_cycles,
                                     period, write_outputs=write_outputs)
    if avg_df is None or avg_df.empty:
        return None

    with open(param_yamls[0]) as f:
        params = (yaml.safe_load(f) or {}) \
            .get(CONTROLLER_NODE_NAME, {}) \
            .get('ros__parameters', {}) or {}

    return {'name': run_dir.name,
            'params': augment_with_derived(params),
            'avg_df': avg_df}


def values_equal(values: list) -> bool:
    """True if every value is the same. Tolerates floats within tolerance."""
    if len(values) <= 1:
        return True
    first = values[0]
    for v in values[1:]:
        if isinstance(first, float) and isinstance(v, (int, float)):
            if not math.isclose(float(first), float(v),
                                abs_tol=PARAM_EQ_ATOL, rel_tol=PARAM_EQ_RTOL):
                return False
        elif first != v:
            return False
    return True


def find_changing_params(runs: list[dict]) -> list[str]:
    """Param keys whose value differs across at least two runs.

    Keys missing from some runs are also treated as changing.
    """
    all_keys = set()
    for r in runs:
        all_keys.update(r['params'].keys())

    sentinel = object()
    changing = []
    for k in sorted(all_keys):
        values = [r['params'].get(k, sentinel) for r in runs]
        if not values_equal(values):
            changing.append(k)
    return changing


def _normalize_yaml_value(v):
    """Round floats to 6 decimals so the YAML output stays clean."""
    if isinstance(v, float):
        return round(v, 6)
    return v


def _annotate_angles_with_degrees(yaml_text: str) -> str:
    """Append '# X degrees' to value lines for params listed in ANGLE_PARAMS_DEG.

    Handles both scalar params at indent 4 ("    delta_phi_vh: 1.5708")
    and list items at indent 6 inside a varies block
    ("      - 1.5708"). Tracks the current param so list items know which
    key they belong to.
    """
    out_lines = []
    current_param: str | None = None

    for line in yaml_text.splitlines():
        stripped = line.lstrip()
        indent = len(line) - len(stripped)
        annotated = line

        # Controller-param key line at indent 4.
        if (indent == 4 and not stripped.startswith('-')
                and ':' in stripped):
            head, _, tail = stripped.partition(':')
            head = head.strip()
            tail = tail.strip()
            if tail:
                # Scalar param on a single line.
                current_param = None
                if head in ANGLE_PARAMS_DEG:
                    try:
                        deg = math.degrees(float(tail))
                        annotated = f'{line}  # {round(deg, 1):g} degrees'
                    except ValueError:
                        pass
            else:
                # Mapping param (a `varies` block follows).
                current_param = head

        # List item inside a varies values: block at indent 6.
        elif (indent == 6 and stripped.startswith('- ')
              and current_param in ANGLE_PARAMS_DEG):
            value = stripped[2:].strip()
            try:
                deg = math.degrees(float(value))
                annotated = f'{line}  # {round(deg, 1):g} degrees'
            except ValueError:
                pass

        out_lines.append(annotated)

    return '\n'.join(out_lines) + '\n'


def write_parameters_summary(sweep_dir: Path, runs: list[dict]) -> None:
    """Write <sweep_dir>/sweep_parameters.yaml describing the controller params.

    Constant params are stored as their single value; params that varied
    across runs are stored as {varies: true, values: [sorted unique vals]}.
    Derived params (k_v, k_h, k) are excluded — this file mirrors the raw
    controller config layout from <run>_params.yaml. Values of params in
    ANGLE_PARAMS_DEG get inline "# X degrees" annotations.
    """
    skip_keys = set(DERIVED_PARAMS.keys()) | {K_ALIAS}

    all_keys = set()
    for r in runs:
        all_keys.update(r['params'].keys())
    all_keys -= skip_keys

    summary: dict = {}
    for k in sorted(all_keys):
        values = [r['params'].get(k) for r in runs]
        if values_equal(values):
            first = next((v for v in values if v is not None), None)
            summary[k] = _normalize_yaml_value(first)
        else:
            unique = list({_normalize_yaml_value(v) for v in values
                           if v is not None})
            try:
                unique.sort()
            except TypeError:
                unique.sort(key=str)
            summary[k] = {'varies': True, 'values': unique}

    out = {CONTROLLER_NODE_NAME: {'ros__parameters': summary}}
    yaml_text = yaml.safe_dump(out, default_flow_style=False, sort_keys=False)
    annotated = _annotate_angles_with_degrees(yaml_text)

    out_path = sweep_dir / 'sweep_parameters.yaml'
    with open(out_path, 'w') as f:
        f.write(annotated)
    print(f'Wrote {out_path}')


def _plot_displacement_3d(summary_df: pd.DataFrame,
                          changing_display: list[str],
                          skip_values: list[float],
                          args: argparse.Namespace,
                          *, x_col: str, y_col: str,
                          title_prefix: str, file_slug: str) -> bool:
    """Render one 3D chart per changing param using (x_col, y_col, param) axes.

    In --scan-skip-cycles mode each offset is drawn as its own viridis-colored
    line. Figures are saved to PNG and left open (caller calls plt.show()).
    Returns True if at least one chart was actually drawn and saved.
    """
    if not {x_col, y_col}.issubset(summary_df.columns):
        return False

    drew_anything = False
    for param_col in changing_display:
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection='3d')
        drew_any = False
        cols = [x_col, y_col, param_col]

        if args.scan_skip_cycles:
            cmap = plt.get_cmap('viridis')
            denom = max(len(skip_values) - 1, 1)
            for i, skip_val in enumerate(skip_values):
                sub = summary_df[summary_df['skip_cycles'] == skip_val][cols] \
                    .dropna().sort_values(param_col)
                if sub.empty:
                    continue
                ax.plot(sub[x_col], sub[y_col], sub[param_col], 'o-',
                        color=cmap(i / denom), ms=4,
                        label=f'skip_cycles={skip_val:.1f}')
                drew_any = True
            if drew_any:
                ax.legend(loc='best', fontsize='small', ncol=2)
        else:
            sub = summary_df[cols].dropna().sort_values(param_col)
            if not sub.empty:
                ax.plot(sub[x_col], sub[y_col], sub[param_col], 'o-',
                        color='tab:blue')
                drew_any = True

        if not drew_any:
            plt.close(fig)
            continue

        ax.set_xlabel(f'{x_col} [m]')
        ax.set_ylabel(f'{y_col} [m]')
        ax.set_zlabel(param_col)
        ax.set_title(f'{title_prefix} vs {param_col}  ({args.sweep_dir.name})')

        png = args.sweep_dir / f'sweep_summary_{file_slug}_vs_{param_col}.png'
        fig.savefig(png, dpi=120, bbox_inches='tight')
        print(f'Wrote {png}')
        drew_anything = True

    return drew_anything


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Aggregate per-run metrics across a sweep and plot them.')
    parser.add_argument('sweep_dir', type=Path,
                        help='Sweep output directory (contains per-run subdirs)')
    parser.add_argument('--skip-cycles', type=float, default=2.0,
                        help='Cycles of transient to skip (default: 5)')
    parser.add_argument('--analysis-cycles', type=int, default=12,
                        help='Number of averaged windows to compute (default: 5)')
    parser.add_argument('--period', type=float, default=5.0,
                        help='Controller period in seconds (default: 5.0)')
    parser.add_argument('--scan-skip-cycles', action='store_true',
                        help=f'Repeat metric computation for {SCAN_SKIP_COUNT} '
                             f'skip_cycles values (base, base+{SCAN_SKIP_STEP}, '
                             f'..., base+{SCAN_SKIP_STEP*(SCAN_SKIP_COUNT-1):.1f}) '
                             f'and overlay them as separate series on each chart.')
    args = parser.parse_args()

    if not args.sweep_dir.is_dir():
        print(f'ERROR: {args.sweep_dir} is not a directory', file=sys.stderr)
        return 1

    run_dirs = sorted(p for p in args.sweep_dir.iterdir() if p.is_dir())
    if not run_dirs:
        print(f'ERROR: no run subdirectories in {args.sweep_dir}', file=sys.stderr)
        return 1

    skip_values = (
        [args.skip_cycles + i * SCAN_SKIP_STEP for i in range(SCAN_SKIP_COUNT)]
        if args.scan_skip_cycles else [args.skip_cycles]
    )

    # First pass: load each run at the base offset. This is the only pass
    # that writes per-run avg_com.csv/.png — additional offsets in scan
    # mode are computed in memory only.
    base_runs: list[tuple[Path, dict]] = []
    for d in run_dirs:
        loaded = load_run(d, args.skip_cycles, args.analysis_cycles,
                          args.period, write_outputs=True)
        if loaded is None:
            print(f'  {d.name}: missing body_trajectory.csv or params.yaml, '
                  f'or no avg-COM points produced — skipped', file=sys.stderr)
            continue
        base_runs.append((d, loaded))

    if not base_runs:
        print(f'ERROR: no usable runs in {args.sweep_dir}', file=sys.stderr)
        return 1

    # If k_v and k_h match in every run, collapse them into a single 'k'
    # column so the swept dimension is reported as one variable. The decision
    # is sticky for the rest of the script — including any extra runs loaded
    # later for the skip_cycles scan.
    alias_k = should_alias_k([r for _, r in base_runs])
    if alias_k:
        for _, r in base_runs:
            r['params'] = apply_alias_k(r['params'])

    changing = find_changing_params([r for _, r in base_runs])
    changing = drop_redundant_sources(changing)
    changing_display = [display_column(k) for k in changing]

    print(f'Loaded {len(base_runs)} runs from {args.sweep_dir}')
    if changing:
        print(f'Changing parameters: {", ".join(changing)}')
    else:
        print('No parameters varied across runs; summary will still be written.')
    if alias_k:
        print('k_v == k_h across all runs — exposing a single "k" column.')
    if args.scan_skip_cycles:
        print(f'Scanning skip_cycles over {len(skip_values)} values: '
              f'{skip_values[0]:.2f} ... {skip_values[-1]:.2f} '
              f'(step {SCAN_SKIP_STEP})')

    # Build the summary table. One row per (skip_value, run); in non-scan
    # mode the skip_cycles column is omitted so the output matches the
    # pre-scan format byte-for-byte.
    rows = []
    for i, skip_val in enumerate(skip_values):
        for d, base_loaded in base_runs:
            if i == 0:
                run_data = base_loaded
            else:
                run_data = load_run(d, skip_val, args.analysis_cycles,
                                    args.period, write_outputs=False)
                if run_data is None:
                    continue
                if alias_k:
                    run_data['params'] = apply_alias_k(run_data['params'])

            row = {'run_name': run_data['name']}
            for k in changing:
                row[display_column(k)] = display_value(k, run_data['params'].get(k))
            for name, fn in METRICS.items():
                try:
                    row[name] = fn(run_data['avg_df'])
                except Exception as e:
                    print(f'  {run_data["name"]}: metric {name} failed: {e!r}',
                          file=sys.stderr)
                    row[name] = float('nan')
            if args.scan_skip_cycles:
                row['skip_cycles'] = skip_val
            rows.append(row)

    summary_df = pd.DataFrame(rows)
    # Sort by the first changing param when present, else by run name —
    # so the CSV reads in a useful order and the plot lines are monotone.
    # In scan mode, sort by skip_cycles first to keep each series contiguous.
    sort_keys = []
    if args.scan_skip_cycles:
        sort_keys.append('skip_cycles')
    sort_keys.append(changing_display[0] if changing_display else 'run_name')
    summary_df = summary_df.sort_values(sort_keys, kind='stable').reset_index(drop=True)

    summary_csv = args.sweep_dir / 'sweep_summary.csv'
    summary_df.to_csv(summary_csv, index=False)
    print(f'Wrote {summary_csv}')

    write_parameters_summary(args.sweep_dir, [r for _, r in base_runs])

    if not changing_display:
        print('Skipping plots: no changing parameter to put on the X axis.')
        return 0

    for metric in METRICS:
        for param_col in changing_display:
            fig, ax = plt.subplots(figsize=(8, 5))
            drew_any = False

            if args.scan_skip_cycles:
                cmap = plt.get_cmap('viridis')
                denom = max(len(skip_values) - 1, 1)
                for i, skip_val in enumerate(skip_values):
                    sub = summary_df[summary_df['skip_cycles'] == skip_val] \
                        [[param_col, metric]].dropna().sort_values(param_col)
                    if sub.empty:
                        continue
                    ax.plot(sub[param_col], sub[metric], 'o',
                            color=cmap(i / denom), ms=4,
                            label=f'skip_cycles={skip_val:.1f}')
                    drew_any = True
                if drew_any:
                    ax.legend(loc='best', fontsize='small', ncol=2)
            else:
                sub = summary_df[[param_col, metric]].dropna().sort_values(param_col)
                if not sub.empty:
                    ax.plot(sub[param_col], sub[metric], 'o', color='tab:blue')
                    drew_any = True

            if not drew_any:
                plt.close(fig)
                continue

            ax.set_xlabel(param_col)
            ax.set_ylabel(metric)
            ax.set_title(f'{metric} vs {param_col}  ({args.sweep_dir.name})')
            ax.grid(True, alpha=0.3)

            png = args.sweep_dir / f'sweep_summary_{metric}_vs_{param_col}.png'
            fig.savefig(png, dpi=120, bbox_inches='tight')
            plt.close(fig)
            print(f'Wrote {png}')

    # 3D scatter charts: end-displacement components on the X/Y axes and the
    # swept controller param on Z. Two flavors:
    #   - world frame:  raw (com_x[last]-com_x[first], com_y[last]-com_y[first])
    #   - robot frame:  the same vector rotated into the body frame at the
    #                   start of the analysis window — meaningful when the
    #                   robot has already turned during the skip phase.
    # Figures stay open after saving so plt.show() at the end opens them as
    # interactive windows.
    shown_any_3d = False
    shown_any_3d |= _plot_displacement_3d(
        summary_df, changing_display, skip_values, args,
        x_col='displacement_x', y_col='displacement_y',
        title_prefix='end displacement (world frame)',
        file_slug='displacement3d',
    )
    shown_any_3d |= _plot_displacement_3d(
        summary_df, changing_display, skip_values, args,
        x_col='displacement_x_local', y_col='displacement_y_local',
        title_prefix='end displacement (robot frame at window start)',
        file_slug='displacement_local3d',
    )

    if shown_any_3d:
        print('Opening 3D chart(s); close the window(s) to exit.')
        plt.show()

    return 0


if __name__ == '__main__':
    sys.exit(main())
