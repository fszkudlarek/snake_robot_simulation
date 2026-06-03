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

# Axis labels for plots. Keys are column names as they appear in
# sweep_summary.csv (post-display_column transform — so angle params use
# the '_deg' suffix). Values are matplotlib mathtext strings; the raw
# column name is used as a fallback when a key isn't listed.
# Extend this dict to customize how parameters and metrics are labeled
# on chart axes.
AXIS_LABELS = {
    'delta_phi_vh_deg': r'$\Delta\Phi_{VH}\,[^\circ]$',
    'displacement_x_local': r'$\Delta x_{avg}\,[m]$',
    'displacement_y_local': r'$\Delta y_{avg}\,[m]$',
    'orientation_change_deg': r'$\Delta\theta_{avg}\,[^\circ]$',
    # Used for the combined displacement_x_local + displacement_y_local chart.
    'displacement_local': r'distance$\,[m]$',
    # Unified k alias — only present when k_v == k_h across every run.
    'k': r'$k_V = k_H$',
    'k_v': r'$k_V$',
    'k_h': r'$k_H=\frac{k_V}{2}$',
    'O_v_deg': r'$O_V$',
    'O_h_deg': r'$O_H$',
    'A_v_deg': r'$A_V$',
    'A_h_deg': r'$A_H$',
}

# Legend labels — used when a column appears as a *series* on a chart
# rather than as the axis itself. Units are typically dropped here since
# they already appear on the axis label. Falls back to AXIS_LABELS, then
# to the bare column name.
LEGEND_LABELS = {
    'displacement_x_local': r'$\Delta x_{avg}$',
    'displacement_y_local': r'$\Delta y_{avg}$',
}

# Per-metric chart colors. Keyed by column name (same keys as the METRICS
# dict). Used by both the per-metric 2D charts and the combined
# displacements chart, so colors stay consistent across the whole sweep.
# Unlisted metrics get DEFAULT_METRIC_COLOR.
METRIC_COLORS = {
    'displacement_x_local': 'tab:blue',
    'displacement_y_local': 'tab:orange',
    'orientation_change_deg': 'tab:green',
}
DEFAULT_METRIC_COLOR = 'tab:blue'


def axis_label(col: str) -> str:
    return AXIS_LABELS.get(col, col)


def legend_label(col: str) -> str:
    if col in LEGEND_LABELS:
        return LEGEND_LABELS[col]
    return AXIS_LABELS.get(col, col)


def metric_color(name: str) -> str:
    return METRIC_COLORS.get(name, DEFAULT_METRIC_COLOR)


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


def metric_orientation_change_deg(avg_df: pd.DataFrame) -> float:
    """Average per-cycle body orientation change (deg).

    (theta[last] - theta[first]) / (n_cycles - 1), converted to degrees.
    Uses the branch-continuous PCA principal-axis angle written by
    compute_avg_com, so the value remains meaningful for multi-revolution
    turns.
    """
    if 'orientation_rad' not in avg_df.columns:
        return float('nan')
    valid = avg_df.dropna(subset=['orientation_rad'])
    if len(valid) < 2:
        return float('nan')
    rad = (valid['orientation_rad'].iloc[-1]
           - valid['orientation_rad'].iloc[0]) / (len(valid) - 1)
    return math.degrees(float(rad))


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
    'orientation_change_deg': metric_orientation_change_deg,
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


def _has_sweep_runs(d: Path) -> bool:
    """True if `d` contains at least one subdirectory with a body_trajectory CSV."""
    if not d.is_dir():
        return False
    for child in d.iterdir():
        if child.is_dir() and any(child.glob('*_body_trajectory.csv')):
            return True
    return False


def resolve_sweep_runs_dir(input_dir: Path) -> Path | None:
    """Locate the directory that actually holds the per-run subdirectories.

    Tries `input_dir` first; if no subdirectory there contains a
    `*_body_trajectory.csv`, falls back to `<input_dir>/runs`. Returns the
    matching path or None if neither location has any sweep runs.
    """
    if _has_sweep_runs(input_dir):
        return input_dir
    runs_dir = input_dir / 'runs'
    if _has_sweep_runs(runs_dir):
        return runs_dir
    return None


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
                          file_slug: str) -> bool:
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

        ax.set_xlabel(axis_label(x_col))
        ax.set_ylabel(axis_label(y_col))
        ax.set_zlabel(axis_label(param_col))

        png = args.sweep_dir / f'sweep_summary_{file_slug}_vs_{param_col}.png'
        fig.savefig(png, dpi=120, bbox_inches='tight')
        print(f'Wrote {png}')
        drew_anything = True

    return drew_anything


def _plot_displacement_param_color(summary_df: pd.DataFrame,
                                   changing_display: list[str],
                                   skip_values: list[float],
                                   args: argparse.Namespace,
                                   *, x_col: str, y_col: str,
                                   file_slug: str) -> bool:
    """Render one 2D scatter per changing param using (x, y) axes with the
    param encoded as a viridis color gradient.

    Same data as _plot_displacement_3d but flattens the param dimension into
    a colorbar instead of a Z axis. In --scan-skip-cycles mode each skip
    value is drawn with its own alpha so they overlay legibly, while the
    colormap range stays pinned to the full param span across all skips.
    """
    if not {x_col, y_col}.issubset(summary_df.columns):
        return False

    drew_anything = False
    for param_col in changing_display:
        cols = [x_col, y_col, param_col]
        if not set(cols).issubset(summary_df.columns):
            continue

        full = summary_df[cols].dropna()
        if full.empty:
            continue
        vmin = float(full[param_col].min())
        vmax = float(full[param_col].max())

        fig, ax = plt.subplots(figsize=(7, 6))
        sc = None
        n_skips = len(skip_values)

        if args.scan_skip_cycles:
            for i, skip_val in enumerate(skip_values):
                sub = summary_df[summary_df['skip_cycles'] == skip_val][cols] \
                    .dropna()
                if sub.empty:
                    continue
                alpha = 0.35 + 0.65 * (i / max(n_skips - 1, 1))
                sc = ax.scatter(sub[x_col], sub[y_col], c=sub[param_col],
                                cmap='viridis', vmin=vmin, vmax=vmax,
                                alpha=alpha, s=30)
        else:
            sc = ax.scatter(full[x_col], full[y_col], c=full[param_col],
                            cmap='viridis', vmin=vmin, vmax=vmax, s=30)

        if sc is None:
            plt.close(fig)
            continue

        cbar = fig.colorbar(sc, ax=ax)
        cbar.set_label(axis_label(param_col))
        ax.set_xlabel(axis_label(x_col))
        ax.set_ylabel(axis_label(y_col))
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')

        png = args.sweep_dir / f'sweep_summary_{file_slug}_vs_{param_col}.png'
        fig.savefig(png, dpi=120, bbox_inches='tight')
        plt.close(fig)
        print(f'Wrote {png}')
        drew_anything = True

    return drew_anything


def _build_summary_from_runs(args: argparse.Namespace) \
        -> tuple[pd.DataFrame, list[str], list[float]] | None:
    """Run the full per-run pipeline and return (summary_df, changing_display, skip_values).

    Writes sweep_summary.csv and sweep_parameters.yaml as side effects.
    Returns None on any error worth aborting the script for.
    """
    runs_dir = resolve_sweep_runs_dir(args.sweep_dir)
    if runs_dir is None:
        print(f'ERROR: no sweep runs found in {args.sweep_dir} or '
              f'{args.sweep_dir / "runs"}', file=sys.stderr)
        return None
    if runs_dir != args.sweep_dir:
        print(f'Using sweep runs from {runs_dir}')

    run_dirs = sorted(p for p in runs_dir.iterdir() if p.is_dir())

    skip_values = (
        [args.skip_cycles + i * SCAN_SKIP_STEP for i in range(SCAN_SKIP_COUNT)]
        if args.scan_skip_cycles else [args.skip_cycles]
    )

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
        return None

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
    sort_keys = []
    if args.scan_skip_cycles:
        sort_keys.append('skip_cycles')
    sort_keys.append(changing_display[0] if changing_display else 'run_name')
    summary_df = summary_df.sort_values(sort_keys, kind='stable').reset_index(drop=True)

    summary_csv = args.sweep_dir / 'sweep_summary.csv'
    summary_df.to_csv(summary_csv, index=False)
    print(f'Wrote {summary_csv}')

    write_parameters_summary(args.sweep_dir, [r for _, r in base_runs])

    return summary_df, changing_display, skip_values


def _load_summary_from_csv(args: argparse.Namespace) \
        -> tuple[pd.DataFrame, list[str], list[float]] | None:
    """Charts-only flow: read existing sweep_summary.csv and derive metadata.

    Infers args.scan_skip_cycles from whether the CSV has a skip_cycles
    column. Treats any non-metric, non-meta column as a changing parameter
    (i.e. an X-axis candidate). Returns None if the CSV is missing.
    """
    summary_csv = args.sweep_dir / 'sweep_summary.csv'
    if not summary_csv.exists():
        print(f'ERROR: --charts-only requires {summary_csv} to exist',
              file=sys.stderr)
        return None

    summary_df = pd.read_csv(summary_csv)
    args.scan_skip_cycles = 'skip_cycles' in summary_df.columns
    if args.scan_skip_cycles:
        skip_values = sorted(summary_df['skip_cycles'].unique().tolist())
    else:
        skip_values = [args.skip_cycles]

    excluded = {'run_name', 'skip_cycles'} | set(METRICS.keys())
    changing_display = [c for c in summary_df.columns if c not in excluded]

    print(f'Loaded {summary_csv} ({len(summary_df)} rows)')
    if changing_display:
        print(f'Changing parameters: {", ".join(changing_display)}')
    if args.scan_skip_cycles:
        print(f'Scan mode detected: {len(skip_values)} skip_cycles values')

    return summary_df, changing_display, skip_values


def _is_processable_sweep(d: Path, charts_only: bool) -> bool:
    """True if `d` can be used as a sweep_dir in the current mode.

    In --charts-only mode the only requirement is that the prebuilt
    sweep_summary.csv exists; otherwise the directory must hold per-run
    subdirs (directly or under `runs/`).
    """
    if not d.is_dir():
        return False
    if charts_only:
        return (d / 'sweep_summary.csv').exists()
    return resolve_sweep_runs_dir(d) is not None


def process_sweep(args: argparse.Namespace) -> int:
    """Run the full pipeline for `args.sweep_dir`. Returns 0 / 1 like main()."""
    result = (_load_summary_from_csv(args) if args.charts_only
              else _build_summary_from_runs(args))
    if result is None:
        return 1
    summary_df, changing_display, skip_values = result

    if not changing_display:
        print('Skipping plots: no changing parameter to put on the X axis.')
        return 0

    for metric in METRICS:
        color = metric_color(metric)
        for param_col in changing_display:
            fig, ax = plt.subplots(figsize=(8, 5))
            drew_any = False
            n_skips = len(skip_values)

            for i, skip_val in enumerate(skip_values):
                if args.scan_skip_cycles:
                    base = summary_df[summary_df['skip_cycles'] == skip_val]
                    alpha = 0.35 + 0.65 * (i / max(n_skips - 1, 1))
                    label = f'skip_cycles={skip_val:.1f}'
                else:
                    base = summary_df
                    alpha = 1.0
                    label = None

                sub = base[[param_col, metric]] \
                    .dropna().sort_values(param_col)
                if sub.empty:
                    continue
                ax.plot(sub[param_col], sub[metric], 'o',
                        color=color, alpha=alpha, ms=4, label=label)
                drew_any = True

            if not drew_any:
                plt.close(fig)
                continue

            if args.scan_skip_cycles:
                ax.legend(loc='best', fontsize='small', ncol=2)
            ax.set_xlabel(axis_label(param_col))
            ax.set_ylabel(axis_label(metric))
            ax.grid(True, alpha=0.3)

            png = args.sweep_dir / f'sweep_summary_{metric}_vs_{param_col}.png'
            fig.savefig(png, dpi=120, bbox_inches='tight')
            plt.close(fig)
            print(f'Wrote {png}')

    # Combined chart: both displacement_x_local and displacement_y_local on
    # one set of axes, as separate series. Useful for reading off how the
    # two components evolve together as the swept param changes.
    disp_x_col = 'displacement_x_local'
    disp_y_col = 'displacement_y_local'
    if {disp_x_col, disp_y_col}.issubset(summary_df.columns):
        for param_col in changing_display:
            fig, ax = plt.subplots(figsize=(8, 5))
            drew_any = False
            n_skips = len(skip_values)

            for i, skip_val in enumerate(skip_values):
                if args.scan_skip_cycles:
                    base = summary_df[summary_df['skip_cycles'] == skip_val]
                    alpha = 0.35 + 0.65 * (i / max(n_skips - 1, 1))
                else:
                    base = summary_df
                    alpha = 1.0

                # Only attach a legend label on the first (base) skip value,
                # so the legend stays tight in scan mode.
                label_x = legend_label(disp_x_col) if i == 0 else None
                label_y = legend_label(disp_y_col) if i == 0 else None

                sub_x = base[[param_col, disp_x_col]] \
                    .dropna().sort_values(param_col)
                sub_y = base[[param_col, disp_y_col]] \
                    .dropna().sort_values(param_col)

                if not sub_x.empty:
                    ax.plot(sub_x[param_col], sub_x[disp_x_col], 'o',
                            color=metric_color(disp_x_col), alpha=alpha,
                            ms=4, label=label_x)
                    drew_any = True
                if not sub_y.empty:
                    ax.plot(sub_y[param_col], sub_y[disp_y_col], 'o',
                            color=metric_color(disp_y_col), alpha=alpha,
                            ms=4, label=label_y)
                    drew_any = True

            if not drew_any:
                plt.close(fig)
                continue

            ax.set_xlabel(axis_label(param_col))
            ax.set_ylabel(axis_label('displacement_local'))
            ax.grid(True, alpha=0.3)
            ax.legend(loc='best', fontsize='small')

            png = args.sweep_dir / f'sweep_summary_displacements_local_vs_{param_col}.png'
            fig.savefig(png, dpi=120, bbox_inches='tight')
            plt.close(fig)
            print(f'Wrote {png}')

    # Stacked figure: the combined displacements chart on top, the
    # orientation-change chart on the bottom, sharing the X axis. Same
    # styling/conventions as the standalone versions (per-metric colors,
    # alpha gradient across skip values in scan mode).
    ori_col = 'orientation_change_deg'
    if {disp_x_col, disp_y_col, ori_col}.issubset(summary_df.columns):
        for param_col in changing_display:
            fig, (ax_disp, ax_ori) = plt.subplots(
                2, 1, sharex=True, figsize=(8, 8))
            drew_any = False
            n_skips = len(skip_values)

            for i, skip_val in enumerate(skip_values):
                if args.scan_skip_cycles:
                    base = summary_df[summary_df['skip_cycles'] == skip_val]
                    alpha = 0.35 + 0.65 * (i / max(n_skips - 1, 1))
                else:
                    base = summary_df
                    alpha = 1.0

                label_x = legend_label(disp_x_col) if i == 0 else None
                label_y = legend_label(disp_y_col) if i == 0 else None

                sub_x = base[[param_col, disp_x_col]] \
                    .dropna().sort_values(param_col)
                sub_y = base[[param_col, disp_y_col]] \
                    .dropna().sort_values(param_col)
                sub_ori = base[[param_col, ori_col]] \
                    .dropna().sort_values(param_col)

                if not sub_x.empty:
                    ax_disp.plot(sub_x[param_col], sub_x[disp_x_col], 'o',
                                 color=metric_color(disp_x_col), alpha=alpha,
                                 ms=4, label=label_x)
                    drew_any = True
                if not sub_y.empty:
                    ax_disp.plot(sub_y[param_col], sub_y[disp_y_col], 'o',
                                 color=metric_color(disp_y_col), alpha=alpha,
                                 ms=4, label=label_y)
                    drew_any = True
                if not sub_ori.empty:
                    ax_ori.plot(sub_ori[param_col], sub_ori[ori_col], 'o',
                                color=metric_color(ori_col), alpha=alpha, ms=4)
                    drew_any = True

            if not drew_any:
                plt.close(fig)
                continue

            ax_disp.set_ylabel(axis_label('displacement_local'))
            ax_disp.grid(True, alpha=0.3)
            ax_disp.legend(loc='best', fontsize='small')

            ax_ori.set_xlabel(axis_label(param_col))
            ax_ori.set_ylabel(axis_label(ori_col))
            ax_ori.grid(True, alpha=0.3)

            png = args.sweep_dir / f'sweep_summary_motion_vs_{param_col}.png'
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
        file_slug='displacement3d',
    )
    shown_any_3d |= _plot_displacement_3d(
        summary_df, changing_display, skip_values, args,
        x_col='displacement_x_local', y_col='displacement_y_local',
        file_slug='displacement_local3d',
    )

    # 2D analog of the local-frame 3D chart: same (x_local, y_local) axes,
    # but the swept param is encoded as point color rather than the Z axis.
    _plot_displacement_param_color(
        summary_df, changing_display, skip_values, args,
        x_col='displacement_x_local', y_col='displacement_y_local',
        file_slug='displacement_local_color',
    )

    # if shown_any_3d:
    #     print('Opening 3D chart(s); close the window(s) to exit.')
    #     plt.show()

    return 0


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
    parser.add_argument('--charts-only', action='store_true',
                        help='Skip per-run data loading; redraw charts from the '
                             'existing sweep_summary.csv in <sweep_dir>. '
                             'Scan mode is inferred from the CSV.')
    parser.add_argument('-r', '--recursive', action='store_true',
                        help='Treat <sweep_dir> as a parent of multiple sweeps '
                             'and process each immediate subdirectory that '
                             'looks like a sweep target.')
    args = parser.parse_args()

    if not args.sweep_dir.is_dir():
        print(f'ERROR: {args.sweep_dir} is not a directory', file=sys.stderr)
        return 1

    if not args.recursive:
        return process_sweep(args)

    parent = args.sweep_dir
    candidates = sorted(p for p in parent.iterdir() if p.is_dir())
    targets = [p for p in candidates
               if _is_processable_sweep(p, args.charts_only)]
    if not targets:
        print(f'ERROR: no processable sweep subdirectories under {parent}',
              file=sys.stderr)
        return 1

    print(f'Recursive mode: processing {len(targets)} sweep(s) under {parent}')
    failures = 0
    for sub in targets:
        print(f'\n=== {sub.name} ===')
        args.sweep_dir = sub
        try:
            if process_sweep(args) != 0:
                failures += 1
        finally:
            plt.close('all')
    args.sweep_dir = parent

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
