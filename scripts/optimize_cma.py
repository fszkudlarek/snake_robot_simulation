#!/usr/bin/env python3
"""
CMA-ES tuning of the sidewinding gait so the robot's center-of-mass path
geometrically matches the desired trajectory.

The simulation runs OPEN-LOOP: snake_sim_launch.py is invoked with
`use_trajectory_publisher:=true` so the trajectory marker is visible (and the
desired polyline is well-defined for scoring), but no `trajectory_tracker` is
started. CMA only sees the feedforward gait parameters and the resulting COM
path; the controller never receives the trajectory.

Search space (6 dims) — same parameterization as before, chosen to keep the
optimizer inside the "alternating sidewinding gait" regime by construction:

    A_h               horizontal wave amplitude (rad)
    delta_phi_h       horizontal inter-module phase difference (rad)
    delta_phi_v       vertical inter-module phase difference (rad)
    delta_phi_vh      vertical-to-horizontal phase offset (rad)
    O_v_frac          O_v / A_v ∈ (-0.9, 0.9)
    O_h_frac          O_h / A_h ∈ (-0.9, 0.9)

Fixed: A_v = 1.0, T = 5.0.

Objective (minimize):

    J = sqrt(mean(||COM(t) - desired(t)||^2))

The desired trajectory is time-parametrized: trajectory_publisher's
`linear_speed` (m/s along the curve) maps each polyline vertex to a
`time_from_start`, exactly the parametrization the moving marker uses in
RViz. At every COM timestamp the desired (x, y) is linearly interpolated
from that parametrization, and J is the RMS distance between the two.

The body-CSV clock is rebased so t=0 corresponds to the first post-transient
sample — i.e. the desired trajectory effectively restarts after the gait has
settled. This penalizes both geometric error AND speed mismatch: a snake
moving too slowly drifts behind the moving target every timestep and
accumulates distance, so a stationary snake can no longer be a degenerate
optimum.

Per-evaluation output: <repo>/sweep_output/cma/<session_name>/eval_<gen>_<idx>/
The CMA-ES state is checkpointed to <session_name>/cma_state.pkl after every
generation, so an interrupted run can be resumed by re-invoking with the
same --session-name.

Usage:
    python3 scripts/optimize_cma.py --session-name run1 --budget 60

Resume:
    python3 scripts/optimize_cma.py --session-name run1 --budget 60

Dependencies: pip install cma
"""
import argparse
import csv
import math
import pickle
import shutil
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

try:
    import cma
except ImportError:
    print('ERROR: the `cma` package is required.\n'
          '  pip install cma', file=sys.stderr)
    sys.exit(1)

# Local imports — both scripts live in scripts/.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_sweep import run_one, load_default_params, REPO_ROOT, DEFAULTS_FILE


# Source-tree trajectory definition. Mirrors the same file the installed
# share serves to trajectory_publisher — when the user edits it they need
# `colcon build` for the sim to pick it up; the optimizer reads from source
# so it sees edits immediately.
TRAJECTORY_FILE = REPO_ROOT / 'config' / 'trajectory.yaml'
TRAJECTORY_NODE_NAME = 'trajectory_publisher'

# Hardcoded physical-param values used by the reparameterization.
A_V_FIXED = 1.0
OFFSET_FRAC_LIMIT = 0.9                # |O_h|/A_h and |O_v|/A_v ≤ this

# Order matters — defines the CMA vector layout. Don't reorder without
# rewriting any pickled state. These are the *internal* search variables;
# the physical 7-param set fed to the controller is reconstructed in
# vector_to_params().
PARAM_NAMES = [
    'A_h',
    'delta_phi_h',
    'delta_phi_v',
    'delta_phi_vh',
    'O_v_frac',
    'O_h_frac',
]

PARAM_BOUNDS = {
    'A_h':            (0.05, 5 * math.pi / 18),                      # ~3° to ~50°
    'delta_phi_h':    (0.50, 3 * math.pi),
    'delta_phi_v':    (0.50, 3 * math.pi),
    'delta_phi_vh':   (-math.pi, math.pi),
    'O_v_frac':       (-OFFSET_FRAC_LIMIT, OFFSET_FRAC_LIMIT),
    'O_h_frac':       (-OFFSET_FRAC_LIMIT, OFFSET_FRAC_LIMIT),
}

# Physical params written into the eval log — what you actually want to
# analyze afterwards. Order is not load-bearing; the dict lookup in the
# row builder uses these names.
PHYSICAL_PARAM_NAMES = [
    'A_h', 'delta_phi_v', 'delta_phi_h', 'delta_phi_vh', 'O_v', 'O_h',
]

LOG_COLUMNS = (
    ['gen', 'eval', 'wall_seconds']
    + PHYSICAL_PARAM_NAMES
    + ['n_samples', 'duration_s', 'mean_distance_m',
       'rms_distance_m', 'max_distance_m', 'J', 'status']
)

# Tag stored in session_config.yaml so a pickled CMA state from an
# incompatible run — different objective sign, different log schema, or a
# different CMA vector layout — is rejected instead of silently resumed.
# Bump this whenever PARAM_NAMES changes meaning.
OBJECTIVE_TAG = 'trajectory_time_aligned_rms_v2'

# Returned J when the simulation produced no usable data. Large enough to be
# uncompetitive vs. any real run, finite so CMA's covariance update stays
# well-defined.
FAILED_RUN_PENALTY = 1.0e3


def load_desired_trajectory() -> tuple[np.ndarray, np.ndarray]:
    """Reconstruct the time-parametrized desired trajectory from
    config/trajectory.yaml, mirroring trajectory_publisher's
    `_build_*()` + `_make_trajectory()` pipeline.

    Returns:
        polyline:       (N, 2) array of [x, y] points
        desired_times:  (N,)   array of time_from_start in seconds (monotone,
                               starts at 0)
    """
    if not TRAJECTORY_FILE.exists():
        raise FileNotFoundError(
            f'Cannot find {TRAJECTORY_FILE}; the optimizer needs the desired '
            f'trajectory definition to score evaluations.'
        )
    with open(TRAJECTORY_FILE) as f:
        data = yaml.safe_load(f) or {}
    params = data.get(TRAJECTORY_NODE_NAME, {}).get('ros__parameters', {}) or {}

    traj_type = params.get('type', 'waypoints')
    n = int(params.get('num_points', 100))
    sx = float(params.get('start_x', 0.0))
    sy = float(params.get('start_y', 0.36))

    if traj_type == 'waypoints':
        flat = params.get('waypoints', []) or []
        pts = [(float(flat[i]), float(flat[i + 1]))
               for i in range(0, len(flat) - 1, 2)]
    elif traj_type == 'line':
        length = float(params.get('length', 2.0))
        angle = math.radians(float(params.get('angle', 0.0)))
        dx, dy = math.cos(angle), math.sin(angle)
        pts = [(sx + i * length / (n - 1) * dx,
                sy + i * length / (n - 1) * dy) for i in range(n)]
    elif traj_type == 'circle':
        r = float(params.get('radius', 0.5))
        d = str(params.get('direction', 'ccw')).strip().lower()
        s = -1 if d in ('cw', 'clockwise', 'right') else 1
        cy = sy + s * r
        # +1 to close the loop, matching trajectory_publisher._build_circle.
        pts = [
            (sx + r * math.cos(s * (2 * math.pi * i / n - math.pi / 2)),
             cy + r * math.sin(s * (2 * math.pi * i / n - math.pi / 2)))
            for i in range(n + 1)
        ]
    elif traj_type == 'sine':
        length = float(params.get('length', 2.0))
        amp = float(params.get('amplitude', 0.3))
        wl = float(params.get('wavelength', 1.0))
        pts = [
            (sx + i * length / (n - 1),
             sy + amp * math.sin(2 * math.pi * (i * length / (n - 1)) / wl))
            for i in range(n)
        ]
    else:
        raise ValueError(f'Unknown trajectory type: {traj_type!r}')

    if len(pts) < 2:
        raise ValueError(
            f'Trajectory has only {len(pts)} points — need at least 2 for a polyline.'
        )

    linear_speed = float(params.get('linear_speed', 0.1))
    if linear_speed <= 0.0:
        raise ValueError(
            f'trajectory.yaml linear_speed must be > 0 to time-parametrize the '
            f'trajectory; got {linear_speed}.'
        )

    polyline = np.asarray(pts, dtype=float)
    segment_lengths = np.linalg.norm(np.diff(polyline, axis=0), axis=1)
    cumulative_arc = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    desired_times = cumulative_arc / linear_speed
    return polyline, desired_times


def compute_tracking_error(csv_path: Path, polyline: np.ndarray,
                           desired_times: np.ndarray,
                           transient_seconds: float) -> dict:
    """RMS time-aligned distance between the COM and the desired position.

    The body-CSV clock is rebased so the first post-transient sample is t=0,
    matching the desired trajectory's own t=0 (the first polyline vertex).
    Past the trajectory's final time, the desired position clamps to the last
    vertex — same behavior as trajectory_publisher._desired_xy_at.
    """
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=['com_x', 'com_y']).reset_index(drop=True)
    if len(df) < 2:
        raise ValueError(f'{csv_path}: fewer than 2 valid COM samples.')

    t0 = float(df['time'].iloc[0]) + transient_seconds
    df = df[df['time'] >= t0].reset_index(drop=True)
    if len(df) < 2:
        raise ValueError(
            f'{csv_path}: no samples after {transient_seconds}s transient.'
        )

    body_times = df['time'].to_numpy()
    t_rel = body_times - body_times[0]
    com = df[['com_x', 'com_y']].to_numpy()

    # np.interp clamps to endpoint values for queries outside desired_times,
    # which matches trajectory_publisher's behavior past the final vertex.
    desired_x = np.interp(t_rel, desired_times, polyline[:, 0])
    desired_y = np.interp(t_rel, desired_times, polyline[:, 1])
    desired = np.column_stack([desired_x, desired_y])

    distances = np.linalg.norm(com - desired, axis=1)

    return {
        'duration_s': float(t_rel[-1]),
        'n_samples': len(distances),
        'mean_distance_m': float(distances.mean()),
        'max_distance_m': float(distances.max()),
        'rms_distance_m': float(np.sqrt(np.mean(distances ** 2))),
    }


def vector_to_params(x, defaults: dict) -> dict:
    """Reconstruct the full 7-param controller config from the 6-dim CMA vector.

    delta_phi_v and delta_phi_h are independent dimensions. The offsets are
    still scaled by their wave amplitudes so |offset| < amplitude holds
    structurally — this is purely a numerical safety constraint, not a
    physical coupling.
    """
    A_h, delta_phi_h, delta_phi_v, delta_phi_vh, O_v_frac, O_h_frac = (
        float(v) for v in x
    )
    p = dict(defaults)
    p['A_h'] = A_h
    p['delta_phi_h'] = delta_phi_h
    p['delta_phi_v'] = delta_phi_v
    p['delta_phi_vh'] = delta_phi_vh
    p['O_v'] = O_v_frac * A_V_FIXED
    p['O_h'] = O_h_frac * A_h
    return p


def physical_params_from_defaults(defaults: dict) -> list[float]:
    """Map the defaults dict into an initial CMA vector in the new layout."""
    A_h = float(defaults.get('A_h', (PARAM_BOUNDS['A_h'][0]
                                     + PARAM_BOUNDS['A_h'][1]) / 2))
    delta_phi_h = float(defaults.get('delta_phi_h', math.pi / 2))
    delta_phi_v = float(defaults.get('delta_phi_v', delta_phi_h))
    delta_phi_vh = float(defaults.get('delta_phi_vh', math.pi / 2))
    O_v = float(defaults.get('O_v', 0.0))
    O_h = float(defaults.get('O_h', 0.0))

    return [
        A_h,
        delta_phi_h,
        delta_phi_v,
        delta_phi_vh,
        O_v / A_V_FIXED,
        O_h / A_h if A_h > 1e-9 else 0.0,
    ]


def write_best_params(path: Path, params: dict) -> None:
    """Write a ROS params YAML that run_sweep / launch can consume directly."""
    payload = {'movement_controller_node': {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)


def build_log_row(iteration: int, eval_count: int, wall: float,
                  physical: dict, info: dict, J: float) -> list:
    head = [iteration, eval_count, f'{wall:.1f}']
    params = [f'{physical[name]:.6f}' for name in PHYSICAL_PARAM_NAMES]

    tail = []
    for col in ('n_samples', 'duration_s', 'mean_distance_m',
                'rms_distance_m', 'max_distance_m'):
        v = info.get(col)
        if v is None:
            tail.append('')
        elif col == 'n_samples':
            tail.append(str(v))
        elif col == 'duration_s':
            tail.append(f'{v:.2f}')
        else:
            tail.append(f'{v:.4f}')
    tail.append(f'{J:.6f}')
    tail.append(info.get('status', ''))

    return head + params + tail


def evaluate(x, defaults: dict, eval_dir: Path, run_name: str,
             wait_seconds: int, polyline: np.ndarray,
             desired_times: np.ndarray,
             transient_seconds: float) -> tuple[float, dict]:
    """Run one simulation, score it against the time-parametrized trajectory.

    Returns (J, info). `info` always contains 'status'; on success it also
    contains the keys build_log_row expects (n_samples, duration_s,
    mean/rms/max_distance_m).
    """
    params = vector_to_params(x, defaults)
    try:
        traj_csv = run_one(run_name, params, wait_seconds, eval_dir,
                           take_snapshot=False, render_gif=False, quiet=True,
                           use_trajectory_publisher=False)
    except Exception as e:
        return FAILED_RUN_PENALTY, {'status': f'sim_error: {e!r}'}

    if not traj_csv.exists():
        return FAILED_RUN_PENALTY, {'status': 'no_csv'}

    try:
        m = compute_tracking_error(traj_csv, polyline, desired_times,
                                   transient_seconds)
    except (ValueError, FileNotFoundError, KeyError) as e:
        return FAILED_RUN_PENALTY, {'status': f'metrics_error: {e}'}

    return m['rms_distance_m'], {'status': 'ok', **m}


def main() -> int:
    parser = argparse.ArgumentParser(
        description='CMA-ES gait tuning to minimize COM-to-desired-trajectory '
                    'RMS distance.')
    parser.add_argument('--session-name', required=True,
                        help='Folder name under sweep_output/cma/. Resumes if state exists.')
    parser.add_argument('--budget', type=int, default=50,
                        help='Total evaluations across all generations (default: 50)')
    parser.add_argument('--popsize', type=int, default=None,
                        help='CMA population size per generation '
                             '(default: 4 + 3*ln(n_dims) ≈ 9)')
    parser.add_argument('--sigma-frac', type=float, default=0.25,
                        help='Initial sigma as fraction of each bound range (default: 0.25)')
    parser.add_argument('--wait-seconds', type=int, default=300,
                        help='Wall-clock seconds per simulation (default: 300)')
    parser.add_argument('--transient-seconds', type=float, default=10.0,
                        help='Seconds of initial settling to discard before '
                             'scoring (default: 10.0 = ~2 gait cycles at T=5s)')
    parser.add_argument('--seed', type=int, default=42,
                        help='CMA-ES seed for reproducibility (default: 42)')
    args = parser.parse_args()

    defaults = load_default_params()
    if not defaults:
        print('ERROR: could not load defaults from default_controller_params.yaml',
              file=sys.stderr)
        return 1

    polyline, desired_times = load_desired_trajectory()
    print(f'Loaded desired trajectory: {len(polyline)} points, '
          f'duration {desired_times[-1]:.1f}s, from {TRAJECTORY_FILE.name}',
          flush=True)

    n_dims = len(PARAM_NAMES)
    lower = [PARAM_BOUNDS[n][0] for n in PARAM_NAMES]
    upper = [PARAM_BOUNDS[n][1] for n in PARAM_NAMES]

    # Initial mean: derived from the user's manually-tuned defaults, mapped
    # into the reparameterized CMA vector. Clamped to bounds in case a
    # derived value sits outside the allowed range.
    x0_raw = physical_params_from_defaults(defaults)
    x0 = [
        max(lower[i], min(upper[i], x0_raw[i]))
        for i in range(n_dims)
    ]
    stds = [args.sigma_frac * (upper[i] - lower[i]) for i in range(n_dims)]

    session_dir = REPO_ROOT / 'sweep_output' / 'cma' / args.session_name
    session_dir.mkdir(parents=True, exist_ok=True)

    state_path = session_dir / 'cma_state.pkl'
    log_path = session_dir / 'eval_log.csv'
    best_path = session_dir / 'best_params.yaml'
    config_path = session_dir / 'session_config.yaml'

    # Refuse to resume a session that was created with a different objective —
    # the old gait-based optimizer had opposite minimize/maximize sign and a
    # different log schema; silently continuing would corrupt the run.
    if state_path.exists():
        prior_tag = None
        if config_path.exists():
            with open(config_path) as f:
                prior_cfg = yaml.safe_load(f) or {}
            prior_tag = prior_cfg.get('objective')
        if prior_tag != OBJECTIVE_TAG:
            prior_desc = prior_tag or '(legacy gait-based optimizer)'
            print(f'ERROR: session {args.session_name!r} was created with '
                  f'objective={prior_desc!r}; this optimizer minimizes '
                  f'{OBJECTIVE_TAG!r}. Use a different --session-name.',
                  file=sys.stderr)
            return 1
        print(f'Resuming from {state_path}', flush=True)
        with open(state_path, 'rb') as f:
            es, eval_count, best_J, best_x = pickle.load(f)
    else:
        popsize = args.popsize or (4 + int(3 * math.log(n_dims)))
        cma_opts = {
            'bounds': [lower, upper],
            'CMA_stds': stds,
            'popsize': popsize,
            'seed': args.seed,
            'verbose': -9,
        }
        es = cma.CMAEvolutionStrategy(x0, 1.0, cma_opts)
        eval_count = 0
        best_J = float('inf')
        best_x = None
        with open(log_path, 'w', newline='') as f:
            csv.writer(f).writerow(LOG_COLUMNS)
        with open(config_path, 'w') as f:
            yaml.safe_dump({'objective': OBJECTIVE_TAG}, f)
        # Snapshot the trajectory + defaults YAMLs so the session is
        # self-contained: J values stay interpretable even if the source-tree
        # configs change later, and the snapshots describe the full physical
        # parameterization (CMA only varies 6 dims; A_v, T, and alpha_distribution
        # come from the defaults).
        shutil.copyfile(TRAJECTORY_FILE, session_dir / 'trajectory.yaml')
        if DEFAULTS_FILE.exists():
            shutil.copyfile(DEFAULTS_FILE,
                            session_dir / 'default_controller_params.yaml')
        print(f'Starting fresh CMA-ES session: {session_dir}', flush=True)
        print(f'Objective: minimize {OBJECTIVE_TAG} (RMS COM-to-polyline distance)',
              flush=True)
        print(f'Budget: {args.budget} evals, popsize={popsize}, n_dims={n_dims}', flush=True)
        print(f'Transient discard: {args.transient_seconds:.1f}s', flush=True)
        print('Search-vector init (CMA-internal layout):', flush=True)
        for name, x_val, std in zip(PARAM_NAMES, x0, stds):
            lo, hi = PARAM_BOUNDS[name]
            print(f'  {name:16s} = {x_val:+.4f}  '
                  f'σ0={std:.4f}  bounds=[{lo:+.4f}, {hi:+.4f}]', flush=True)

    # Round the budget up to a full generation — partial generations consume
    # sim time without informing CMA's covariance update.
    popsize = es.popsize
    budget = args.budget
    if budget % popsize != 0:
        rounded = ((budget // popsize) + 1) * popsize
        print(f'WARNING: --budget {budget} is not a multiple of popsize {popsize}. '
              f'A trailing partial generation would not inform CMA. '
              f'Rounding up to {rounded}.', flush=True)
        budget = rounded

    # Save state on Ctrl-C so an in-progress generation isn't lost.
    interrupted = {'flag': False}

    def _handle_sigint(signum, frame):
        if interrupted['flag']:
            print('\n  Second Ctrl-C — exiting immediately.', flush=True)
            sys.exit(130)
        interrupted['flag'] = True
        print('\n  Ctrl-C received — will checkpoint and exit after current eval.',
              flush=True)
    signal.signal(signal.SIGINT, _handle_sigint)

    eval_durations = []

    try:
        while eval_count < budget and not es.stop():
            iteration = es.countiter
            xs = es.ask()
            fs = []
            full_pop = True

            for i, x in enumerate(xs):
                if eval_count >= budget or interrupted['flag']:
                    full_pop = False
                    break

                run_name = f'eval_{iteration:03d}_{i:02d}'
                eval_dir = session_dir / run_name
                t_start = time.time()

                physical = vector_to_params(x, defaults)
                print(f'\n=== gen {iteration} eval {eval_count + 1}/{budget}  '
                      f'(pop {i + 1}/{len(xs)}) ===', flush=True)
                print(f'  A_h={physical["A_h"]:.4f}  '
                      f'dphi_h={physical["delta_phi_h"]:.4f}  '
                      f'dphi_v={physical["delta_phi_v"]:.4f}  '
                      f'dphi_vh={physical["delta_phi_vh"]:.4f}  '
                      f'O_v={physical["O_v"]:.4f}  '
                      f'O_h={physical["O_h"]:.4f}', flush=True)

                J, info = evaluate(
                    x, defaults, eval_dir, run_name,
                    args.wait_seconds, polyline, desired_times,
                    args.transient_seconds,
                )
                # CMA minimizes; J is already the cost (RMS distance), so
                # feed it directly. Lower is better.
                fs.append(J)

                wall = time.time() - t_start
                eval_durations.append(wall)
                eval_count += 1

                with open(log_path, 'a', newline='') as f:
                    csv.writer(f).writerow(
                        build_log_row(iteration, eval_count, wall,
                                      physical, info, J)
                    )

                if J < best_J:
                    best_J = J
                    best_x = list(x)
                    write_best_params(best_path, vector_to_params(best_x, defaults))

                avg = sum(eval_durations) / len(eval_durations)
                eta_s = avg * (budget - eval_count)
                print(f'  J={J:.6f}  '
                      f'mean={info.get("mean_distance_m", 0):.4f}  '
                      f'max={info.get("max_distance_m", 0):.4f}  '
                      f'status={info.get("status", "?")}', flush=True)
                print(f'  best so far: J={best_J:.6f}  '
                      f'wall {wall:.0f}s  ETA {eta_s / 60:.1f} min', flush=True)

            if full_pop:
                es.tell(xs, fs)
                with open(state_path, 'wb') as f:
                    pickle.dump((es, eval_count, best_J, best_x), f)
            else:
                # Partial generation (interrupted or budget hit) — keep state
                # at the last completed generation. Don't tell() with partial fs.
                break

    finally:
        print(f'\nFinished {eval_count} evals.', flush=True)
        if best_x is not None:
            best_params = vector_to_params(best_x, defaults)
            print(f'Best J = {best_J:.6f}  (RMS COM-to-polyline distance, m)',
                  flush=True)
            print('Physical params (these are what get fed to the controller):',
                  flush=True)
            for name in PHYSICAL_PARAM_NAMES:
                print(f'  {name:14s} = {best_params[name]:+.6f}', flush=True)
            print(f'\nBest params written to: {best_path}', flush=True)
            print(f'Replay it with:', flush=True)
            print(f'  ros2 launch snake_sim snake_sim_launch.py '
                  f'controller_params_file:={best_path} '
                  f'use_trajectory_publisher:=true', flush=True)

    return 0


if __name__ == '__main__':
    sys.exit(main())
