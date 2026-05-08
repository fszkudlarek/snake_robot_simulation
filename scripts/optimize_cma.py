#!/usr/bin/env python3
"""
CMA-ES optimization of the sidewinding gait controller parameters.

Search space (6 dims) — each chosen to keep the optimizer inside the
"alternating sidewinding gait" regime by construction:

    A_h               horizontal wave amplitude (rad)
    delta_phi_h       horizontal inter-module phase difference (rad)
    delta_phi_diff    delta_phi_v - delta_phi_h, bounded to ±pi/5
                      (the H2 paper assumes delta_phi_v == delta_phi_h;
                       we explore small symmetric deviations)
    delta_phi_vh      vertical-to-horizontal phase offset (rad)
    O_v_frac          O_v / A_v ∈ (-0.9, 0.9). Keeps |O_v| < A_v so the
                      vertical wave crosses zero and pads alternate grip/slide.
    O_h_frac          O_h / A_h ∈ (-0.9, 0.9). Keeps |O_h| < A_h so the
                      horizontal wave crosses zero — prevents persistent
                      same-direction bend that causes module overlap.

The controller still receives all 7 physical params (A_v, A_h, delta_phi_v,
delta_phi_h, delta_phi_vh, O_v, O_h, T); CMA only sees these 6 and the rest
are reconstructed in vector_to_params() with the constraints structurally
satisfied.

Fixed:
    A_v = 1.0   (only the sign matters in the controller logic)
    T   = 5.0   (gait period — speed knob, not gait shape)

Objective: J = v * S^alpha (maximize), where v is forward speed of the COM
and S is path straightness. Computed by compute_metrics.compute_metrics().

Per-evaluation output is written under
    <repo>/sweep_output/cma/<session_name>/eval_<gen>_<idx>/
including the params YAML and the body trajectory CSV. The CMA-ES state is
checkpointed to <session_name>/cma_state.pkl after every generation, so an
interrupted run can be resumed by re-invoking with the same --session-name.

Usage:
    python3 scripts/optimize_cma.py --session-name run1 --budget 60

Resume:
    python3 scripts/optimize_cma.py --session-name run1 --budget 60   # detects state, continues

Dependencies: pip install cma
"""
import argparse
import csv
import math
import pickle
import signal
import sys
import time
from pathlib import Path

import yaml

try:
    import cma
except ImportError:
    print('ERROR: the `cma` package is required.\n'
          '  pip install cma', file=sys.stderr)
    sys.exit(1)

# Local imports — both scripts live in scripts/.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_sweep import run_one, load_default_params, REPO_ROOT
from compute_metrics import compute_metrics


# Hardcoded physical-param values used by the reparameterization.
A_V_FIXED = 1.0
DELTA_PHI_DIFF_LIMIT = math.pi / 6     # |delta_phi_v - delta_phi_h| ≤ this
OFFSET_FRAC_LIMIT = 0.9                # |O_h|/A_h and |O_v|/A_v ≤ this

# Order matters — defines the CMA vector layout. Don't reorder without
# rewriting any pickled state. These are the *internal* search variables;
# the physical 7-param set fed to the controller is reconstructed in
# vector_to_params().
PARAM_NAMES = [
    'A_h',
    'delta_phi_h',
    'delta_phi_diff',   # delta_phi_v - delta_phi_h
    'delta_phi_vh',
    'O_v_frac',         # O_v / A_v
    'O_h_frac',         # O_h / A_h
]

PARAM_BOUNDS = {
    'A_h':            (0.05, 5 * math.pi / 18),                      # ~3° to ~50° (to avoid the robot segments overlaping)
    'delta_phi_h':    (0.50, 2 * math.pi),                           # ~28° to ~180°
    'delta_phi_diff': (-DELTA_PHI_DIFF_LIMIT, DELTA_PHI_DIFF_LIMIT),
    'delta_phi_vh':   (- math.pi,  math.pi),                               # sidewinding lives near π/2
    'O_v_frac':       (-OFFSET_FRAC_LIMIT, OFFSET_FRAC_LIMIT),
    'O_h_frac':       (-OFFSET_FRAC_LIMIT, OFFSET_FRAC_LIMIT),
}

# Physical params written into the eval log — what you actually want to
# analyze afterwards. Order is not load-bearing; the dict lookup in the
# row builder uses these names.
PHYSICAL_PARAM_NAMES = [
    'A_h', 'delta_phi_v', 'delta_phi_h', 'delta_phi_vh', 'O_v', 'O_h',
]

SUPPORTED_GAITS = ('sidewinding', 'rotating')

# Per-gait CSV tail columns. The leading columns (gen, eval, wall_seconds,
# physical params) are common to both. The order here is the order written.
GAIT_LOG_TAIL = {
    'sidewinding': ['n_cycles', 'duration_s', 'net_disp_m', 'S', 'v', 'J', 'status'],
    'rotating':    ['n_cycles', 'duration_s', 'net_rotation_rad',
                    'drift_distance_m', 'R', 'omega', 'J', 'status'],
}


def log_columns(gait: str) -> list[str]:
    return ['gen', 'eval', 'wall_seconds'] + PHYSICAL_PARAM_NAMES + GAIT_LOG_TAIL[gait]


# Format spec per metric column. Anything not listed falls back to plain
# str() — used for n_cycles (int) and status (string). When the key is
# missing from `info`, an empty cell is written.
_METRIC_FORMATS = {
    'duration_s':       '.2f',
    'net_disp_m':       '.4f',
    'net_rotation_rad': '.4f',
    'drift_distance_m': '.4f',
    'S':                '.4f',
    'R':                '.4f',
    'v':                '.6f',
    'omega':            '.6f',
}


def build_log_row(gait: str, iteration: int, eval_count: int, wall: float,
                  physical: dict, info: dict, J: float) -> list:
    head = [iteration, eval_count, f'{wall:.1f}']
    params = [f'{physical[name]:.6f}' for name in PHYSICAL_PARAM_NAMES]

    tail = []
    for col in GAIT_LOG_TAIL[gait]:
        if col == 'J':
            tail.append(f'{J:.6f}')
            continue
        if col == 'status':
            tail.append(info.get('status', ''))
            continue
        if col not in info:
            tail.append('')
            continue
        spec = _METRIC_FORMATS.get(col)
        tail.append(format(info[col], spec) if spec else str(info[col]))

    return head + params + tail


def vector_to_params(x, defaults: dict) -> dict:
    """Reconstruct the full 7-param controller config from the 6-dim CMA vector.

    The reparameterization makes constraints structural rather than penalized:
    delta_phi_v is built from delta_phi_h plus a small bounded difference,
    and the offsets are scaled by their wave amplitudes so |offset| < amplitude.
    """
    A_h, delta_phi_h, delta_phi_diff, delta_phi_vh, O_v_frac, O_h_frac = (
        float(v) for v in x
    )
    p = dict(defaults)
    p['A_h'] = A_h
    p['delta_phi_h'] = delta_phi_h
    p['delta_phi_v'] = 2 * delta_phi_h + delta_phi_diff
    p['delta_phi_vh'] = delta_phi_vh
    p['O_v'] = O_v_frac * A_V_FIXED
    p['O_h'] = O_h_frac * A_h
    return p


def physical_params_from_defaults(defaults: dict) -> list[float]:
    """Map the defaults dict into an initial CMA vector in the new layout.

    Falls back to mid-bound values if a default is missing or would produce
    a NaN (e.g. division by zero on O_h_frac if A_h default were 0).
    """
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
        delta_phi_v - delta_phi_h,
        delta_phi_vh,
        O_v / A_V_FIXED,
        O_h / A_h if A_h > 1e-9 else 0.0,
    ]


def write_best_params(path: Path, params: dict) -> None:
    """Write a ROS params YAML that run_sweep / launch can consume directly."""
    payload = {'movement_controller_node': {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)


def evaluate(x, defaults: dict, eval_dir: Path, run_name: str,
             wait_seconds: int, period: float, alpha: float,
             gait: str) -> tuple[float, dict]:
    """Run one simulation, compute metrics. Returns (J, info).

    `info` always contains 'status'; on success it also contains the gait's
    full metric dict (S/v/... or R/omega/...) which build_log_row knows
    how to lay out.
    """
    params = vector_to_params(x, defaults)
    try:
        traj_csv = run_one(run_name, params, wait_seconds, eval_dir,
                           take_snapshot=False, render_gif=False, quiet=True)
    except Exception as e:
        return 0.0, {'status': f'sim_error: {e!r}'}

    if not traj_csv.exists():
        return 0.0, {'status': 'no_csv'}

    try:
        m = compute_metrics(traj_csv, period,
                            transient_cycles=5.0, alpha=alpha, gait=gait)
    except (ValueError, FileNotFoundError, KeyError) as e:
        return 0.0, {'status': f'metrics_error: {e}'}

    return m['J'], {'status': 'ok', **m}


def main() -> int:
    parser = argparse.ArgumentParser(description='CMA-ES gait parameter optimization.')
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
    parser.add_argument('--period', type=float, default=5.0,
                        help='Gait period T (default: 5.0)')
    parser.add_argument('--alpha', type=float, default=2.0,
                        help='Purity exponent in J = speed * purity^alpha (default: 2.0)')
    parser.add_argument('--gait', choices=list(SUPPORTED_GAITS), default=None,
                        help='Which gait metric to optimize. On a fresh session this '
                             'choice is locked into session_config.yaml. On resume, '
                             'the locked value is used and any --gait passed here '
                             'must match it (default: sidewinding for fresh sessions).')
    parser.add_argument('--seed', type=int, default=42,
                        help='CMA-ES seed for reproducibility (default: 42)')
    args = parser.parse_args()

    defaults = load_default_params()
    if not defaults:
        print('ERROR: could not load defaults from default_controller_params.yaml',
              file=sys.stderr)
        return 1

    n_dims = len(PARAM_NAMES)
    lower = [PARAM_BOUNDS[n][0] for n in PARAM_NAMES]
    upper = [PARAM_BOUNDS[n][1] for n in PARAM_NAMES]

    # Initial mean: derived from the user's manually-tuned defaults, mapped
    # into the reparameterized CMA vector. Clamped to bounds in case a
    # derived value (e.g. O_h_frac when defaults aren't yet in the gait
    # regime) sits outside the allowed range.
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

    # Resolve the gait for this session. The first invocation locks the
    # choice into session_config.yaml; subsequent invocations must agree.
    # Pre-existing sessions without the sidecar are assumed to be sidewinding
    # (the only gait that existed before this option was added).
    if config_path.exists():
        with open(config_path) as f:
            session_cfg = yaml.safe_load(f) or {}
        locked_gait = session_cfg.get('gait', 'sidewinding')
    elif state_path.exists():
        locked_gait = 'sidewinding'  # legacy session — pre-gait-flag
    else:
        locked_gait = None

    if locked_gait is not None:
        if args.gait is not None and args.gait != locked_gait:
            print(f'ERROR: session {args.session_name!r} is locked to '
                  f'gait={locked_gait!r}; --gait {args.gait!r} would corrupt '
                  f'its eval_log.csv schema. Use a different --session-name.',
                  file=sys.stderr)
            return 1
        gait = locked_gait
    else:
        gait = args.gait or 'sidewinding'

    if state_path.exists():
        print(f'Resuming from {state_path}  (gait={gait})', flush=True)
        with open(state_path, 'rb') as f:
            es, eval_count, best_J, best_x = pickle.load(f)
        # Backfill the sidecar for legacy sessions so future resumes are explicit.
        if not config_path.exists():
            with open(config_path, 'w') as f:
                yaml.safe_dump({'gait': gait}, f)
    else:
        popsize = args.popsize or (4 + int(3 * math.log(n_dims)))
        cma_opts = {
            'bounds': [lower, upper],
            'CMA_stds': stds,
            'popsize': popsize,
            'seed': args.seed,
            'verbose': -9,  # silence cma's own stdout chatter
        }
        # CMAEvolutionStrategy expects sigma0 as a scalar; CMA_stds rescales
        # per-dimension stds relative to it. Using sigma0=1 and per-dim stds
        # from --sigma-frac gives the intended initial spread.
        es = cma.CMAEvolutionStrategy(x0, 1.0, cma_opts)
        eval_count = 0
        best_J = -float('inf')
        best_x = None
        with open(log_path, 'w', newline='') as f:
            csv.writer(f).writerow(log_columns(gait))
        with open(config_path, 'w') as f:
            yaml.safe_dump({'gait': gait}, f)
        print(f'Starting fresh CMA-ES session: {session_dir}', flush=True)
        print(f'Gait: {gait}', flush=True)
        print(f'Budget: {args.budget} evals, popsize={popsize}, n_dims={n_dims}', flush=True)
        print('Search-vector init (CMA-internal layout):', flush=True)
        for name, x_val, std in zip(PARAM_NAMES, x0, stds):
            lo, hi = PARAM_BOUNDS[name]
            print(f'  {name:16s} = {x_val:+.4f}  '
                  f'σ0={std:.4f}  bounds=[{lo:+.4f}, {hi:+.4f}]', flush=True)

    # CMA learns from a complete generation (popsize evals followed by tell()).
    # If --budget isn't a multiple of popsize, the trailing partial generation
    # would consume sim time without informing the search. Round up so every
    # eval contributes to a covariance update.
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
            iteration = es.countiter  # 0-based; advances after tell()
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
                    args.wait_seconds, args.period, args.alpha, gait,
                )
                fs.append(-J)  # CMA minimizes; we maximize J

                wall = time.time() - t_start
                eval_durations.append(wall)
                eval_count += 1

                with open(log_path, 'a', newline='') as f:
                    csv.writer(f).writerow(
                        build_log_row(gait, iteration, eval_count, wall,
                                      physical, info, J)
                    )

                if J > best_J:
                    best_J = J
                    best_x = list(x)
                    write_best_params(best_path, vector_to_params(best_x, defaults))

                avg = sum(eval_durations) / len(eval_durations)
                eta_s = avg * (budget - eval_count)
                if gait == 'sidewinding':
                    print(f'  J={J:.6f}  S={info.get("S", 0):.4f}  '
                          f'v={info.get("v", 0):.4f}  '
                          f'status={info.get("status", "?")}', flush=True)
                else:  # rotating
                    print(f'  J={J:.6f}  R={info.get("R", 0):.4f}  '
                          f'omega={info.get("omega", 0):.4f}  '
                          f'drift={info.get("drift_speed", 0):.6f}  '
                          f'status={info.get("status", "?")}', flush=True)
                print(f'  best so far: J={best_J:.6f}  '
                      f'wall {wall:.0f}s  ETA {eta_s / 60:.1f} min', flush=True)

            if full_pop:
                es.tell(xs, fs)
                # Checkpoint after every complete generation.
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
            print(f'Best J = {best_J:.6f}', flush=True)
            print('Physical params (these are what get fed to the controller):',
                  flush=True)
            for name in PHYSICAL_PARAM_NAMES:
                print(f'  {name:14s} = {best_params[name]:+.6f}', flush=True)
            print(f'\nBest params written to: {best_path}', flush=True)
            print(f'Replay it with:', flush=True)
            print(f'  ros2 launch snake_sim snake_sim_launch.py '
                  f'controller_params_file:={best_path}', flush=True)

    return 0


if __name__ == '__main__':
    sys.exit(main())
