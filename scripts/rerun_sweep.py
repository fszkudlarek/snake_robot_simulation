#!/usr/bin/env python3
"""
Re-run individual sweep runs in place, reusing their recorded parameters.

Given one or more paths to existing per-run output directories produced by
run_sweep.py, this script re-launches each run with the exact parameters stored
in that run's '<run_name>_params.yaml', and writes the fresh outputs back into
the same directory (overwriting the originals). The produced artifacts have the
same form as a normal sweep run (params YAML, snapshot PNG, body-trajectory CSV,
sim log, plus whatever the logger nodes emit).

Usage:
    python3 scripts/rerun_sweep.py \\
        sweep_output/locomotion_principles/sidewinding/vertical_offset/runs/O_v_-50_deg

    # multiple runs at once:
    python3 scripts/rerun_sweep.py \\
        sweep_output/.../runs/O_v_-50_deg \\
        sweep_output/.../runs/O_v_50_deg

The simulation wait time is fixed at 120 seconds, matching the sweep configs.
"""
import argparse
import sys
import time
from pathlib import Path

import yaml

# This script lives next to run_sweep.py, so its directory is already on
# sys.path[0] when invoked as a script. Reuse run_sweep's machinery verbatim so
# the re-run output is byte-for-byte the same kind of output a sweep produces.
import run_sweep
from run_sweep import (
    CONTROLLER_NODE_NAME,
    run_one,
    stop_ros2_daemon,
    sweep_residuals,
)

# Fixed simulation duration per run (seconds), matching the sweep configs.
WAIT_SECONDS = 120


def find_params_file(run_dir: Path) -> Path:
    """Return the '<run_name>_params.yaml' inside run_dir.

    Errors if zero or more than one match is found so the caller never reruns
    against an ambiguous parameter set.
    """
    matches = sorted(run_dir.glob('*_params.yaml'))
    if not matches:
        raise FileNotFoundError(f'no *_params.yaml found in {run_dir}')
    if len(matches) > 1:
        names = ', '.join(m.name for m in matches)
        raise ValueError(f'multiple *_params.yaml found in {run_dir}: {names}')
    return matches[0]


def load_effective_params(params_file: Path) -> dict:
    """Read the controller parameters recorded in a run's params YAML.

    The file written by run_sweep already holds the *effective* params (defaults
    merged with per-run overrides) under movement_controller_node/ros__parameters,
    so they can be replayed directly with no further merging.
    """
    with open(params_file) as f:
        data = yaml.safe_load(f) or {}
    params = data.get(CONTROLLER_NODE_NAME, {}).get('ros__parameters', {})
    if not params:
        raise ValueError(
            f'{params_file} has no {CONTROLLER_NODE_NAME}/ros__parameters params')
    return params


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Re-run existing sweep runs in place using their recorded params.')
    parser.add_argument('run_paths', type=Path, nargs='+',
                        help='Path(s) to per-run output dir(s), e.g. '
                             'sweep_output/.../runs/O_v_-50_deg')
    args = parser.parse_args()

    # Resolve and validate every target up front so a typo doesn't surface only
    # after a long simulation has already run.
    targets = []  # (run_name, effective_params, run_dir)
    for raw in args.run_paths:
        run_dir = raw if raw.is_absolute() else Path.cwd() / raw
        run_dir = run_dir.resolve()
        if not run_dir.is_dir():
            print(f'ERROR: not a directory: {run_dir}', file=sys.stderr)
            return 1
        try:
            params_file = find_params_file(run_dir)
            effective = load_effective_params(params_file)
        except (FileNotFoundError, ValueError) as e:
            print(f'ERROR: {e}', file=sys.stderr)
            return 1
        # run_one names every output after run_name; derive it from the params
        # file so the rewritten files reuse the exact original names.
        run_name = params_file.name[:-len('_params.yaml')]
        targets.append((run_name, effective, run_dir))

    print(f'Re-running {len(targets)} run(s), wait {WAIT_SECONDS}s each:')
    for run_name, _, run_dir in targets:
        print(f'  {run_name}  ->  {run_dir}')

    # Clear residuals from prior invocations before starting.
    sweep_residuals()
    stop_ros2_daemon()

    run_durations = []
    for i, (run_name, effective, run_dir) in enumerate(targets, start=1):
        print('')
        print(f'========== Re-run {i}/{len(targets)}: {run_name} ==========')
        print(f'  Output dir: {run_dir}')
        print(f'  Effective params: {effective}')

        t_start = time.time()
        # Match run_sweep.main()'s call so output form is identical.
        run_one(run_name, effective, WAIT_SECONDS, run_dir,
                render_gif=False, quiet=True)
        wall = time.time() - t_start
        run_durations.append(wall)

        avg = sum(run_durations) / len(run_durations)
        eta_s = avg * (len(targets) - i)
        print(f'  Re-run {run_name} complete.  '
              f'wall {wall:.0f}s  ETA {eta_s / 60:.1f} min')

    print('')
    print(f'All {len(targets)} re-run(s) complete.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
