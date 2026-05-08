#!/usr/bin/env python3
"""
Parameter sweep driver for the snake simulation.

Reads a sweep config YAML listing named runs with per-run controller parameters,
then for each run:
  1. Writes a ROS params YAML file with that run's overrides
  2. Launches the simulation with those parameters (headless, no RViz)
  3. Waits, captures the scene snapshot (PNG + CSVs)
  4. Cleans up every process it started

Usage:
    python3 scripts/run_sweep.py config/sweeps/example_sweep.yaml

Sweep config format (see config/sweeps/example_sweep.yaml):

    # Applied to every run unless overridden inside the run entry.
    wait_seconds: 300
    output_name: amplitude_sweep

    runs:
      - name: A_v_1.0
        params:
          A_v: 1.0
      - name: A_v_2.0
        params:
          A_v: 2.0
"""
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml


# Path to the canonical defaults file (source tree, relative to this script).
# Resolving from __file__ rather than the installed share keeps edits to
# default_controller_params.yaml effective without requiring a colcon build.
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULTS_FILE = REPO_ROOT / 'config' / 'default_controller_params.yaml'
CREATE_GIF_SCRIPT = REPO_ROOT / 'scripts' / 'create_gif.py'

# create_gif.py needs pandas/matplotlib which live in the scripts/.venv
# virtualenv, while this script itself needs yaml/rclpy from system Python.
# Invoke the GIF renderer with the venv's interpreter when it exists.
VENV_PYTHON = REPO_ROOT / 'scripts' / '.venv' / 'bin' / 'python3'

# Node name used inside the ROS params YAML files.
CONTROLLER_NODE_NAME = 'movement_controller_node'


# Processes owned by the simulation pipeline. Used to sweep orphans between runs.
SIM_PROCESS_PATTERNS = [
    'snake_sim_launch.py',
    'ros_gz_bridge',
    'ros_gz_sim',
    'gz sim',
    'ruby .*gz',
    'robot_state_publisher',
    'sidewinding_movement_controller',
    'center_of_mass_calculator',
    'odometry_tf_broadcaster',
    'robot_body_logger',
    'controller_manager',
    'joint_state_broadcaster',
    'movement_controller',
]


def pgid_alive(pgid: int) -> bool:
    return subprocess.run(
        ['pgrep', '-g', str(pgid)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def kill_pgid_gracefully(pgid: int, timeout_s: int = 10) -> None:
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    for _ in range(timeout_s):
        if not pgid_alive(pgid):
            return
        time.sleep(1)
    print(f'  SIGTERM timed out, sending SIGKILL to pgid {pgid}...', flush=True)
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        return
    time.sleep(1)


def sweep_residuals() -> None:
    for pat in SIM_PROCESS_PATTERNS:
        subprocess.run(['pkill', '-TERM', '-f', pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    for pat in SIM_PROCESS_PATTERNS:
        subprocess.run(['pkill', '-KILL', '-f', pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_ros2_daemon() -> None:
    subprocess.run(['ros2', 'daemon', 'stop'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def assert_clean() -> None:
    """Exit if any simulation process is still running."""
    residuals = []
    for pat in SIM_PROCESS_PATTERNS:
        r = subprocess.run(['pgrep', '-af', pat], capture_output=True, text=True)
        if r.returncode == 0:
            for line in r.stdout.strip().splitlines():
                if 'run_sweep.py' in line:
                    continue
                residuals.append(line)
    if residuals:
        print('ERROR: residual processes remain after cleanup:', file=sys.stderr)
        for line in residuals:
            print(f'  {line}', file=sys.stderr)
        sys.exit(1)


def load_default_params() -> dict:
    """Return the default controller parameters from default_controller_params.yaml."""
    if not DEFAULTS_FILE.exists():
        print(f'WARNING: {DEFAULTS_FILE} not found — runs will use only overrides.',
              file=sys.stderr)
        return {}
    with open(DEFAULTS_FILE) as f:
        data = yaml.safe_load(f) or {}
    return data.get(CONTROLLER_NODE_NAME, {}).get('ros__parameters', {}) or {}


def effective_params(defaults: dict, overrides: dict) -> dict:
    """Merge per-run overrides on top of the defaults."""
    merged = dict(defaults)
    merged.update(overrides or {})
    return merged


def write_params_file(path: Path, params: dict) -> None:
    """Write a ROS 2 params YAML file the launch file will load."""
    ros_params = {
        CONTROLLER_NODE_NAME: {
            'ros__parameters': params or {},
        }
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(ros_params, f, default_flow_style=False, sort_keys=False)


def run_one(run_name: str, effective: dict, wait_seconds: int, output_dir: Path,
            *, take_snapshot: bool = True, render_gif: bool = True,
            quiet: bool = False) -> Path:
    """Run one simulation. Returns the path to the body trajectory CSV.

    `take_snapshot` and `render_gif` are off-switches for the optimizer driver,
    which evaluates many parameter sets back-to-back and only needs the CSV.
    `quiet` redirects ros2 launch + snapshot stdout/stderr to a per-eval log
    file, so the optimizer's own progress output isn't drowned out. The log
    is still written to disk under output_dir for post-mortem inspection.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # This file is both the input to the launch and the record of what ran.
    params_file = output_dir / f'{run_name}_params.yaml'
    write_params_file(params_file, effective)

    snapshot_path = output_dir / f'{run_name}.png'
    trajectory_log_path = output_dir / f'{run_name}_body_trajectory.csv'
    gif_path = output_dir / f'{run_name}.gif'
    sim_log_path = output_dir / f'{run_name}_sim.log'

    print(f'  Launching simulation (params file: {params_file.name})...', flush=True)

    log_handle = open(sim_log_path, 'w') if quiet else None
    sim_stdout = log_handle if quiet else None
    sim_stderr = subprocess.STDOUT if quiet else None

    proc = subprocess.Popen(
        [
            'ros2', 'launch', 'snake_sim', 'snake_sim_launch.py',
            'use_rviz:=false',
            f'controller_params_file:={params_file}',
            f'trajectory_log_path:={trajectory_log_path}',
        ],
        preexec_fn=os.setsid,
        stdin=subprocess.DEVNULL,
        stdout=sim_stdout,
        stderr=sim_stderr,
    )
    pgid = proc.pid

    try:
        print(f'  Simulating for {wait_seconds}s (pgid={pgid})...', flush=True)
        time.sleep(wait_seconds)

        if take_snapshot:
            print('  Capturing snapshot...', flush=True)
            subprocess.run(
                [
                    'ros2', 'run', 'snake_sim', 'scene_snapshot',
                    '--ros-args',
                    '-p', f'output_path:={snapshot_path}',
                    '-p', 'wait_seconds:=3.0',
                ],
                check=False,
                stdout=sim_stdout,
                stderr=sim_stderr,
            )
        print(f'  Saved outputs to: {output_dir}', flush=True)
    finally:
        print('  Stopping simulation...', flush=True)
        kill_pgid_gracefully(pgid, timeout_s=10)
        sweep_residuals()
        stop_ros2_daemon()
        assert_clean()
        if log_handle is not None:
            log_handle.close()
        # Let DDS multicast state unwind before next run.
        time.sleep(3)

    if render_gif:
        if trajectory_log_path.exists():
            gif_python = str(VENV_PYTHON) if VENV_PYTHON.exists() else 'python3'
            print(f'  Rendering GIF (via {gif_python})...', flush=True)
            subprocess.run(
                [gif_python, str(CREATE_GIF_SCRIPT), str(trajectory_log_path), str(gif_path)],
                check=False,
            )
        else:
            print(f'  Skipping GIF: {trajectory_log_path.name} not found.', flush=True)

    return trajectory_log_path


def main() -> int:
    parser = argparse.ArgumentParser(description='Parameter sweep for snake simulation.')
    parser.add_argument('sweep_file', type=Path,
                        help='Path to sweep config YAML')
    args = parser.parse_args()

    with open(args.sweep_file) as f:
        sweep = yaml.safe_load(f)

    default_wait = int(sweep.get('wait_seconds', 300))
    output_name = sweep.get('output_name', 'sweep')
    runs = sweep.get('runs', [])
    if not runs:
        print('ERROR: sweep config has no runs.', file=sys.stderr)
        return 1

    base_output_dir = Path.cwd() / 'sweep_output' / output_name
    base_output_dir.mkdir(parents=True, exist_ok=True)

    defaults = load_default_params()

    print(f'Sweep: {len(runs)} runs, output under {base_output_dir}')
    print(f'Defaults loaded from: {DEFAULTS_FILE}')

    # Clear residuals from prior invocations before starting.
    sweep_residuals()
    stop_ros2_daemon()

    run_durations = []

    for i, run in enumerate(runs, start=1):
        run_name = run['name']
        overrides = run.get('params', {})
        effective = effective_params(defaults, overrides)
        wait_seconds = int(run.get('wait_seconds', default_wait))

        print('')
        print(f'========== Run {i}/{len(runs)}: {run_name} ==========')
        print(f'  Overrides: {overrides}')
        print(f'  Effective params: {effective}')

        run_dir = base_output_dir / run_name
        t_start = time.time()
        run_one(run_name, effective, wait_seconds, run_dir,
                render_gif=False, quiet=True)
        wall = time.time() - t_start
        run_durations.append(wall)

        avg = sum(run_durations) / len(run_durations)
        eta_s = avg * (len(runs) - i)
        print(f'  Run {run_name} complete.  '
              f'wall {wall:.0f}s  ETA {eta_s / 60:.1f} min')

    print('')
    print(f'All {len(runs)} runs complete. Results in: {base_output_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
