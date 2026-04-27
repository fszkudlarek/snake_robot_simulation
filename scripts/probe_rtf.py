#!/usr/bin/env python3
"""
Measure the real-time factor (RTF) achieved by the current Gazebo configuration.

Launches the snake sim headless, waits past startup, then samples Gazebo's
/world/empty/stats topic for a measurement window. RTF = Δsim_time / Δreal_time
across the window — unaffected by startup overhead because both values are
pulled from the same protobuf message at start and end of the window.

Usage:
    python3 scripts/probe_rtf.py [--warmup 20] [--window 10]
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]

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


def sweep_residuals() -> None:
    for pat in SIM_PROCESS_PATTERNS:
        subprocess.run(['pkill', '-TERM', '-f', pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    for pat in SIM_PROCESS_PATTERNS:
        subprocess.run(['pkill', '-KILL', '-f', pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill_pgid(pgid: int, timeout_s: int = 10) -> None:
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    for _ in range(timeout_s):
        r = subprocess.run(['pgrep', '-g', str(pgid)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode != 0:
            return
        time.sleep(1)
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        return


# Captures every `X { sec: S nsec: N }` pair so we can match start/end of window.
TIME_FIELD_RE = re.compile(
    r'(sim_time|real_time)\s*\{\s*sec:\s*(-?\d+)\s*nsec:\s*(-?\d+)\s*\}',
    re.DOTALL,
)


def parse_stats_messages(text: str) -> list[dict]:
    """Parse protobuf text-format stats dumps into a list of {sim_s, real_s}."""
    matches = TIME_FIELD_RE.findall(text)
    samples = []
    current = {}
    for field, sec, nsec in matches:
        current[field] = int(sec) + int(nsec) * 1e-9
        if 'sim_time' in current and 'real_time' in current:
            samples.append(current)
            current = {}
    return samples


def main() -> int:
    parser = argparse.ArgumentParser(description='Measure Gazebo RTF.')
    parser.add_argument('--warmup', type=float, default=20.0,
                        help='Seconds to wait after launch before measuring (default: 20)')
    parser.add_argument('--window', type=float, default=10.0,
                        help='Measurement window in wall seconds (default: 10)')
    parser.add_argument('--world', type=str, default='empty',
                        help='Gazebo world name (default: empty)')
    args = parser.parse_args()

    print(f'Clearing residuals...', flush=True)
    sweep_residuals()
    subprocess.run(['ros2', 'daemon', 'stop'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    trajectory_log = Path('/tmp/rtf_probe_body_trajectory.csv')
    launch_log = open('/tmp/rtf_probe_launch.log', 'w')

    print(f'Launching simulation headless...', flush=True)
    proc = subprocess.Popen(
        [
            'ros2', 'launch', 'snake_sim', 'snake_sim_launch.py',
            'use_rviz:=false',
            f'trajectory_log_path:={trajectory_log}',
        ],
        preexec_fn=os.setsid,
        stdin=subprocess.DEVNULL,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
    )
    pgid = proc.pid

    try:
        print(f'Warmup: sleeping {args.warmup}s for sim to reach steady state...',
              flush=True)
        time.sleep(args.warmup)

        print(f'Measuring over {args.window}s wall-clock window via '
              f'/world/{args.world}/stats...', flush=True)
        stats_proc = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{args.world}/stats'],
            capture_output=True, text=True, timeout=args.window + 2,
        )
        text = stats_proc.stdout + stats_proc.stderr
    except subprocess.TimeoutExpired as e:
        # Expected: gz topic -e runs until killed by timeout.
        def _as_text(x) -> str:
            if x is None:
                return ''
            if isinstance(x, bytes):
                return x.decode('utf-8', errors='replace')
            return x
        text = _as_text(e.stdout) + _as_text(e.stderr)
    finally:
        print('Stopping simulation...', flush=True)
        kill_pgid(pgid, timeout_s=10)
        sweep_residuals()
        subprocess.run(['ros2', 'daemon', 'stop'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        launch_log.close()

    samples = parse_stats_messages(text)
    if len(samples) < 2:
        print(f'ERROR: only {len(samples)} stats message(s) captured — '
              f'sim may not have started in time. See /tmp/rtf_probe_launch.log',
              file=sys.stderr)
        return 1

    first, last = samples[0], samples[-1]
    d_sim = last['sim_time'] - first['sim_time']
    d_real = last['real_time'] - first['real_time']
    rtf = d_sim / d_real if d_real > 0 else float('nan')

    print('')
    print(f'Samples captured:    {len(samples)}')
    print(f'Δ sim_time:          {d_sim:.3f} s')
    print(f'Δ real_time:         {d_real:.3f} s')
    print(f'Achieved RTF:        {rtf:.2f}x real-time')
    return 0


if __name__ == '__main__':
    sys.exit(main())
