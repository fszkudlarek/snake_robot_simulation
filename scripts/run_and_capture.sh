#!/bin/bash
# Launches the snake simulation, waits, captures a scene snapshot, then repeats.
#
# Usage: ./run_and_capture.sh [NUM_RUNS] [WAIT_SECONDS] [PREFIX]
#   NUM_RUNS     - number of simulation runs (default: 5)
#   WAIT_SECONDS - how long to let each run execute before snapshot (default: 300 = 5 min)
#   PREFIX       - filename prefix for all snapshots (default: "run")

NUM_RUNS=${1:-5}
WAIT_SECONDS=${2:-300}
PREFIX=${3:-run}
OUTPUT_DIR="$(pwd)/screenshots"

# Executables owned by the simulation pipeline. Used to verify clean shutdown.
# Matched against full command line via pgrep -f, so entries can be substrings.
SIM_PROCESS_PATTERNS=(
    "snake_sim_launch.py"
    "ros_gz_bridge"
    "ros_gz_sim"
    "gz sim"
    "ruby .*gz"
    "robot_state_publisher"
    "sidewinding_movement_controller"
    "center_of_mass_calculator"
    "odometry_tf_broadcaster"
    "controller_manager"
    "joint_state_broadcaster"
    "movement_controller"
)

mkdir -p "$OUTPUT_DIR"

# Kills PGID with SIGTERM, polls up to timeout seconds, then SIGKILL if needed.
kill_pgid() {
    local pgid="$1"
    local timeout="${2:-10}"
    kill -TERM -- -"$pgid" 2>/dev/null || true
    for _ in $(seq 1 "$timeout"); do
        # If no process remains in the group, we're done.
        if ! pgrep -g "$pgid" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    echo "SIGTERM timed out, sending SIGKILL to pgid $pgid..."
    kill -KILL -- -"$pgid" 2>/dev/null || true
    sleep 1
}

# Returns 0 if no simulation process is running, 1 otherwise. Prints offenders.
check_clean() {
    local found=0
    for pat in "${SIM_PROCESS_PATTERNS[@]}"; do
        local hits
        hits=$(pgrep -af "$pat" 2>/dev/null | grep -v "run_and_capture.sh" || true)
        if [ -n "$hits" ]; then
            if [ "$found" -eq 0 ]; then
                echo "WARNING: residual processes detected:"
                found=1
            fi
            echo "$hits"
        fi
    done
    return "$found"
}

# Sweeps any residual processes matching our patterns (orphans from previous runs).
sweep_residuals() {
    for pat in "${SIM_PROCESS_PATTERNS[@]}"; do
        pkill -TERM -f "$pat" 2>/dev/null || true
    done
    sleep 2
    for pat in "${SIM_PROCESS_PATTERNS[@]}"; do
        pkill -KILL -f "$pat" 2>/dev/null || true
    done
}

echo "Will run simulation $NUM_RUNS times, waiting ${WAIT_SECONDS}s per run."
echo "Snapshots saved to: $OUTPUT_DIR"

# Clear any residuals from prior (possibly failed) invocations.
sweep_residuals
ros2 daemon stop >/dev/null 2>&1 || true

for i in $(seq 1 "$NUM_RUNS"); do
    echo ""
    echo "========== Run $i / $NUM_RUNS =========="

    # Launch the simulation in its own process group via setsid.
    # This ensures `kill -- -$PGID` reaches every descendant.
    echo "Launching simulation..."
    setsid ros2 launch snake_sim snake_sim_launch.py use_rviz:=false </dev/null &
    LAUNCH_PID=$!
    # With setsid, the new process is the leader of a new group whose PGID == PID.
    LAUNCH_PGID=$LAUNCH_PID

    # Let the simulation run.
    echo "Simulation running (pgid=$LAUNCH_PGID), waiting ${WAIT_SECONDS}s..."
    sleep "$WAIT_SECONDS"

    # Capture the scene via the headless snapshot node (collects data for 3s, then saves).
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    SCREENSHOT_FILE="$OUTPUT_DIR/${PREFIX}_${i}_${TIMESTAMP}.png"

    echo "Capturing scene snapshot..."
    ros2 run snake_sim scene_snapshot --ros-args \
        -p output_path:="$SCREENSHOT_FILE" \
        -p wait_seconds:=3.0

    echo "Snapshot saved: $SCREENSHOT_FILE"

    # Stop the simulation: terminate the whole process group with SIGTERM,
    # poll for exit, escalate to SIGKILL if needed.
    echo "Stopping simulation..."
    kill_pgid "$LAUNCH_PGID" 10

    # Sweep any orphans that escaped the process group (shouldn't happen, but safe).
    sweep_residuals

    # Stop ros2 daemon so discovery cache from this run doesn't bleed into the next.
    ros2 daemon stop >/dev/null 2>&1 || true

    # Verify clean state before next iteration.
    if ! check_clean; then
        echo "ERROR: residual processes remain after cleanup. Aborting."
        exit 1
    fi

    # Give DDS multicast state a moment to time out before next run starts.
    sleep 3

    echo "Run $i complete."
done

echo ""
echo "All $NUM_RUNS runs complete. Snapshots in: $OUTPUT_DIR"
