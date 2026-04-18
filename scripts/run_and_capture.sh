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

mkdir -p "$OUTPUT_DIR"

echo "Will run simulation $NUM_RUNS times, waiting ${WAIT_SECONDS}s per run."
echo "Snapshots saved to: $OUTPUT_DIR"

for i in $(seq 1 "$NUM_RUNS"); do
    echo ""
    echo "========== Run $i / $NUM_RUNS =========="

    # Launch the simulation
    echo "Launching simulation..."
    ros2 launch snake_sim snake_sim_launch.py use_rviz:=false &
    LAUNCH_PID=$!

    # Let the simulation run
    echo "Simulation running, waiting ${WAIT_SECONDS}s..."
    sleep "$WAIT_SECONDS"

    # Capture the scene via the headless snapshot node (collects data for 3s, then saves)
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    SCREENSHOT_FILE="$OUTPUT_DIR/${PREFIX}_${i}_${TIMESTAMP}.png"

    echo "Capturing scene snapshot..."
    ros2 run snake_sim scene_snapshot --ros-args \
        -p output_path:="$SCREENSHOT_FILE" \
        -p wait_seconds:=3.0

    echo "Snapshot saved: $SCREENSHOT_FILE"

    # Kill the simulation (all ROS 2 nodes and Gazebo)
    echo "Stopping simulation..."
    kill -- -"$LAUNCH_PID" 2>/dev/null  # Kill the process group
    # Clean up any remaining processes
    pkill -f "gz sim" 2>/dev/null
    pkill -f "ros2" 2>/dev/null
    pkill -f "ruby.*gz" 2>/dev/null

    # Wait for everything to shut down
    sleep 5

    echo "Run $i complete."
done

echo ""
echo "All $NUM_RUNS runs complete. Snapshots in: $OUTPUT_DIR"
