"""
Capture RViz window screenshots at exact gait phases.

Subscribes to /movement_controller/phase (std_msgs/Float64, wrapped to [0, 2π))
published by the sidewinding movement controller, and grabs the RViz window with
ImageMagick's `import` every time the (unwrapped) gait phase crosses one of the
target phases. Because triggering is driven by the published phase rather than a
wall-clock timer, the spacing between frames is phase-exact regardless of the
simulation's real-time factor.

By default it captures `frames_per_period` evenly spaced phases over
`num_periods` consecutive periods, starting from the first full period boundary
(phase ≈ 0) so frame 0 lands at a meaningful φ = 0 reference.

Usage (X11; requires `xdotool` and ImageMagick `import` on PATH):
  ros2 run snake_sim rviz_phase_capture \
      --ros-args -p output_dir:=/tmp/gait_frames \
                 -p frames_per_period:=4 \
                 -p num_periods:=2

Optional:
  -p rviz_window:='rviz'      # regex matched against window class then name
  -p crop:='WxH+X+Y'          # ImageMagick crop geometry (e.g. just the 3D view)
  -p start_on_zero:=true      # begin at the next phase wrap (φ=0); else capture
                              #   immediately from the first received phase
"""
import math
import os
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

TWO_PI = 2.0 * math.pi


class RvizPhaseCapture(Node):
    def __init__(self):
        super().__init__('rviz_phase_capture')

        self.declare_parameter('output_dir', 'gait_frames')
        self.declare_parameter('frames_per_period', 4)
        self.declare_parameter('num_periods', 2)
        self.declare_parameter('rviz_window', 'rviz')
        self.declare_parameter('crop', '')
        self.declare_parameter('start_on_zero', True)

        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.frames_per_period = int(self.get_parameter('frames_per_period').value)
        self.num_periods = int(self.get_parameter('num_periods').value)
        self.rviz_window = self.get_parameter('rviz_window').value
        self.crop = self.get_parameter('crop').value
        self.start_on_zero = bool(self.get_parameter('start_on_zero').value)

        os.makedirs(self.output_dir, exist_ok=True)

        # Target phases as cumulative (unwrapped) radians: k · 2π/N for
        # k = 0 .. frames_per_period·num_periods − 1.
        step = TWO_PI / self.frames_per_period
        self.targets = [
            k * step
            for k in range(self.frames_per_period * self.num_periods)
        ]

        self.window_id = self._find_rviz_window()
        if self.window_id is None:
            self.get_logger().error(
                f"No RViz window matching '{self.rviz_window}' found. "
                "Is RViz running on this display?")
            raise SystemExit(1)

        # Raise the window so it is fully visible; we capture the composited
        # root framebuffer and crop to this window's rectangle (XGetImage on the
        # RViz window directly fails because it is an OpenGL-rendered surface).
        subprocess.run(['xdotool', 'windowactivate', '--sync', self.window_id],
                       capture_output=True, text=True, check=False)
        self.get_logger().info(f'Capturing RViz window id {self.window_id}')

        # Phase-tracking state.
        self.started = not self.start_on_zero
        self.prev_phase = None
        self.wrap_count = 0
        self.next_idx = 0

        self.create_subscription(
            Float64, '/movement_controller/phase', self._on_phase, 50)

        self.get_logger().info(
            f'Waiting for phase=0 boundary to capture {len(self.targets)} frames '
            f'({self.frames_per_period}/period × {self.num_periods} periods) '
            f'into {self.output_dir}' if self.start_on_zero else
            f'Capturing {len(self.targets)} frames into {self.output_dir}')

    def _find_rviz_window(self):
        """Return the largest visible matching window id.

        A single RViz process owns several X windows (tiny Qt helper/selection
        windows as well as the main frame), so taking the first hit can land on a
        10x10 utility window. We match by window class only — matching by title
        would also catch unrelated windows that merely mention "rviz" (e.g. an
        editor showing this file) — and keep the largest visible match.
        """
        try:
            out = subprocess.run(
                ['xdotool', 'search', '--onlyvisible', '--class',
                 self.rviz_window],
                capture_output=True, text=True, check=False)
        except FileNotFoundError:
            self.get_logger().error('xdotool not found on PATH.')
            return None
        candidates = {i for i in out.stdout.split() if i.strip()}

        best_id, best_area = None, 0
        for win_id in candidates:
            geo = self._window_geometry(win_id)
            if geo is None:
                continue
            wh, _, _ = geo.partition('+')
            w, _, h = wh.partition('x')
            area = int(w) * int(h)
            if area > best_area:
                best_id, best_area = win_id, area

        # Ignore tiny helper windows; the real frame is far larger.
        if best_area < 100 * 100:
            return None
        return best_id

    def _on_phase(self, msg: Float64):
        phase = msg.data

        if self.prev_phase is None:
            self.prev_phase = phase
            return

        # Detect a wrap (phase jumps from ≈2π back to ≈0).
        wrapped = phase < self.prev_phase - math.pi
        self.prev_phase = phase

        if not self.started:
            # Hold until the first wrap so frame 0 sits at φ ≈ 0.
            if wrapped:
                self.started = True
                self.wrap_count = 0
            else:
                return
        elif wrapped:
            self.wrap_count += 1

        unwrapped = self.wrap_count * TWO_PI + phase

        # Fire every target the phase has now reached or passed.
        while (self.next_idx < len(self.targets)
               and unwrapped >= self.targets[self.next_idx]):
            self._capture(self.next_idx, self.targets[self.next_idx])
            self.next_idx += 1

        if self.next_idx >= len(self.targets):
            self.get_logger().info(
                f'Captured all {len(self.targets)} frames. Done.')
            raise SystemExit(0)

    def _window_geometry(self, win_id=None):
        """Return 'WxH+X+Y' for the given window (default RViz), or None."""
        out = subprocess.run(
            ['xdotool', 'getwindowgeometry', '--shell',
             win_id or self.window_id],
            capture_output=True, text=True, check=False)
        if out.returncode != 0:
            return None
        vals = {}
        for line in out.stdout.splitlines():
            if '=' in line:
                key, _, value = line.partition('=')
                vals[key.strip()] = value.strip()
        try:
            return (f"{vals['WIDTH']}x{vals['HEIGHT']}"
                    f"+{vals['X']}+{vals['Y']}")
        except KeyError:
            return None

    def _capture(self, idx, target_phase):
        period = idx // self.frames_per_period
        deg = round(math.degrees(target_phase % TWO_PI))
        path = os.path.join(
            self.output_dir,
            f'frame_{idx:02d}_p{period}_phase{deg:03d}.png')

        # An explicit `crop` param overrides the auto-detected window rectangle.
        geometry = self.crop or self._window_geometry()
        if geometry is None:
            self.get_logger().error(
                f'Could not determine window geometry for {path}.')
            return

        # Capture the composited root window, then crop to the RViz rectangle.
        cmd = ['import', '-window', 'root', '-crop', geometry, '+repage', path]
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if result.returncode != 0:
            self.get_logger().error(
                f'import failed for {path}: {result.stderr.strip()}')
        else:
            self.get_logger().info(
                f'Saved {path}  (period {period}, φ≈{deg}°)')


def main():
    rclpy.init()
    node = RvizPhaseCapture()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
