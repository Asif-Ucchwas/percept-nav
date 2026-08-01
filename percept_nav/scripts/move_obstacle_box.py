import subprocess
import time
import math

# Oscillates the moving_obstacle_box back and forth along x, between
# x=0.4 and x=1.6, using direct pose-setting via gz service rather than
# the VelocityControl plugin (which did not produce reliable physics-based
# motion in testing -- see DEVLOG for the debugging process).

MODEL_NAME = "moving_obstacle_box"
CENTER_X = 1.0
AMPLITUDE = 0.6
PERIOD_SEC = 4.0
DURATION_SEC = 20.0
RATE_HZ = 10.0


def set_pose(x, y=0.0, z=0.15):
    req = f'name: "{MODEL_NAME}", position: {{x: {x}, y: {y}, z: {z}}}'
    subprocess.run([
        "gz", "service", "-s", "/world/default/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", req
    ], capture_output=True)


def main():
    start = time.time()
    dt = 1.0 / RATE_HZ
    while time.time() - start < DURATION_SEC:
        t = time.time() - start
        x = CENTER_X + AMPLITUDE * math.sin(2 * math.pi * t / PERIOD_SEC)
        set_pose(x)
        time.sleep(dt)
    print("Movement sequence complete")


if __name__ == "__main__":
    main()
