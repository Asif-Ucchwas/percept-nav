import subprocess
import time
import math
import sys

# Spawns N moving obstacles and oscillates them at a configurable speed,
# to find the point where Nav2 navigation starts failing (collision,
# stuck robot, or navigation timeout). See DEVLOG for methodology.

SDF_PATH = "/home/jannatul/ros2_ws/src/percept_nav_repo/percept_nav/worlds/moving_box.sdf"


def spawn_box(name, x, y):
    req = f'sdf_filename: "{SDF_PATH}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: 0.15}}}}'
    result = subprocess.run([
        "gz", "service", "-s", "/world/default/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", req
    ], capture_output=True, text=True)
    return "true" in result.stdout.lower()


def set_pose(name, x, y=0.25, z=0.15):
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'
    subprocess.run([
        "gz", "service", "-s", "/world/default/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", req
    ], capture_output=True)


def get_robot_position():
    result = subprocess.run(
        ["ros2", "topic", "echo", "/odom", "--once"],
        capture_output=True, text=True, timeout=5
    )
    lines = result.stdout.splitlines()
    x = y = None
    for i, line in enumerate(lines):
        if line.strip() == "position:" and x is None:
            x = float(lines[i + 1].split(":")[1])
            y = float(lines[i + 1 + 1].split(":")[1])
            break
    return x, y


def run_stress_level(num_obstacles, speed, duration, center_x=1.0, amplitude=0.6, period=4.0):
    print(f"\n=== Stress level: {num_obstacles} obstacles, speed factor {speed} ===")

    names = [f"stress_box_{i}" for i in range(num_obstacles)]
    for i, name in enumerate(names):
        start_x = 0.5 + i * 0.4
        ok = spawn_box(name, start_x, 0.25)
        print(f"  Spawned {name} at x={start_x}: {'OK' if ok else 'FAILED'}")
        time.sleep(0.3)

    start = time.time()
    dt = 1.0 / (10.0 * speed)
    collision_detected = False
    warmup_seconds = 3.0  # give Nav2 a real chance to see and react

    while time.time() - start < duration:
        elapsed = time.time() - start
        t = (time.time() - start) * speed
        for i, name in enumerate(names):
            phase_offset = i * (2 * math.pi / max(num_obstacles, 1))
            x = center_x + amplitude * math.sin(2 * math.pi * t / period + phase_offset)
            set_pose(name, x)

        rx, ry = get_robot_position()
        if rx is not None and elapsed > warmup_seconds:
            for i, name in enumerate(names):
                phase_offset = i * (2 * math.pi / max(num_obstacles, 1))
                ox = center_x + amplitude * math.sin(2 * math.pi * t / period + phase_offset)
                dist = math.sqrt((rx - ox) ** 2 + (ry - 0.25) ** 2)
                if dist < 0.25:
                    collision_detected = True
                    print(f"  COLLISION RISK: robot at ({rx:.2f},{ry:.2f}), "
                          f"{name} at ({ox:.2f},0.25), dist={dist:.2f}m")

        time.sleep(max(dt, 0.05))

    print(f"  Level complete. Collision detected: {collision_detected}")
    return collision_detected


if __name__ == "__main__":
    levels = [
        (1, 1.0, 12),
        (2, 1.5, 12),
        (3, 2.0, 12),
        (4, 2.5, 12),
        (5, 3.0, 12),
        (6, 3.5, 12),
        (8, 4.0, 12),
    ]

    for num_obstacles, speed, duration in levels:
        failed = run_stress_level(num_obstacles, speed, duration)
        if failed:
            print(f"\n*** FAILURE POINT REACHED: {num_obstacles} obstacles at speed {speed} ***")
            break
    else:
        print("\nAll stress levels completed without detected collision.")
