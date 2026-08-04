import subprocess
import time
import math
import csv

SDF_PATH = "/home/jannatul/ros2_ws/src/percept_nav_repo/percept_nav/worlds/moving_box.sdf"
GOAL_X, GOAL_Y = 1.8, 0.3
START_X, START_Y = 0.0, 0.0
TRIAL_TIMEOUT = 60
COLLISION_THRESHOLD = 0.25


def gz_spawn(name, x, y):
    req = f'sdf_filename: "{SDF_PATH}", name: "{name}", pose: {{position: {{x: {x}, y: {y}, z: 0.15}}}}'
    result = subprocess.run(
        ["gz", "service", "-s", "/world/default/create", "--reqtype", "gz.msgs.EntityFactory",
         "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
        capture_output=True, text=True
    )
    return "true" in result.stdout.lower()


def gz_remove(name):
    subprocess.run(
        ["gz", "service", "-s", "/world/default/remove", "--reqtype", "gz.msgs.Entity",
         "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", f'name: "{name}", type: MODEL'],
        capture_output=True
    )


def gz_set_pose(name, x, y=0.25, z=0.15):
    subprocess.run(
        ["gz", "service", "-s", "/world/default/set_pose", "--reqtype", "gz.msgs.Pose",
         "--reptype", "gz.msgs.Boolean", "--timeout", "1000",
         "--req", f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'],
        capture_output=True
    )


def get_robot_position():
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "/odom", "--once"],
            capture_output=True, text=True, timeout=5
        )
    except subprocess.TimeoutExpired:
        return None, None
    lines = result.stdout.splitlines()
    for i, line in enumerate(lines):
        if line.strip() == "position:":
            x = float(lines[i + 1].split(":")[1])
            y = float(lines[i + 2].split(":")[1])
            return x, y
    return None, None


def get_plan_length():
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "/plan", "--once"],
            capture_output=True, text=True, timeout=5
        )
    except subprocess.TimeoutExpired:
        # /plan stops publishing once navigation is idle (no active goal) --
        # not an error, just means there's no fresh plan to measure right now.
        return None

    xs, ys = [], []
    lines = result.stdout.splitlines()
    for i, line in enumerate(lines):
        if line.strip() == "position:":
            xs.append(float(lines[i + 1].split(":")[1]))
            ys.append(float(lines[i + 2].split(":")[1]))
    length = 0.0
    for i in range(1, len(xs)):
        length += math.sqrt((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2)
    return length if xs else None


def send_goal_and_wait(x, y, timeout):
    cmd = [
        "ros2", "action", "send_goal", "/navigate_to_pose", "nav2_msgs/action/NavigateToPose",
        f"{{pose: {{header: {{frame_id: 'map'}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{w: 1.0}}}}}}}}"
    ]
    start = time.time()
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        elapsed = time.time() - start
        succeeded = "status: SUCCEEDED" in result.stdout
        return succeeded, elapsed
    except subprocess.TimeoutExpired:
        return False, timeout


def run_trial(obstacle_names, obstacle_positions):
    collisions = 0
    success, elapsed = send_goal_and_wait(GOAL_X, GOAL_Y, TRIAL_TIMEOUT)

    if obstacle_positions:
        rx, ry = get_robot_position()
        if rx is not None:
            for (ox, oy) in obstacle_positions:
                dist = math.sqrt((rx - ox) ** 2 + (ry - oy) ** 2)
                if dist < COLLISION_THRESHOLD:
                    collisions += 1

    path_length = get_plan_length()

    return {
        'success': success,
        'time_to_goal': elapsed if success else None,
        'path_length': path_length,
        'collisions': collisions
    }


def run_condition(condition_name, num_obstacles, speed, num_trials, results_writer):
    print(f"\n=== Condition: {condition_name} ({num_obstacles} obstacles, speed={speed}) ===")

    for trial in range(1, num_trials + 1):
        print(f"  Returning to start before trial {trial}...")
        send_goal_and_wait(START_X, START_Y, 45)

        obstacle_names = [f"bench_box_{i}" for i in range(num_obstacles)]
        obstacle_positions = []
        for i, name in enumerate(obstacle_names):
            x = 0.7 + i * 0.4
            gz_spawn(name, x, 0.25)
            obstacle_positions.append((x, 0.25))
            time.sleep(0.3)

        if speed > 0:
            start_move = time.time()
            move_duration = 2.0
            while time.time() - start_move < move_duration:
                t = (time.time() - start_move) * speed
                for i, name in enumerate(obstacle_names):
                    x = 0.7 + i * 0.4 + 0.3 * math.sin(t)
                    gz_set_pose(name, x)
                time.sleep(0.1)

        result = run_trial(obstacle_names, obstacle_positions)
        status = "OK" if result['success'] else "FAIL"
        print(f"  Trial {trial}/{num_trials}: {status}, "
              f"time={result['time_to_goal']}, path_len={result['path_length']}, "
              f"collisions={result['collisions']}")

        results_writer.writerow({
            'condition': condition_name,
            'trial': trial,
            'success': result['success'],
            'time_to_goal': result['time_to_goal'],
            'path_length': result['path_length'],
            'collisions': result['collisions']
        })

        for name in obstacle_names:
            gz_remove(name)
        time.sleep(1.0)


def main():
    output_path = "/home/jannatul/ros2_ws/src/percept_nav_repo/percept_nav/benchmark_results.csv"
    with open(output_path, 'w', newline='') as f:
        fieldnames = ['condition', 'trial', 'success', 'time_to_goal', 'path_length', 'collisions']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        conditions = [
            ("baseline", 0, 0, 1),
            ("static_obstacle", 1, 0, 1),
            ("single_moving", 1, 1.0, 1),
            ("multiple_moving", 3, 2.0, 1),
        ]

        for name, num_obs, speed, trials in conditions:
            run_condition(name, num_obs, speed, trials, writer)
            f.flush()

    print(f"\nAll trials complete. Results saved to {output_path}")


if __name__ == '__main__':
    main()
