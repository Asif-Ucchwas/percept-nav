import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path, Odometry
import subprocess
import time
import math
import csv
import sys

SDF_PATH = "/home/jannatul/ros2_ws/src/percept_nav_repo/percept_nav/worlds/moving_box.sdf"
GOAL_X, GOAL_Y = 1.8, 0.3
START_X, START_Y = 0.0, 0.0  # true map-frame origin, confirmed from Nav2 logs (launch spawn param does not map directly to map-frame coords)
TRIAL_TIMEOUT = 60.0
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


class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('benchmark_trials')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.latest_odom = None
        self.latest_plan_len = None
        self.latest_plan_time = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)

    def _odom_cb(self, msg):
        self.latest_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _plan_cb(self, msg):
        poses = msg.poses
        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            length += math.sqrt(dx * dx + dy * dy)
        self.latest_plan_len = length
        self.latest_plan_time = time.time()

    def run_trial(self, obstacle_names, obstacle_positions):
        self._action_client.wait_for_server(timeout_sec=5.0)
        self.latest_plan_len = None  # reset so we don't carry over a stale plan from a previous trial

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = GOAL_X
        goal_msg.pose.pose.position.y = GOAL_Y
        goal_msg.pose.pose.orientation.w = 1.0

        result_holder = {'done': False, 'success': False, 'nav_time': None}

        def feedback_cb(feedback_msg):
            pass

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {'success': False, 'time_to_goal': None, 'path_length': None, 'collisions': 0}

        result_future = goal_handle.get_result_async()

        start_time = time.time()
        collisions = 0
        checked_positions = set()

        while time.time() - start_time < TRIAL_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.latest_odom and obstacle_positions:
                rx, ry = self.latest_odom
                for name, (ox, oy) in zip(obstacle_names, obstacle_positions):
                    dist = math.sqrt((rx - ox) ** 2 + (ry - oy) ** 2)
                    if dist < COLLISION_THRESHOLD:
                        collisions += 1

            if result_future.done():
                break

        elapsed = time.time() - start_time
        goal_status = None
        if result_future.done():
            result = result_future.result()
            if result is not None:
                goal_status = result.status
        # GoalStatus.STATUS_SUCCEEDED == 4 (action_msgs/msg/GoalStatus)
        success = (goal_status == 4) and elapsed < TRIAL_TIMEOUT
        path_length = self.latest_plan_len

        if not success:
            print(f"    DEBUG: goal_status={goal_status}, future_done={result_future.done()}, elapsed={elapsed:.2f}")

        return {
            'success': success,
            'time_to_goal': elapsed if success else None,
            'path_length': path_length,
            'collisions': collisions
        }

    def navigate_to(self, x, y, timeout=45.0):
        self._action_client.wait_for_server(timeout_sec=5.0)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                return True
        return False


def run_condition(node, condition_name, num_obstacles, speed, num_trials, results_writer):
    print(f"\n=== Condition: {condition_name} ({num_obstacles} obstacles, speed={speed}) ===")

    for trial in range(1, num_trials + 1):
        print(f"  Returning to start before trial {trial}...")
        node.navigate_to(START_X, START_Y)
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

        result = node.run_trial(obstacle_names, obstacle_positions)
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
    rclpy.init()
    node = BenchmarkNode()

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
            run_condition(node, name, num_obs, speed, trials, writer)
            f.flush()

    print(f"\nAll trials complete. Results saved to {output_path}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
