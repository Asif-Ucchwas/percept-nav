# Task 13 — Benchmark Test Protocol

Mirrors the rigor of the thesis's 50-trial methodology, scoped honestly
to this project's resource constraints (see note on trial count below).

## Fixed test parameters
- Start pose: robot spawn default (-2.0, -0.5, 0.0) per percept_nav_headless.launch.py
- Goal pose: (1.8, 0.3, 0.0) -- same goal used in Task 11/12 verification,
  a real distance requiring the robot to cross the open middle area of
  the hexagonal test world
- World: turtlebot3_world (hexagonal room, 9-pillar 3x3 obstacle grid)
- Nav2 config: config/nav2_params.yaml (includes custom detection_layer)
- use_sim_time: true throughout

## Test conditions
1. **Baseline** -- no additional obstacles. Establishes ideal path
   length/time with only the static 9-pillar grid present.
2. **Static obstacle** -- one moving_obstacle_box spawned directly in the
   direct path between start and goal, held stationary (speed=0).
3. **Single moving obstacle** -- one moving_obstacle_box oscillating at
   the baseline speed verified working in Task 11 (period=4.0s,
   amplitude=0.6m).
4. **Multiple moving obstacles** -- 3 moving_obstacle_boxes oscillating,
   speed factor 2.0 (a known-stable level from Task 12's stress test,
   not pushed to the edge of failure).

## Trials
10 trials per condition (40 total). Thesis methodology used 50 trials;
scoped down here due to real, documented resource constraints in this
project's environment (WSLg/Gazebo rendering degradation after extended
runtime, confirmed multiple times in Tasks 8, 10, and 12's DEVLOG
entries). Noted as an explicit, honest limitation -- a full 50-trial
run is listed as future work in the eventual paper draft, not hidden.

## Metrics captured per trial
- **Success** (bool) -- did the robot reach the goal pose (within a
  reasonable tolerance) without collision, within a timeout?
- **Time to goal** (seconds) -- navigation_time from the action feedback
  at successful completion
- **Path length** (meters) -- sum of consecutive waypoint distances in
  the final accepted /plan
- **Collision count** (int) -- number of times robot-obstacle distance
  fell below a safety threshold (0.25m, same threshold validated in
  Task 12) during the trial
- **Replanning latency** (seconds, conditions 2-4 only) -- time between
  an obstacle first appearing in the local costmap at lethal cost and
  the /plan topic publishing a new path

## Derived comparisons (for the report/paper)
- Success rate per condition
- Path length overhead: (condition path length / baseline path length) - 1
- Mean replanning latency per condition (2-4)
- Collision rate per condition

## Known limitations, stated upfront
- 10 trials, not 50 -- documented resource constraint, not hidden
- Single world (hexagonal test world) -- not tested across multiple
  environment geometries
- Obstacle positions along a single line of oscillation, not varied
  approach angles -- a real methodological narrowing worth naming
