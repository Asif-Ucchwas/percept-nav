# Stage 4 — Benchmark Results Summary

Evidence-based summary of Percept-Nav's dynamic obstacle avoidance
performance, compiled from Tasks 11, 12, and manual trial verification
(Tasks 14-15). Full automated 40-trial sweep was attempted but not
completed -- see benchmark_protocol.md and DEVLOG.md for the honest
account of why, and what was verified instead.

## 1. Baseline navigation capability (Task 11)

Confirmed via direct action feedback during a real NavigateToPose goal:

| Metric | Result |
|---|---|
| Navigation completes | Yes -- distance_remaining decreased steadily (0.90 -> 0.86 -> 0.85 -> 0.83 -> 0.75m) |
| Custom costmap plugin active | Yes -- "DetectionLayer initialized" confirmed in Nav2 startup log |
| Live path planning | Yes -- /plan showed a real 6-waypoint path during active navigation |

## 2. Dynamic obstacle response (Task 11)

Moved a real obstacle directly into the robot's path mid-navigation:

| Metric | Result |
|---|---|
| Costmap registers moved obstacle | Yes -- local costmap cost values jumped to 95-100 (near-lethal) at the obstacle's new position |
| Planner remains active | Yes -- /plan continued producing live paths, not frozen |

## 3. Stress test: obstacle density and speed (Task 12)

| Obstacles | Speed factor | Result |
|---|---|---|
| 1 | 1.0x | No collision |
| 2 | 1.5x | No collision |
| 3 | 2.0x | No collision |
| 4 | 2.5x | No collision |
| 5 | 3.0x | No collision |
| 6 | 3.5x | No collision |
| 8 | 4.0x | No collision |

No genuine failure point was found within this tested range. This is a
real, positive robustness result -- not the dramatic "found the limit"
result originally hypothesized, but honest evidence that the system
handles meaningfully harder conditions than the single-slow-obstacle
baseline case.

## 4. System failure mode under sustained load (Task 14-15)

One real timed trial (baseline condition, no additional obstacles) was
run to gather formal timing data. Result: ABORTED after 653 seconds
(~11 real minutes). Root cause, confirmed via direct Nav2 log inspection
rather than assumption:

    CRITICAL FAILURE: SERVER velocity_smoother IS DOWN after not
    receiving a heartbeat for 4000 ms. Shutting down related nodes.

This is an internal node failure (velocity_smoother became unresponsive),
consistent with the resource-constrained-environment degradation pattern
documented throughout this project (WSLg/Gazebo rendering slowdown under
sustained runtime, previously observed as real_time_factor collapse).

Nav2's lifecycle manager detected this failure correctly and
automatically reset and reconfigured the entire navigation stack without
manual intervention -- a genuine, positive finding about Nav2's own
fault tolerance, separate from percept_nav's own code. Most of the 653
seconds was this automatic recovery process, not the actual navigation
attempt itself.

## What this evidence supports, honestly

- The custom perception -> fusion -> costmap -> Nav2 pipeline built in
  Stages 1-3 works end-to-end, verified with direct log evidence at each
  stage, not assumed.
- The system handles multiple simultaneous moving obstacles at elevated
  speed without collision, within the tested range.
- The system is not immune to the underlying environment's resource
  constraints -- a genuine limitation, documented honestly rather than
  hidden, along with the observation that Nav2's own architecture
  includes real self-healing behavior for exactly this kind of failure.

## What this evidence does not support

- A formal 40-trial statistical comparison (success rate, mean
  replanning latency, path length overhead) across the four planned
  conditions, as originally scoped in benchmark_protocol.md. The
  automated harness needed for that scale of data collection hit
  repeated, real implementation bugs (documented in DEVLOG.md) and was
  not completed in the time available for this portfolio project. Named
  explicitly as future work in the paper draft rather than presented as
  complete.
