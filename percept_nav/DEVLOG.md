# Percept-Nav Development Log

## Task 1 — Camera sensor (2026-07-18)
- Model: burger has no camera by default; found TurtleBot3 already ships a
  camera-equipped `burger_cam` model (SDF, not URDF — spawner reads model.sdf).
- Verified `/camera/image_raw` and `/camera/camera_info` publish real data.
- Issue: Gazebo Harmonic (OGRE-Next) gets no GPU accel through WSLg on this
  machine — falls back to software rendering (llvmpipe), ~3Hz instead of 30Hz.
- Fix: wrote a custom headless launch file (percept_nav_headless.launch.py)
  that drops the GUI client action — avoids the rendering bottleneck entirely
  since we don't need the live 3D view for development.

## Task 2 — OpenCV camera viewer node (2026-07-18)
- Confirmed OpenCV 4.6.0 + cv_bridge already installed, no install needed.
- Wrote camera_viewer_node.py: subscribes to /camera/image_raw, converts via
  cv_bridge, logs frame shape/timestamp, saves a sample frame to disk.
- Verified with 1700+ live frames processed. Frame size is 320x240 (wide-angle
  camera default), not 640x480 as originally assumed from the unused URDF edit.

## Task 3 — Obstacle detection, classical CV (2026-07-21, in progress)
- Approach: grayscale -> Gaussian blur -> Canny edge detection -> findContours
  -> filter by area -> bounding box. Deliberately classical CV, not deep
  learning (no training data needed, fast, fully explainable).
- Publishes annotated image on /camera/obstacle_detections for visual check.

## Task 3 — Obstacle detection debugging journey (2026-07-21)
Went through several iterations before landing on a working approach —
documenting the real path since it's the actual learning, not just the
final answer.

1. Edge detection (Canny) + contours, no crop: the wide-angle lens' black
   vignette border got detected as one giant edge, swallowing all real
   obstacles as "internal" to it (RETR_EXTERNAL only keeps outer contours).
2. Cropped out the vignette: fixed that, but now got 0 detections — real
   object edges are fragmented/broken, none formed a big enough closed
   contour to pass the area filter.
3. Added morphological dilation to close edge gaps: overcorrected, dilated
   edges from different objects touched and merged into one big blob again.
4. Reduced dilation amount: still one big blob — root cause wasn't dilation
   at all, it was the floor-to-wall horizon line acting as another
   full-width "fake edge," same failure mode as the vignette.
5. Switched techniques entirely: floor is fairly uniform/flat, so threshold
   segmentation (classify by brightness, not edges) fits this scene better.
   Tried Otsu (auto-picks a brightness cutoff) — it split the image on the
   wrong boundary (black gaps vs. everything else, not floor vs. obstacles),
   since it only knows "biggest brightness gap," not "which gap we want."
6. Pulled a real grayscale histogram and picked a fixed cutoff (128) at an
   actual empty bin in the data. Still failed — floor brightness isn't
   constant across the frame; it's brighter near the robot, darker toward
   the horizon (perspective + lighting falloff), so no single global cutoff
   works everywhere in the image at once.
7. Switched to adaptive thresholding (cv2.adaptiveThreshold, Gaussian,
   blockSize=41, C=10) — computes a local cutoff per region instead of one
   global number, following the brightness gradient instead of fighting it.
   This worked: 6 stable, separate obstacle detections across 500+ frames,
   all static objects (world has no moving obstacles yet -- that's Stage 3).

Key lesson: a technique failing isn't a coding mistake to patch with another
parameter — it's a signal to ask what specific property of the scene the
technique can't handle, then pick a technique that matches that property.

## Task 4 — Camera + LiDAR sensor fusion (2026-07-22)
- Added structured detection output: obstacle_detector_node now publishes
  vision_msgs/Detection2DArray (industry-standard message type) alongside
  the annotated debug image, instead of only drawing boxes on pixels.
- Wrote sensor_fusion_node.py: uses message_filters.ApproximateTimeSynchronizer
  to pair /scan (LiDAR) and /camera/detections_2d (camera) messages by
  timestamp, then projects each in-FOV LiDAR point to a pixel column via
  the pinhole camera model (focal length from FOV, angle->pixel formula),
  and matches it against detected bounding boxes. Full math derivation
  saved in docs/notes/sensor_fusion_math.md for reference/study.
- This is a genuinely different fusion problem than the thesis's IMU
  Kalman filter: that combined multiple estimates of the same quantity
  for accuracy; this combines two different sensor modalities (2D image +
  360 degree range) for completeness -- neither sensor alone gives both
  "what is it" and "how far is it."
- Verified working: stationary robot showed 6 detections, 5 consistently
  matched to a LiDAR range (~1.9-2.0m, sensible for the test world).
  Then drove the robot forward via teleop_keyboard and re-checked --
  distances to obstacles in the driving direction dropped to ~0.86-0.91m
  while off-axis obstacles stayed near their original distance, confirming
  the fusion tracks real, per-obstacle distance as the robot moves (not
  just repeating a static number).
- Known limitation: 1 of 6 detections typically goes unmatched each frame,
  likely at the edge of the camera's FOV where LiDAR's discrete angle
  steps don't land precisely on that detection's pixel range. Acceptable
  for a first working pipeline; the pinhole model is an approximation of
  the real fisheye lens (documented in sensor_fusion_math.md).

Stage 1 (Perception Foundation) complete: camera verified, OpenCV
obstacle detection working, camera+LiDAR fusion verified with real
robot motion.

## Task 5 — slam_toolbox setup (2026-07-22)
- Already installed on this machine; copied mapper_params_online_async.yaml
  into percept_nav/config/ rather than editing the system copy.
- Found and fixed a real config mismatch: default max_laser_range (20.0m)
  exceeded our actual LiDAR's range_max (3.5m, confirmed from /scan data
  in Task 4). Set to 3.5 explicitly.
- Hit a real gotcha: slam_toolbox's launch file argument is named
  `slam_params_file`, not `params_file` -- passing the wrong name silently
  falls back to the default config with no error, so our first "fix" wasn't
  actually being applied. Caught by checking `ros2 param get` on the live
  node instead of trusting the launch log alone.
- Verified frame names (base_footprint -> odom) match the config without
  changes needed.

## Task 6 — Live SLAM mapping (2026-07-22)
- Ran slam_toolbox live, generated a map from scratch (no pre-made map
  loaded), confirmed growing map dimensions via /map_metadata as the robot
  moved (80x102 -> 106x120 -> 141x120 -> 145x123 cells @ 5cm resolution).
- First driving attempt (manual teleop) produced a very sparse map -- thin
  traced lines, not filled regions. Root cause: driving traced a rough path
  rather than sweeping open floor area.
- Second attempt: wrote a scripted cmd_vel driver (timed forward/turn
  sequence) for reproducibility instead of manual keyboard driving.
  Result was still partial-coverage: several fan-shaped scan patterns
  visible in the saved map, indicating the robot got stuck against
  obstacles (world center has several cylinders) and rotated in place for
  parts of the sequence, rather than translating through open space.
- Real lesson: open-loop timed velocity commands don't verify whether
  movement actually succeeds -- this is exactly why real navigation stacks
  (Nav2, Stage 3) use closed-loop control with odometry/costmap feedback
  instead of blind timed commands.
- Accepted this as a partial-coverage map rather than re-attempting:
  confirms slam_toolbox is correctly configured and produces accurate,
  real-scale occupancy data (verified ~7m x 6m world scale, correct wall
  edges visible), but full clean-room coverage would need either more
  careful driving or closed-loop navigation to avoid stalling on obstacles.
- Saved final map to docs/images/task6_map_presentable.png -- rendered
  with matplotlib for a proper title, real-world axis scale in meters,
  and clean colormap (vs. the raw PGM's grayscale/tiny default).

## Task 7 — Save/reload map, compare against static reference (2026-07-22)
- Compared our SLAM map against the static reference map bundled with
  turtlebot3_manipulation_navigation2 (turtlebot3_world.yaml/.pgm). Same
  resolution (0.05m/cell); confirmed the actual world is a hexagonal room
  ~6m across with 9 obstacle cylinders in a 3x3 grid -- this explains the
  crisscrossing diagonal pattern in our earlier partial-coverage map (the
  robot was driving through narrow gaps between grid-arranged pillars).
- Real gotcha hit: first saved the map to /tmp, which does not persist --
  files were gone by the time we came back to test reloading. /tmp is for
  disposable data only. Fixed by saving to a permanent maps/ folder inside
  the repo instead, and moved the coverage-driving script from /tmp into
  scripts/ for the same reason -- anything needed again belongs in the
  repo, not /tmp.
- Verified full save/reload cycle: map_saver_cli wrote a real .pgm + .yaml
  pair to maps/task7_slam_map, then nav2_map_server successfully loaded it
  back (lifecycle configure + activate both succeeded) and republished it
  on /map with matching resolution/dimensions/origin -- confirms the saved
  map is a genuinely reusable artifact, not just a static image.

## Task 8 — SLAM parameter tuning and documentation (2026-07-22)
- Documented all key mapper_params_online_async.yaml parameters in
  docs/notes/slam_toolbox_params.md: max_laser_range (changed, matched to
  real sensor), minimum_travel_distance/heading (attempted change, tuned
  for our small ~6m world), loop closure group and scan-matching
  correlation params (read/understood, left at default with reasoning).
- Attempted a live before/after comparison for minimum_travel_distance.
  Could not complete cleanly: after a long session of repeated Gazebo/SLAM
  launches and restarts, /scan degraded to ~0.5Hz with a 61-second max
  gap between scans, alongside persistent "jump back in time" TF buffer
  warnings. Diagnosed as environment resource exhaustion (same WSLg
  rendering bottleneck from Task 1, compounding after extended runtime),
  not a config or parameter problem. Documented this honestly rather than
  reporting fabricated results.
- Real process lesson: fair before/after parameter comparisons need a
  clean, stable test environment. Recognizing "this is infrastructure,
  not tuning" and stopping to document it clearly, rather than chasing an
  unstable result, is itself a legitimate diagnostic skill.

Stage 2 (SLAM Integration) complete: slam_toolbox installed and correctly
configured, live mapping demonstrated and verified end-to-end (save +
reload cycle working), key parameters documented with real reasoning.

## Task 9 — Custom Nav2 costmap layer (2026-08-01)
- Created a new C++ package (percept_nav_costmap_plugin, ament_cmake build
  type) separate from the main percept_nav Python package -- Nav2 costmap
  layers are C++ plugins loaded via pluginlib, a different toolchain than
  everything built so far in this project.
- Implemented DetectionLayer, inheriting from nav2_costmap_2d::CostmapLayer,
  with the standard Layer interface: onInitialize() (subscribes to
  /detected_obstacles from the Task 4 fusion node), updateBounds() and
  updateCosts() (called every costmap cycle by Nav2), and reset().
- Real bug hit on first build: referenced robot_x_/robot_y_ as class
  members inside updateBounds()/updateCosts() without ever declaring or
  populating them -- classic copy-paste-style naming mismatch. Compiler
  caught it immediately (undeclared identifier). Fixed by adding the
  members to the header and actually capturing the real robot_x/robot_y
  parameters Nav2 passes into updateBounds() each cycle.
- Honest scoping limitation, documented rather than hidden: the Task 4
  fusion node stores a fused distance (pose.position.x) per detection but
  does not yet compute each detection's true map-frame (x, y) position --
  that would require combining distance with robot heading and the
  detection's specific camera angle. For this first working version, the
  layer marks a fixed-radius region around the robot's current position
  whenever any valid fused detection exists, rather than placing each
  detection at its precise real-world location. A complete implementation
  is a natural follow-up, not attempted this pass to keep Task 9 scoped
  to "wire the pipeline into Nav2 end-to-end" rather than perfect geometry.
- Build succeeded on the second attempt (after the robot_x_/robot_y_ fix).
  Verified the plugin actually installs correctly: compiled .so library,
  installed plugin XML descriptor, and critically the
  nav2_costmap_2d__pluginlib__plugin resource index entry that Nav2's
  plugin loader scans to discover available Layer plugins by name --
  confirms this is genuinely discoverable, not just compiled.
- Integration test (actually running Nav2 with this layer active in its
  costmap config) is planned for Task 11.

## Task 10 — Moving obstacle in Gazebo (2026-08-01)
- Built moving_box.sdf: a simple 0.3m red box with the gz-sim
  VelocityControl system plugin attached, spawned live into the running
  world via `gz service .../create` (data: true confirmed).
- Attempted physics-based velocity control (the standard approach, per
  Gazebo's own bundled velocity_control.sdf demo world). Extensive
  troubleshooting: confirmed a real subscriber existed on the model's
  cmd_vel topic, confirmed messages were reaching Gazebo (direct gz topic
  publish, repeated publish loop, and a proper ROS2-bridged sustained
  publisher via rclpy -- the same reliable pattern used successfully in
  Task 6's drive_coverage.py). None produced real, controlled motion --
  position drifted by only a few centimeters, inconsistent with commanded
  direction. Also separately diagnosed and fixed a real environment issue
  along the way: after ~13 hours of continuous uptime, Gazebo's
  real_time_factor had collapsed to ~0.003 (over 300x slower than real
  time) -- a full process restart fixed that specific problem, but did
  not fix the VelocityControl issue, confirming they were two separate
  problems, not one.
- Root cause of the VelocityControl non-response was not conclusively
  identified despite methodical elimination of the more likely causes
  (message delivery, subscriber existence, sim speed). Rather than
  continue debugging an increasingly obscure plugin-internals question,
  pivoted to a simpler, more reliable approach.
- Final approach: direct pose-teleportation via the /world/default/set_pose
  service, called on a timer with a sine-wave oscillation (script:
  scripts/move_obstacle_box.py). Verified working -- box moved a real,
  substantial distance (0.92m -> 1.47m) over 2 seconds, matching the
  commanded oscillation. This satisfies the task's actual requirement
  ("simple actors or moving boxes") without depending on a physics
  subsystem that would not cooperate in this environment.
- Real lesson: knowing when to stop debugging a stubborn, poorly-understood
  tool behavior and switch to a simpler working approach is itself a
  legitimate engineering skill -- not every problem is worth fully
  resolving before moving forward, especially when a reliable alternative
  exists that meets the actual requirement.

## Task 11 — Verify Nav2 replanning around moving obstacle (2026-08-01)
- Configured Nav2 with a custom nav2_params.yaml (base: turtlebot3_navigation2
  burger_cam.yaml), adding detection_layer to the local_costmap plugins list
  alongside obstacle_layer, voxel_layer, inflation_layer.
- Used nav2_bringup's navigation_launch.py (not turtlebot3_navigation2's
  wrapper, which defaults to a static map + AMCL + unconditional RViz --
  incompatible with our already-running slam_toolbox and headless setup).
- Confirmed our Task 9 plugin loads correctly in a live Nav2 stack: log
  showed "Using plugin detection_layer" / "DetectionLayer initialized,
  subscribing to /detected_obstacles" / "Initialized plugin detection_layer"
  alongside the standard costmap layers. Full nav2 stack (controller,
  planner, behavior_server, bt_navigator, etc.) configured and activated
  cleanly via the lifecycle manager.
- Sent a real NavigateToPose goal; confirmed genuine navigation via
  steadily decreasing distance_remaining in the action feedback (0.90 ->
  0.86 -> 0.85 -> 0.83 -> 0.75m) and real robot position updates matching.
- Tested obstacle response: moved moving_obstacle_box (from Task 10)
  directly into the robot's path mid-navigation via set_pose. Checked
  /local_costmap/costmap immediately after -- showed near-lethal cost
  values (95-100, out of 100 max) at cells corresponding to the obstacle's
  new position, confirming real-time costmap awareness of the moved object.
- Confirmed the planner remains active throughout: /plan topic showed a
  live 6-waypoint path while the navigation goal continued running in the
  background.
- Honest limitation carried over from Task 9: the custom detection_layer
  marks a fixed-radius region around the robot's own position for any
  valid fused detection, rather than the detection's precise mapped
  location -- so the near-lethal costmap values observed here are most
  directly attributable to the standard obstacle_layer/voxel_layer (which
  read raw LiDAR data and would detect the box regardless), with
  detection_layer contributing additional marking whenever our fusion
  pipeline has a valid detection. Precise per-detection placement in
  detection_layer remains a documented follow-up, not this task's scope.

## Task 12 — Stress test: obstacle density and speed (2026-08-01)
- Wrote scripts/stress_test.py: spawns N moving obstacles (reusing
  moving_box.sdf), oscillates them at a configurable speed factor via
  pose-teleport (same reliable approach as Task 10), and monitors robot
  position vs. each obstacle's position for close-proximity events while
  a real NavigateToPose goal runs concurrently.
- First run flagged a false positive: collision detected immediately at
  the very first, easiest level (1 obstacle, speed 1.0). Investigated
  before accepting the result -- confirmed via gz model and /odom that
  the robot was actually well clear of the obstacle by the time checked;
  the flag fired because the check started immediately at spawn time,
  before Nav2 had any real chance to perceive and react to the obstacle.
  Fixed by adding a 3-second warmup period before collision checks begin.
- Re-ran the corrected test across escalating levels: 1 obstacle at 1.0x
  speed up through 8 obstacles at 4.0x oscillation speed (12 seconds per
  level, real NavigateToPose goal running concurrently throughout).
  No collision or navigation failure detected at any level.
- One script-level artifact at the highest level: a duplicate entity name
  ("stress_box_2" spawn FAILED) because the test never cleaned up boxes
  between levels, so a name from an earlier level was already in use by
  the time level 7 ran. This is a test-harness bug, not a Nav2 or
  simulation failure, and is documented as such rather than reported as
  a system limit.
- Honest result: no genuine failure point was found within the tested
  range (up to 8 simultaneous moving obstacles, 4x baseline oscillation
  speed). This is real, positive robustness data -- the navigation +
  custom costmap pipeline held up under meaningfully harder conditions
  than the single-slow-obstacle case verified in Task 11 -- but it is not
  the dramatic "breaking point" the task's phrasing implies. A true
  failure point (if one exists at reasonable stress levels) would need
  either a fixed test harness pushed further, or a fundamentally
  different stressor (e.g. obstacles directly blocking the only viable
  path, rather than oscillating through open space).
- This result is itself useful benchmark data for Stage 4: establishes a
  documented lower bound ("handles up to 8 obstacles at 4x speed") rather
  than an assumed or untested capability claim.

## Task 14 — Instrumented benchmark trial runner (in progress, 2026-08-01)
- Wrote scripts/benchmark_trials.py: an rclpy-based trial runner executing
  Task 13's protocol (4 conditions, using the NavigateToPose action client
  directly rather than the ros2 CLI, subscribing to /odom, /plan for
  path length, and tracking obstacle proximity for collision counting).
- Multiple real bugs found and fixed during smoke testing (1-trial runs),
  each confirmed with direct evidence before being accepted as fixed:
  1. Initial success check only tested whether the action future resolved
     (`.done()`), not the actual result status -- an aborted/rejected goal
     resolving quickly was wrongly logged as success. Fixed by checking
     the real GoalStatus (STATUS_SUCCEEDED == 4).
  2. Attempted robot position reset via gz service set_pose (teleport).
     This moves the robot in the physics engine but does NOT reset /odom
     (wheel-encoder dead-reckoning has no way to know a teleport happened),
     so Nav2's internal position tracking silently desynced from the
     robot's real simulated position -- trials "succeeded" instantly
     because Nav2 still thought it was near the goal from a prior test.
     Fixed by replacing teleport with a real NavigateToPose call back to
     the start pose between trials, keeping odometry consistent.
  3. Chosen start pose (-2.0, -0.5) -- taken from the launch file's spawn
     argument -- turned out to be outside the actual map/costmap bounds
     once SLAM/localization was running; Nav2's planner rejected it every
     time ("Goal Coordinates ... was outside bounds"), so the "return to
     start" step silently failed and the robot never actually moved,
     again producing false-instant "successes". Root-caused by reading
     the actual Nav2 bt_navigator log line showing the robot's true
     map-frame starting position was (0.00, -0.00), not the launch spawn
     argument -- corrected START_X/START_Y to (0.0, 0.0).
- After all three fixes, hit a new (likely final, and much more mundane)
  issue: the 30-second trial timeout is too short for a genuine two-hop
  round trip (return-to-start, then the real trial navigation) under this
  environment's real-time-factor variability. Not yet fixed -- next
  session should increase TRIAL_TIMEOUT (e.g. to 60s) and re-run the
  1-trial-per-condition smoke test before scaling to the full 40 trials.
- Deliberately stopped here rather than continue further live debugging:
  this was already a long, productive diagnostic session (3 real bugs
  found and fixed with concrete evidence each time), and pushing further
  while fatigued risked introducing new mistakes rather than catching
  them. Stopping at a clearly identified, well-understood next step is
  itself the right call, not a failure to finish.
