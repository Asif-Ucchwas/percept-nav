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
