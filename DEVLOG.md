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
