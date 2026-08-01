# Task 4 — Camera/LiDAR Fusion: Math, By Hand, Then Code

Goal: match a LiDAR point (angle + distance) to a camera pixel, so we
know "an obstacle exists at exact distance X" (LiDAR) AND "here's what
it looks like" (camera's detected bounding box).

---

## Concept 1 — Focal length from field of view

### The formula
    focal_length_px = (image_width / 2) / tan(FOV / 2)

### By hand
Camera specs: FOV = 1.02974 rad (~59°), image width = 320px.

    focal_length_px = (320 / 2) / tan(1.02974 / 2)
                     = 160 / tan(0.51487)
                     = 160 / 0.5673
                     ≈ 282.1

Meaning: conversion factor between "angle off-center" and "pixels
off-center." Bigger focal length = narrower/zoomed view.

### In code
    import math
    focal_length_px = (image_width / 2) / math.tan(fov_rad / 2)

---

## Concept 2 — LiDAR angle -> pixel column

### The formula
    pixel_x = (image_width / 2) + focal_length_px * tan(theta)

### By hand
theta = 0.2 rad (~11.5° right of center):

    pixel_x = 160 + 282.1 * tan(0.2)
            = 160 + 282.1 * 0.2027
            ≈ 217

### In code
    pixel_x = (image_width / 2) + focal_length_px * math.tan(theta)

---

## Concept 3 — Filter LiDAR to camera's FOV only

### The formula
    keep point only if:  -FOV/2 <= theta <= +FOV/2

### By hand
FOV/2 = 1.02974 / 2 = 0.51487 rad. So keep only LiDAR angles between
-0.51487 and +0.51487 radians (~-29.5° to +29.5°). Everything outside
is beside/behind the robot -- invisible to the camera.

### In code
    half_fov = fov_rad / 2
    if -half_fov <= theta <= half_fov:
        # this LiDAR point is inside the camera's view
        ...

---

## Concept 4 — Adjust for image crop

### The formula
    cropped_pixel_x = pixel_x - crop_x

### By hand
crop_x = 64 (our obstacle detector crops a 192px window starting at
x=64 in the raw 320px frame, to remove the lens' black vignette).

If pixel_x = 217:
    cropped_pixel_x = 217 - 64 = 153

Then check: does cropped_pixel_x = 153 fall inside any detected
bounding box's [x, x+w] range? If yes, that LiDAR reading (distance +
angle) is the confirmed real-world distance for that visual detection.

### In code
    cropped_pixel_x = pixel_x - crop_x
    for (box_x, box_y, box_w, box_h) in detected_boxes:
        if box_x <= cropped_pixel_x <= box_x + box_w:
            # match found -- fuse this LiDAR range with this box
            ...

---

## Simplification acknowledged
This uses the pinhole camera model -- treats the lens as ideal/simple.
Our actual sensor is a "wideanglecamera" with real fisheye distortion
(visible as the curved vignette in raw frames), so this projection is
an approximation -- most accurate near image center, less accurate
near edges. Good enough for a first working pipeline; a production
system would use the lens' real distortion coefficients (available in
/camera/camera_info) to correct this properly.
