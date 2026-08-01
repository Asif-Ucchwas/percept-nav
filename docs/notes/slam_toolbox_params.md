# Task 8 — SLAM Toolbox Parameters: What They Do and What We Learned

This documents the key mapper_params_online_async.yaml parameters, based
on real behavior observed while running slam_toolbox in this project
(not just reading the docs).

---

## Parameters we changed, and why

### max_laser_range
Default: 20.0m. Changed to: 3.5m.

What it does: tells SLAM the maximum distance to trust laser readings for
building the map ("for rastering images", per the config's own comment).

Why we changed it: our actual LiDAR's range_max (confirmed by reading raw
/scan data in Task 4) is 3.5m -- the default assumed a much longer-range
sensor. Left at 20.0, slam_toolbox printed a warning every launch:
"maximum laser range setting (20.0 m) exceeds the capabilities of the
used Lidar (3.5 m)". Not a fatal error (it clips internally), but leaving
a known mismatch undocumented is bad practice -- matching config to your
actual hardware specs is a basic but easy-to-skip step.

### minimum_travel_distance / minimum_travel_heading
Default: 0.5m / 0.5 rad. Attempted change: 0.1m / 0.1 rad.

What it does: SLAM only processes a new scan after the robot has moved at
least this far (distance) or turned this much (heading) since the last
processed scan. This is a performance optimization -- it avoids wasting
compute reprocessing near-identical consecutive scans.

Why we wanted to change it: our world is only ~6m across (confirmed by
comparing against the bundled static map). A 0.5m minimum is a large
fraction of the whole space, likely too coarse for dense map coverage in
such a small environment -- lowering it should let SLAM register more
frequent updates as the robot moves through a small area.

Result: attempted a live before/after comparison but could not complete
it -- Gazebo's simulated clock became severely unstable after a long
session (repeated launches/restarts), with /scan dropping to ~0.5Hz and
eventually a 61-second gap between scans, alongside repeated "jump back
in time" TF warnings. This is an environment/resource limitation (the
same WSLg rendering bottleneck documented in Task 1's DEVLOG entry,
compounding after extended runtime), not a parameter or config issue.
Documenting this honestly rather than reporting fabricated results: the
theoretical reasoning above is sound and is what a real tuning pass would
test, but we do not have clean before/after map data to show for it in
this session.

---

## Parameters we read and understood, but left at default (sound reasoning why)

### do_loop_closing (true), loop_search_maximum_distance (3.0m),
### loop_match_minimum_chain_size (10), loop_match_minimum_response_coarse/fine (0.35/0.45)
What they do: loop closure is how SLAM recognizes "I've been here before"
and corrects accumulated drift. loop_search_maximum_distance limits how
far away (in the current map estimate) SLAM will look for a potential
match. loop_match_minimum_chain_size requires a minimum run of consistent
matching scans before accepting a loop closure (avoids false positives
from one lucky match). The response thresholds (coarse then fine) are a
two-stage filter -- a fast, loose check first, then a slower, stricter
check only on promising candidates.

Why left at default: our test drives never completed a genuine loop
(driving out and returning to the same spot) due to the coverage issues
documented in Task 6 -- so loop closure was never meaningfully exercised
in this project yet. Reasonable to revisit if/when a full-coverage drive
is achieved.

### correlation_search_space_dimension (0.5m) / resolution (0.01m) /
### smear_deviation (0.1)
What they do: controls the search area and precision when scan-matching
a new scan against the existing map -- how far around the estimated pose
SLAM searches, and how finely, to find the best alignment. Smear
deviation blurs the search slightly to make matching more tolerant of
small odometry errors.

Why left at default: these are reasonable, well-tested defaults for
indoor mapping at this scale; no specific evidence in our testing pointed
to needing a change here.

---

## Key lesson for this task
Real parameter tuning requires clean, repeatable test conditions --
confirmed the hard way today. When the environment itself becomes
unstable (resource exhaustion, clock drift), no amount of "correct"
parameter values will produce a fair before/after comparison. Recognizing
"this is an infrastructure problem, not a tuning problem" and stopping to
document that clearly is itself a real diagnostic skill, not a failure to
report.
