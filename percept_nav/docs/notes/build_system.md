# DevOps-Rigor Stage 2 — Build System: What We Learned

This documents the colcon/ament build tooling for this workspace (two
packages: `percept_nav`, ament_python; `percept_nav_costmap_plugin`,
ament_cmake), based on real failures hit and fixed during a dependency
audit and clean-build verification (not just reading the docs).

---

## What each tool resolves

### colcon
The build orchestrator for this ROS2 workspace. Discovers packages via
`package.xml`, resolves inter-package build order, and calls the
correct underlying build system per package (`ament_python` for
`percept_nav`, `ament_cmake`/CMake for `percept_nav_costmap_plugin`).

### rosdep
Resolves each package's declared `<depend>`/`<exec_depend>`/
`<test_depend>` entries in `package.xml` against actual system/apt
packages. `rosdep check --from-paths src --ignore-src` is non-destructive
and reports missing dependencies without installing anything - the right
first command to run on an unfamiliar or freshly cloned workspace.

### package.xml vs CMakeLists.txt
`package.xml` is the source of truth for *what* a package depends on;
`CMakeLists.txt`'s `find_package()`/`ament_target_dependencies()` calls
are the source of truth for *how* those dependencies are actually linked
into build targets. They need to agree - `find_package()` can succeed
using a dependency that's silently present on your machine for unrelated
reasons, even if `package.xml` never declared it. Only `package.xml`'s
declarations are checked by `rosdep`/colcon dependency resolution and
determine whether a stranger's fresh clone builds cleanly.

---

## Real bugs found and fixed

### Undeclared dependencies in package.xml (both packages)

What we found: cross-referencing every `#include`/`import` statement
against each package's declared `<depend>` list surfaced real gaps.

`percept_nav` (Python) was missing `cv_bridge`, `geometry_msgs`,
`nav2_msgs`, `nav_msgs` as `<depend>`, and `launch`, `launch_ros`,
`ament_index_python` as `<exec_depend>` - all used directly in nodes or
launch files, none declared.

`percept_nav_costmap_plugin` (C++) was missing `ament_cmake_gtest`,
`rclcpp_lifecycle`, `tf2_ros` as `<test_depend>` - all three required by
`CMakeLists.txt`'s `ament_add_gtest` block and used directly in
`test_detection_layer.cpp`, but only lint tooling (`ament_lint_auto`,
`ament_lint_common`) was listed.

Why this matters: both packages built and tested fine on this machine
the whole time, because these dependencies happened to already be
installed from unrelated earlier work. That's a false positive -
`package.xml` wasn't actually correct, it just never got tested against
a machine that didn't already have everything. Fixed by adding the
missing declarations (commit `109ba22`).

### Stale CMakeCache.txt pointing at the wrong Python interpreter

While proving the fix with a genuinely fresh clone in a scratch
directory, the first `colcon build` attempt failed:
`ModuleNotFoundError: No module named 'catkin_pkg'`, with the traceback
showing CMake invoking `/home/jannatul/projects/telemops/.venv/bin/python3`
- a completely unrelated project's Python virtual environment, active in
the shell from earlier work that session.

Deactivating the venv (`deactivate`) did NOT fix a re-run of the same
build - CMake had already cached the wrong interpreter path in
`build/percept_nav_costmap_plugin/CMakeCache.txt` at first configure
time, and reuses that cached value on subsequent runs rather than
re-detecting Python. Only `rm -rf build install log` (forcing a full
reconfigure) picked up the correct system `/usr/bin/python3`.

Common failure mode, real interview material: any time you switch
between a Python-venv project and a ROS/colcon workspace in the same
terminal session, deactivate the venv *before* the first `colcon build`,
not after a failure - CMake's cache doesn't self-correct once it's
picked up a bad interpreter path. If it does happen, a clean build
(wiping `build/install/log`, not just deactivating and re-running) is
the actual fix.

---

## Verification: clean-clone build proof

Rather than assert "fixed," verified end-to-end from a genuinely fresh
clone (`~/clean_build_test`, no shared state with the main dev
workspace):

1. `git clone` the repo fresh
2. `rosdep check --from-paths src --ignore-src` -> "All system
   dependencies have been satisfied"
3. `colcon build --packages-select percept_nav percept_nav_costmap_plugin`
   -> both packages finished clean, zero manual fixes
4. `./build/percept_nav_costmap_plugin/test_detection_layer` -> all 5
   gtest cases passed

This is the standard this repo's build should be held to going forward:
if `rosdep check` and a clean `colcon build` don't both succeed from a
bare clone, `package.xml` is wrong, regardless of whether the build
happens to work on any one developer's machine.
