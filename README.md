# freeflyer2
![humble build](https://github.com/StanfordASL/freeflyer2/actions/workflows/humble.yml/badge.svg?event=push)
![iron build](https://github.com/StanfordASL/freeflyer2/actions/workflows/iron.yml/badge.svg?event=push)
![rolling build](https://github.com/StanfordASL/freeflyer2/actions/workflows/rolling.yml/badge.svg?event=push)

ASL FreeFlyer software stack with ROS2. See build status for support on ROS2 distrbutions.

## Quick Start
1. Install ROS2 (
[Humble](https://docs.ros.org/en/humble/Installation.html) or
[Rolling](https://docs.ros.org/en/rolling/Installation.html))
2. Clone the repo into a ROS2 workspace, e.g.
```sh
mkdir -p ~/ff_ws/src
git clone git@github.com:StanfordASL/freeflyer2.git ~/ff_ws/src/freeflyer2
```
3. Install dependencies
```sh
rosdep update && rosdep install --from-paths ~/ff_ws/src --ignore-src -y
```
4. Build the code (skipping the driver package)
```sh
cd ~/ff_ws && colcon build --packages-skip ff_drivers
```

5. Source workspace install
```sh
source ~/ff_ws/install/local_setup.bash
```

## Development Guide
The CI will build and run all registered tests. PR can be merged only when all status checks
pass. [ROS2 code styles](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
are enforced for C++. Python code style is enforced through `black`.
Here are some tips for fixing code style issues for a PR.

Run the following command to manually run all tests and see detailed test results including
format violation
```sh
$ colcon build && colcon test && colcon test-result --verbose
```

#### Copyright
Every source file (e.g. `.cpp`, `.hpp`, `.py`) including launch files should have a copy of
the license at the very top. See any source files for an example.

#### C++ Code Style
1. Run `ament_uncrustify --reformat` to automatically format C++ source files.
2. Run `ament_cpplint` to check for style violations and fix them manually.

#### Python Code Style
Please install `black` with `pip install black` for the first time.

Importantly
- to format all Python code with black automatically run
```sh
$ cd freeflyer2
$ black .
```
- to check formatting without changing the files run
```sh
$ cd freeflyer2
$ black --check .
```

#### Disable Code Style Test
If you hate the ROS2 conventions so bad, you can disable specific code style test by adding
the following line before `ament_lint_auto_find_test_dependencies()` in `CMakeLists.txt`.
```cmake
set(AMENT_LINT_AUTO_EXCLUDE <test1> [<test2> ...])
```
For example, if you want to disable code style checks for all Python files, you can add
```cmake
set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257)
```

## Package Layout

* `freeflyer` -- top level pacakge (contains just launch files)
* `ff_estimate` -- implement different state estimators
* `ff_control` -- implement different controllers
* `ff_drivers` -- driver code that interfaces with hardware
* `ff_msgs` -- custom message types
* `ff_params` -- shared parameters about dynamics and actuators
* `ff_sim` -- a lightweight pure Python simulator
* `ff_viz` -- RVIZ visualization

Some random stuff
