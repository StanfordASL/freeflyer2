# freeflyer2
![foxy build](https://github.com/StanfordASL/freeflyer2/actions/workflows/foxy.yml/badge.svg?event=push)
![humble build](https://github.com/StanfordASL/freeflyer2/actions/workflows/humble.yml/badge.svg?event=push)
![rolling build](https://github.com/StanfordASL/freeflyer2/actions/workflows/rolling.yml/badge.svg?event=push)

ASL FreeFlyer software stack with ROS2. See build status for support on ROS2 distrbutions.

## Quick Start
1. Install ROS2 ([Foxy](https://docs.ros.org/en/foxy/Installation.html),
[Humble](https://docs.ros.org/en/humble/Installation.html), or
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
4. Build the code
```sh
cd ~/ff_ws && colcon build
```
5. Source workspace install
```sh
source ~/ff_ws/install/local_setup.bash
```

## Development Guide
The CI will build and run all registered tests. PR can be merged only when all status checks
pass. [ROS2 code styles](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
are enforced for both C++ and Python. Here are some tips for fixing code style issues for a PR.

#### Copyright
Every source file (e.g. `.cpp`, `.hpp`, `.py`) including launch files should have a copy of
the license at the very top. See any source files for an example.

#### C++ Code Style
1. Run `ament_uncrustify --reformat` to automatically format C++ source files.
2. Run `ament_cpplint` to check for style violations and fix them manually.

#### Python Code Style
For the first time, you will need to run the following command to install some dev packages
```sh
sudo apt install -y \
   python3-flake8-docstrings \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```
1. Run `ament_flake8` to check for code style violations and fix them manually.
2. Run `ament_pep257` to check for docstring style violations and fix them manually.

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
* `ff_control` -- implement different controllers
* `ff_msgs` -- custom message types
* `ff_params` -- shared parameters about dynamics and actuators
* `ff_sim` -- a lightweight pure Python simulator
* `ff_viz` -- RVIZ visualization
