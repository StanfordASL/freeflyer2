# freeflyer2
![foxy build](https://github.com/StanfordASL/freeflyer2/actions/workflows/foxy.yml/badge.svg)
![humble build](https://github.com/StanfordASL/freeflyer2/actions/workflows/humble.yml/badge.svg)
![rolling build](https://github.com/StanfordASL/freeflyer2/actions/workflows/rolling.yml/badge.svg)

ASL FreeFlyer software stack with ROS2. See build status for support on ROS2 distrbutions.

## Quick Start
1. Install ROS2 ([Foxy](https://docs.ros.org/en/foxy/Installation.html),
[Humble](https://docs.ros.org/en/humble/Installation.html), or
[Rolling](https://docs.ros.org/en/rolling/Installation.html))
2. Clone the repo into a ROS2 workspace, e.g.
```sh
mkdir -p ~/ff_ws/src
git clone git@github.com:StanfordASL/freeflyer2.git ~/ff_ws/src/freelyer2
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

## Package Layout

* `freeflyer` -- top level pacakge (contains just launch files)
* `ff_control` -- implement different controllers
* `ff_msgs` -- custom message types
* `ff_params` -- shared parameters about dynamics and actuators
* `ff_sim` -- a lightweight pure Python simulator
* `ff_viz` -- RVIZ visualization
