# freeflyer2

ASL FreeFlyer software stack with ROS2

## Quick Start
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
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
