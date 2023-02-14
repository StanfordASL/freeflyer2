# freeflyer2

ASL FreeFlyer software stack with ROS2

## Quick Start
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. Clone the repo into a work space, e.g.
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

## Module Layout

* `ff_viz` -- visualization module
* `ff_sim` -- simulator module
