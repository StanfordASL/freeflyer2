# freeflyer package

Top level package that launches multiple modules together.

### Launch Files

**PD Waypoint Control in Simulator**
```sh
ros2 launch freeflyer sim_pd.launch.py
```

**Keyboard Teleop Velocity Control in Simulator**
```sh
ros2 launch freeflyer sim_key_teleop.launch.py
```
*Note*: this will launch a new terminal window where you can input keyboard commands

**Finite Difference State Estimation with RVIZ Visuliazation**

The following script is useful for debugging connections between hardware freeflyer and
the motion capture system.
```sh
ros2 launch freeflyer estimation_viz.launch.py
```

**PD Waypoint Control in Hardware**
```sh
ros2 launch freeflyer hardware_pd.launch.py rviz:=false  # or set rviz:=true to launch RVIZ
```
