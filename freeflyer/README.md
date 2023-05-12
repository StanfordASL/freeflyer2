# freeflyer package

Top level package that launches multiple modules together.

### Quick Examples

**PD Waypoint Control in Simulator**
```sh
ros2 launch freeflyer sim_pd.launch.py          # launch CPP implementation by default
ros2 launch freeflyer sim_pd.launch.py impl:=py # launch Python implementation
```

**Keyboard Teleop Velocity Control in Simulator**
```sh
ros2 launch freeflyer sim_key_teleop.launch.py          # launch CPP implementation by default
ros2 launch freeflyer sim_key_teleop.launch.py impl:=py # launch Python implementation
```
*Note*: this will launch a new terminal window where you can input keyboard commands
