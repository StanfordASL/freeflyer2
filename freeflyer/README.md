# freeflyer package

Top level package that launches multiple modules together.

### Quick Examples

**PD Waypoint Control in Simulator**
```sh
ros2 launch freeflyer sim_pd.launch.py              # launch CPP node by default
ros2 launch freeflyer sim_pd.launch.py language:=py # launch Python node
```

**Keyboard Teleop Velocity Control in Simulator**
```sh
ros2 launch freeflyer sim_key_teleop.launch.py
```
*Note*: this will launch a new terminal window where you can input keyboard commands
