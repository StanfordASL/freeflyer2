# ff\_drivers

Driver code that interfaces with Freeflyer hardware such as thruster valves and the inertia wheel.
Designed around an embedded Linux device [Odroid N2L](https://wiki.odroid.com/odroid-n2l).

## Quick Start on ODROID
Follow the steps below to bring all the hardware actuators into ROS2 network.
1. Login to `root@<odroid ip>`

    **Important Note**: we need to use the `root` account as it has
    the right permission to access hardware interfaces.
2. Set up the freeflyer workspace `~/ff_ws` following steps 1 - 3 on the
    [main setup guide](../README.md)
3. Build the driver code
```sh
cd ~/ff_ws && colcon build --packages-up-to ff_drivers
```
4. Source workspace install
```sh
source ~/ff_ws/install/local_setup.zsh
```
5. Launch hardware bringup
```sh
ros2 launch ff_drivers hardware_bringup.launch.py
```
