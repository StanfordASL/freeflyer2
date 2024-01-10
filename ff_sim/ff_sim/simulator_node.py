# MIT License
#
# Copyright (c) 2023 Stanford Autonomous Systems Lab
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


"""
Simulates the freeflyer.

Maps thrusters duty cycle + wheel control input
to a state evolution over time
-----------------------------------------------
"""

import math
import sys
import numpy as np
import typing as T

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from ff_msgs.msg import (
    FreeFlyerStateStamped,
    Wrench2DStamped,
    ThrusterPWMCommand,
    WheelVelCommand,
    ThrusterBinaryCommand,
)

from ff_params import RobotParams


def simulate_contact(
    state: np.ndarray, robot_radius: float, obstacles: T.Dict[str, T.Any]
) -> T.Tuple[np.ndarray, np.ndarray, bool]:
    """
    Simulate contact with obstacles.

    Args:
        state (np.ndarray): robot state in [x, y, theta, vx, vy, omega]
        robot_radius (float): robot radius
        obstacles (T.Dict[str, T.Any]): cylinder obstacle attributes including
            cyl_pos_x   -- x coordinates
            cyl_pos_y   -- y coordinates
            cyl_rads    -- radii
            cyl_heights -- cylinder heights (for visualization)

    Returns:
        T.Tuple[np.ndarray, np.ndarray, bool]: updated state, contact wrench, contact flag

    """
    B_contact = False

    F_MAG_NOM = 6.0  # in [N]

    radius = robot_radius
    p_x, p_y, theta, v_x, v_y, omega = state

    # check for contacts with cylinders
    n_obs_cyl = len(obstacles["cyl_pos_x"])
    for o_i in range(n_obs_cyl):
        o_x, o_y, o_r = (
            obstacles["cyl_pos_x"][o_i],
            obstacles["cyl_pos_y"][o_i],
            obstacles["cyl_rads"][o_i],
        )
        if (p_x - o_x) ** 2 + (p_y - o_y) ** 2 < (radius + o_r) ** 2:
            print("Contact with obstacle " + str(o_i) + " !")
            B_contact = True

            # normal direction (pointing outside obstacle)
            F_dir = np.array([p_x - o_x, p_y - o_y])
            F_dir = F_dir / np.linalg.norm(F_dir)

            # make position lie on boundary of obstacle
            state[0:2] = np.array([o_x, o_y])
            state[0:2] += F_dir * (radius + o_r + 1e-3)
            state[2] = state[2]

            # bounce with vel. in opposite direction (eq. (13) of https://arxiv.org/pdf/1909.00079.pdf)
            state[3:5] += -2.0 * np.dot(state[3:5], F_dir) * F_dir
            state[5] = -state[5]

            # External contact wrench (N,N,N*m)
            W_contact = np.array([F_MAG_NOM * F_dir[0], F_MAG_NOM * F_dir[1], 0.0])
            return state, W_contact, B_contact

    W_contact = np.zeros(3)
    return state, W_contact, B_contact


class FreeFlyerSimulator(Node):
    """Free-Flyer simulator class."""

    def __init__(self):
        super().__init__("ff_sim_node")
        self.x_dim = 6  # state dimension

        self.p = RobotParams(self)

        # obstacles
        p_obstacles = self.declare_parameters(
            "obstacles",
            [
                ("cyl_pos_x", []),
                ("cyl_pos_y", []),
                ("cyl_rads", []),
                ("cyl_heights", []),
            ],
        )
        self.obstacles = {
            "cyl_pos_x": p_obstacles[0].get_parameter_value().double_array_value,
            "cyl_pos_y": p_obstacles[1].get_parameter_value().double_array_value,
            "cyl_rads": p_obstacles[2].get_parameter_value().double_array_value,
            "cyl_heights": p_obstacles[3].get_parameter_value().double_array_value,
        }

        self.B_ideal = self.declare_parameter("B_ideal", False).get_parameter_value().bool_value

        # simulation params
        p_sim = self.declare_parameters(
            "",
            [
                ("sim_dt", 0.001),  # update period in [s]
                ("discretization", "Euler"),  # discretization scheme from {"Euler", "RungeKutta"}
                ("x_0", [0.6, 2.0, 0.0, 0.0, 0.0, 0.0]),  # initial state
                ("B_sim_contacts", True),  # if True, simulates contacts
            ],
        )
        self.SIM_DT = p_sim[0].get_parameter_value().double_value
        self.DISCRETIZATION = p_sim[1].get_parameter_value().string_value
        self.x_0 = np.array(p_sim[2].get_parameter_value().double_array_value)
        self.B_sim_contacts = p_sim[3].get_parameter_value().bool_value

        # commands, in world frame
        self.F_cmd_body = np.zeros(2)  # (x,y)
        self.theta_dot_cmd = 0.0
        # commands
        self.thrusters_dutycycle_cmd = np.zeros(8)
        self.wheel_vel_cmd = 0.0
        # current states of the freeflyer, in world frame
        self.x_cur = self.x_0.copy()  # (x, y, theta, v_x, v_y, theta_dot)
        assert self.x_cur.shape[0] == self.x_dim, "Wrong size of initial state."

        # subscribers
        self.sub_wheel_cmd_vel = self.create_subscription(
            WheelVelCommand, "commands/velocity", self.update_wheel_cmd_vel_cb, 10
        )
        self.sub_thrusters_cmd_dutycycle = self.create_subscription(
            ThrusterPWMCommand, "commands/duty_cycle", self.update_thrusters_dutycycle_cmd_cb, 10
        )
        self.sub_thrusters_cmd_binary = self.create_subscription(
            ThrusterBinaryCommand, "commands/binary", self.update_thrusters_binary_cmd_cb, 10
        )
        self.sub_state_init = self.create_subscription(
            FreeFlyerStateStamped, "state_init", self.update_state_init_cb, 10
        )

        # ground truth publishers
        self.pub_state = self.create_publisher(FreeFlyerStateStamped, f"sim/state", 10)
        self.pub_wrench = self.create_publisher(Wrench2DStamped, f"sim/wrench", 10)
        self.pub_ext_force = self.create_publisher(Wrench2DStamped, f"sim/ext_force", 10)

        # simulated motion capture
        self.declare_parameter("mocap_noise_xy", 0.001)
        self.declare_parameter("mocap_noise_theta", math.radians(0.1))
        self.pub_mocap = self.create_publisher(PoseStamped, "mocap/sim/pose", 10)

        self.sim_timer = self.create_timer(self.SIM_DT, self.sim_loop)

    def sim_loop(self) -> None:
        if not self.p.loaded:
            return

        now = self.get_clock().now().to_msg()

        # Get thrusters forces
        # Note: assume perfect mapping from duty cycle to true forces. (ToDo: simulate missmatch)
        F_bodyFrame, M = self.thrusters_dutycycle_to_body_wrench(self.thrusters_dutycycle_cmd)
        # Get true torque
        # Note: assume perfect mapping from duty cycle to true forces. (ToDo: simulate missmatch)
        self.F_cmd_body = F_bodyFrame

        # Publish true exerted force on freeflyer, as a mapping of the thrusters command duty cycle
        R = self.get_rotmatrix_body_to_world(self.x_cur[2])
        F_worldFrame = np.matmul(R, F_bodyFrame)
        wrench_msg = Wrench2DStamped()
        wrench_msg.header.stamp = now
        wrench_msg.header.frame_id = "world"
        wrench_msg.wrench.fx = F_worldFrame[0]
        wrench_msg.wrench.fy = F_worldFrame[1]
        wrench_msg.wrench.tz = M
        self.pub_wrench.publish(wrench_msg)

        # Dynamics
        u_cur = np.array([self.F_cmd_body[0], self.F_cmd_body[1], M])

        # ToDo add more precise simulation (smaller dt)
        x_next = self.compute_dynamics_dt(
            self.x_cur, u_cur, self.SIM_DT, discretization=self.DISCRETIZATION
        )

        # simulate contacts
        if self.B_sim_contacts:
            radius = self.p.dynamics["radius"]
            x_next, W_contact, B_contact = simulate_contact(x_next, radius, self.obstacles)

            wrench_ext_msg = Wrench2DStamped()
            wrench_ext_msg.header.stamp = now
            wrench_ext_msg.header.frame_id = "world"
            wrench_ext_msg.wrench.fx = W_contact[0]
            wrench_ext_msg.wrench.fy = W_contact[1]
            wrench_ext_msg.wrench.tz = W_contact[2]
            self.pub_ext_force.publish(wrench_ext_msg)

        self.x_cur = x_next

        # Save as a message
        state = FreeFlyerStateStamped()
        state.header.stamp = now
        state.header.frame_id = "world"
        state.state.pose.x = self.x_cur[0]
        state.state.pose.y = self.x_cur[1]
        state.state.pose.theta = self.x_cur[2]
        state.state.twist.vx = self.x_cur[3]
        state.state.twist.vy = self.x_cur[4]
        state.state.twist.wz = self.x_cur[5]

        # simulated motion capture with noise
        noise_xy = self.get_parameter("mocap_noise_xy").get_parameter_value().double_value
        noise_theta = self.get_parameter("mocap_noise_theta").get_parameter_value().double_value
        mocap = PoseStamped()
        mocap.header.stamp = now
        mocap.header.frame_id = "world"
        mocap.pose.position.x = self.x_cur[0] + np.random.normal(loc=0.0, scale=noise_xy)
        mocap.pose.position.y = self.x_cur[1] + np.random.normal(loc=0.0, scale=noise_xy)
        theta = self.x_cur[2] + np.random.normal(loc=0.0, scale=noise_theta)
        mocap.pose.orientation.w = np.cos(theta / 2)
        mocap.pose.orientation.x = 0.0
        mocap.pose.orientation.y = 0.0
        mocap.pose.orientation.z = np.sin(theta / 2)

        # Publish
        self.pub_state.publish(state)
        self.pub_mocap.publish(mocap)

    def update_wheel_cmd_vel_cb(self, msg: WheelVelCommand) -> None:
        self.wheel_vel_cmd = msg.velocity

    def update_thrusters_dutycycle_cmd_cb(self, msg: ThrusterPWMCommand) -> None:
        # Saturate controls so within [0,1] (in %)
        self.thrusters_dutycycle_cmd = np.clip(msg.duty_cycles, 0.0, 1.0)

    def update_thrusters_binary_cmd_cb(self, msg: ThrusterBinaryCommand) -> None:
        self.thrusters_dutycycle_cmd = np.array(msg.switches, dtype=float)

    def update_state_init_cb(self, msg: FreeFlyerStateStamped) -> None:
        self.x_cur = np.array(
            [
                msg.state.pose.x,
                msg.state.pose.y,
                msg.state.pose.theta,
                msg.state.twist.vx,
                msg.state.twist.vy,
                msg.state.twist.wz,
            ]
        )

    def thrusters_dutycycle_to_body_wrench(self, thrusters_dutycycles_vec):
        # Thrusters Configuration
        #      (2) e_y (1)        ___
        #     <--   ^   -->      /   \
        #    ^  |   |   |  ^     v M  )
        # (3)|--o-------o--|(0)    __/
        #       | free- |
        #       | flyer |   ---> e_x
        #       | robot |
        # (4)|--o-------o--|(7)
        #    v  |       |  v
        #     <--       -->
        #      (5)     (6)

        Fmax = self.p.actuators["F_max_per_thruster"]
        dist = self.p.actuators["thrusters_lever_arm"]

        u_dc = thrusters_dutycycles_vec

        F_x = Fmax * ((u_dc[2] + u_dc[5]) - (u_dc[1] + u_dc[6]))
        F_y = Fmax * ((u_dc[4] + u_dc[7]) - (u_dc[0] + u_dc[3]))

        M = (dist * Fmax) * (
            (u_dc[1] + u_dc[3] + u_dc[5] + u_dc[7]) - (u_dc[0] + u_dc[2] + u_dc[4] + u_dc[6])
        )

        return np.array([F_x, F_y]), M

    def actuators_mapping(self, u_cmd):
        F_max = self.p.actuators["F_body_max"]
        M_max = self.p.actuators["M_body_max"]
        min_inp_percent = self.p.actuators["min_inp_percent"]
        max_inp_percent = self.p.actuators["max_inp_percent"]
        gamma_min = self.p.actuators["gamma_min"]
        gamma_max = self.p.actuators["gamma_max"]

        # returns the true wrench u from a commanded wrench u
        u_max = np.array([F_max, F_max, M_max]) * np.ones_like(u_cmd)
        u_min = min_inp_percent * u_max
        d_max = (
            max_inp_percent * u_max
        )  # inflexion point where input saturation occurs (due to PWMs overlapping)

        # ideal
        u = u_cmd

        # if input norm too small, then set to zero
        # u = np.heaviside(u-u_min, 0.)*u + np.heaviside(-u-u_min, 0.)*u
        def np_sigmoid_sharp(x, k):
            # smooth approximation to np.heaviside (step function)
            # alternatively, sharper variant of sigmoid.
            # k=1 corresponds to sigmoid, and k=\infty is step function.
            return 1.0 / (1.0 + np.exp(-k * x))

        u = np_sigmoid_sharp(u - u_min, gamma_min) * u + np_sigmoid_sharp(-u - u_min, gamma_min) * u

        # increase close to 100%
        u = (
            u
            + np_sigmoid_sharp(u - d_max, gamma_max) * u
            + np_sigmoid_sharp(-u - d_max, gamma_max) * u
        )

        # saturate
        u = np.maximum(-u_max, np.minimum(u_max, u))

        return u

    def get_rotmatrix_body_to_world(self, theta):
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        return R

    def f_dynamics_continuous_time(self, x, u):
        """
        Continuous time dynamics.
        Returns f, s.t. \dot(x) = f(x,u)
        """
        m = self.p.dynamics["mass"]
        J = self.p.dynamics["inertia"]
        p0 = self.p.dynamics["CoM_offset"]
        F_tilt = self.p.dynamics["force_const"]

        # Apply nonlinear actuators mapping
        if not self.B_ideal:
            u = self.actuators_mapping(u)

        # Extract state and control
        r, theta, v, thetadot = x[0:2], x[2], x[3:5], x[5]
        F, M = u[0:2], u[2]

        # Rotation matrix
        R = self.get_rotmatrix_body_to_world(theta)

        f = np.zeros(6)
        f[0:2] = v
        f[2] = thetadot
        thetaddot = (M - F[1] * p0[0] + F[0] * p0[1]) / J
        f[3:5] = np.matmul(
            R, (F / m - (thetaddot * np.array([-p0[1], p0[0]]) - thetadot ** 2 * p0))
        )
        f[5] = thetaddot

        # add constant force due to table tilt
        f[3:5] = f[3:5] + F_tilt / m

        return f

    def compute_dynamics_dt(self, x_k, u_k, dt, discretization="Euler"):
        # discretization = {"Euler", "RungeKutta"}

        if discretization == "Euler":
            x_next = x_k + dt * self.f_dynamics_continuous_time(x_k, u_k)

        elif discretization == "RungeKutta":
            k1 = self.f_dynamics_continuous_time(x_k, u_k)
            x2 = x_k + 0.5 * dt * k1
            k2 = self.f_dynamics_continuous_time(x2, u_k)
            x3 = x_k + 0.5 * dt * k2
            k3 = self.f_dynamics_continuous_time(x3, u_k)
            x4 = x_k + dt * k3
            k4 = self.f_dynamics_continuous_time(x4, u_k)

            x_next = x_k + (1.0 / 6.0) * dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        else:
            print("[FreeFlyerSimulator::compute_dynamics_dt]: Unknown Discretization Scheme.")

        return x_next


def main():
    rclpy.init()
    ff_sim = FreeFlyerSimulator()
    rclpy.spin(ff_sim)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
