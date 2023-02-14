""" -----------------------------------------------
    Simulates the freeflyer
    Maps thrusters duty cycle + wheel control input
      to a state evolution over time
----------------------------------------------- """

import math
import sys
import numpy as np
import typing as T

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float64, Float64MultiArray

def simulate_contact(
    state: np.ndarray,
    robot_radius: float,
    obstacles: T.Dict[str, T.Any]
) -> T.Tuple[np.ndarray, np.ndarray, bool]:
    B_contact = False

    F_MAG_NOM = 6. # in [N]

    radius                           = robot_radius
    p_x, p_y, theta, v_x, v_y, omega = state

    # check for contacts with cylinders
    n_obs_cyl = len(obstacles['cyl_pos_x'])
    for o_i in range(n_obs_cyl):
        o_x, o_y, o_r = obstacles['cyl_pos_x'][o_i], obstacles['cyl_pos_y'][o_i], obstacles['cyl_rads'][o_i]
        if (p_x-o_x)**2 + (p_y-o_y)**2 < (radius+o_r)**2:
            print("Contact with obstacle " + str(o_i) + " !" )
            B_contact = True

            # normal direction (pointing outside obstacle)
            F_dir = np.array([p_x-o_x, p_y-o_y])
            F_dir = F_dir / np.linalg.norm(F_dir)

            # make position lie on boundary of obstacle
            state[0:2] =  np.array([o_x, o_y])
            state[0:2] += F_dir*(radius+o_r+1e-3)
            state[ 2 ] = state[2]

            # bounce with vel. in opposite direction (eq. (13) of https://arxiv.org/pdf/1909.00079.pdf)
            state[3:5] += -2. * np.dot(state[3:5],F_dir)*F_dir
            state[ 5 ] = -state[5]

            # External contact wrench (N,N,N*m)
            W_contact = np.array([F_MAG_NOM*F_dir[0], F_MAG_NOM*F_dir[1], 0.])
            return state, W_contact, B_contact

    W_contact = np.zeros(3)
    return state, W_contact, B_contact

class FreeFlyerSimulator(Node):
    """Free-Flyer simulator class"""
    def __init__(self):
        super().__init__("ff_sim_node")
        self.x_dim = 6  # state dimension

        # robot name
        p_robot_name = self.declare_parameter("robot_name", "robot")
        self.robot_name = p_robot_name.get_parameter_value().string_value

        # dynamics params
        p_dynamics = self.declare_parameters("dynamics", [
            ("mass", 16.),                      # true total mass [kg]
            ("inertia", 0.18),                  # true inertia at robot CoM (no payload) [kg*m^2]
            ("CoM_offset", [0., 0.]),           # CoM offset, in body frame [m]
            ("force_const", [0.005, 0.005]),    # Constant force (e.g. table tilt), in world frame [N]
        ])
        self.m = p_dynamics[0].get_parameter_value().double_value
        self.J = p_dynamics[1].get_parameter_value().double_value
        self.p0 = np.array(p_dynamics[2].get_parameter_value().double_array_value)
        self.F_tilt = np.array(p_dynamics[3].get_parameter_value().double_array_value)

        # obstacles
        p_obstacles = self.declare_parameters("obstacles",[
            ("cyl_pos_x", []),
            ("cyl_pos_y", []),
            ("cyl_rads", []),
            ("cyl_heights", []),
        ])
        self.obstacles = {
            "cyl_pos_x": p_obstacles[0].get_parameter_value().double_array_value,
            "cyl_pos_y": p_obstacles[1].get_parameter_value().double_array_value,
            "cyl_rads": p_obstacles[2].get_parameter_value().double_array_value,
            "cyl_heights": p_obstacles[3].get_parameter_value().double_array_value,
        }

        # robot params
        p_robot = self.declare_parameters("robot", [
            ("f_max_per_thruster", 0.2),        # max force in [N] of one thruster
            ("thrusters_lever_arm", 0.11461),   # lever arm, in [m]
            ("robot_radius", 0.1),              # total radius of robot, in [m]
        ])
        self.F_MAX_PER_THRUSTER = p_robot[0].get_parameter_value().double_value
        self.THRUSTERS_LEVER_ARM = p_robot[1].get_parameter_value().double_value
        self.radius = p_robot[2].get_parameter_value().double_value

        p_actuators = self.declare_parameters("actuator", [
            ("B_ideal", False),         # if True, then ideal input response
            ("F_max", 0.4),             # force max, in [N]
            ("M_max", 0.05),            # moment / torque max, in [Nm]
            ("min_inp_percent", 0.05),  # in [0,0.15]  - inputs are zero close to min input
            ("max_inp_percent", 0.95),  # in [0.8,1.0] - inflexion pt for input sat (PWMs overlapping)
            ("gamma_min", 500.),        # in [100,1000] - sharpness for inp saturation close to min inp
            ("gamma_max", 500.),        # in [100,1000] - sharpness for inp saturation close to max inp
        ])
        self.B_ideal = p_actuators[0].get_parameter_value().bool_value
        self.F_max = p_actuators[1].get_parameter_value().double_value
        self.M_max = p_actuators[2].get_parameter_value().double_value
        self.min_inp_percent = p_actuators[3].get_parameter_value().double_value
        self.max_inp_percent = p_actuators[4].get_parameter_value().double_value
        self.gamma_min = p_actuators[5].get_parameter_value().double_value
        self.gamma_max = p_actuators[6].get_parameter_value().double_value

        # simulation params
        p_sim = self.declare_parameters("simulation", [
            ("sim_dt", 0.01),                   # update period in [s]
            ("discretization", "Euler"),        # discretization scheme from {"Euler", "RungeKutta"}
            ("x_0", [0.6, 2., 0., 0., 0., 0.]), # initial state
            ("B_sim_contacts", True),           # if True, simulates contacts
        ])
        self.SIM_DT = p_sim[0].get_parameter_value().double_value
        self.DISCRETIZATION = p_sim[1].get_parameter_value().string_value
        self.x_0 = np.array(p_sim[2].get_parameter_value().double_array_value)
        self.B_sim_contacts = p_sim[3].get_parameter_value().bool_value

        # commands, in world frame
        self.F_cmd_body    = np.zeros(2) # (x,y)
        self.theta_dot_cmd = 0.
        # commands
        self.thrusters_dutycycle_cmd = np.zeros(8)
        self.wheel_vel_cmd           = 0.
        # current states of the freeflyer, in world frame
        self.x_cur = self.x_0.copy() # (x, y, theta, v_x, v_y, theta_dot)
        assert self.x_cur.shape[0]==self.x_dim, "Wrong size of initial state."

        # subscribers
        self.sub_wheel_cmd_vel = self.create_subscription(Float64,
            f"/{self.robot_name}/commands/velocity", self.update_wheel_cmd_vel_cb, 10)
        self.sub_thrusters_cmd_dutycycle = self.create_subscription(Float64MultiArray,
            f"/{self.robot_name}/commands/duty_cycle", self.update_thrusters_dutycycle_cmd_cb, 10)
        self.sub_pose_init = self.create_subscription(PoseStamped,
            f"/{self.robot_name}/pose_init", self.update_pose_init_cb, 10)
        self.sub_twist_init = self.create_subscription(PoseStamped,
            f"/{self.robot_name}/twist_init", self.update_twist_init_cb, 10)

        # ground truth publishers
        self.pub_pose = self.create_publisher(PoseStamped, f"/{self.robot_name}/gt/pose", 10)
        self.pub_twist = self.create_publisher(TwistStamped, f"/{self.robot_name}/gt/twist", 10)
        self.pub_wrench = self.create_publisher(WrenchStamped, f"/{self.robot_name}/gt/wrench", 10)
        self.pub_ext_force = self.create_publisher(WrenchStamped, f"/{self.robot_name}/gt/ext_force", 10)

        self.sim_timer = self.create_timer(self.SIM_DT, self.sim_loop)

    def sim_loop(self) -> None:
        now = self.get_clock().now().to_msg()

        # Get thrusters forces
        # Note: assume perfect mapping from duty cycle to true forces. (ToDo: simulate missmatch)
        F_bodyFrame, M = self.thrusters_dutycycle_to_body_wrench(self.thrusters_dutycycle_cmd)
        # Get true torque
        # Note: assume perfect mapping from duty cycle to true forces. (ToDo: simulate missmatch)
        self.F_cmd_body = F_bodyFrame

        # Publish true exerted force on freeflyer, as a mapping of the thrusters command duty cycle
        R            = self.get_rotmatrix_body_to_world(self.x_cur[2])
        F_worldFrame = np.matmul(R, F_bodyFrame)
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = now
        wrench_msg.header.frame_id = "map"
        wrench_msg.wrench.force.x = F_worldFrame[0]
        wrench_msg.wrench.force.y = F_worldFrame[1]
        wrench_msg.wrench.torque.z = M
        self.pub_wrench.publish(wrench_msg)

        # Dynamics
        u_cur = np.array([self.F_cmd_body[0], self.F_cmd_body[1], M])

        # ToDo add more precise simulation (smaller dt)
        x_next = self.compute_dynamics_dt(self.x_cur, u_cur, self.SIM_DT,
                                          discretization=self.DISCRETIZATION)

        # simulate contacts
        if self.B_sim_contacts:
            x_next, W_contact, B_contact = simulate_contact(x_next, self.radius, self.obstacles)

            wrench_ext_msg = WrenchStamped()
            wrench_ext_msg.header.stamp = now
            wrench_ext_msg.header.frame_id = "map"
            wrench_ext_msg.wrench.force.x = W_contact[0]
            wrench_ext_msg.wrench.force.y = W_contact[1]
            wrench_ext_msg.wrench.torque.z = W_contact[2]
            self.pub_ext_force.publish(wrench_ext_msg)

        self.x_cur = x_next

        # Save as a message
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = self.x_cur[:2]
        pose.pose.orientation.x = 0.
        pose.pose.orientation.y = 0.
        pose.pose.orientation.z = math.sin(self.x_cur[2] / 2)
        pose.pose.orientation.w = math.cos(self.x_cur[2] / 2)

        twist = TwistStamped()
        twist.header.stamp = now
        twist.header.frame_id = "map"
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z = self.x_cur[3:]

        # Publish
        self.pub_pose.publish(pose)
        self.pub_twist.publish(twist)

    def update_wheel_cmd_vel_cb(self, msg: Float64) -> None:
        self.wheel_vel_cmd = msg.data

    def update_thrusters_dutycycle_cmd_cb(self, msg: Float64MultiArray) -> None:
        # Saturate controls so within [0,1] (in %)
        self.thrusters_dutycycle_cmd = np.clip(msg.data, 0., 1.)

    def update_pose_init_cb(self, msg: PoseStamped) -> None:
        self.x_cur[:3] = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            2 * math.acos(msg.pose.orientation.w),
        ])

    def update_twist_init_cb(self, msg: TwistStamped) -> None:
        self.x_cur[3:] = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z,
        ])

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

        Fmax = self.F_MAX_PER_THRUSTER
        dist = self.THRUSTERS_LEVER_ARM

        u_dc = thrusters_dutycycles_vec

        F_x = Fmax * ( (u_dc[2]+u_dc[5]) - (u_dc[1]+u_dc[6]) )
        F_y = Fmax * ( (u_dc[4]+u_dc[7]) - (u_dc[0]+u_dc[3]) )

        M   = (dist*Fmax) * ( (u_dc[1]+u_dc[3]+u_dc[5]+u_dc[7]) -
                              (u_dc[0]+u_dc[2]+u_dc[4]+u_dc[6]) )

        return np.array([F_x, F_y]), M

    def actuators_mapping(self, u_cmd):
        # returns the true wrench u from a commanded wrench u
        u_max = np.array([self.F_max, self.F_max, self.M_max])*np.ones_like(u_cmd)
        u_min = self.min_inp_percent * u_max
        d_max = self.max_inp_percent * u_max # inflexion point where input saturation occurs (due to PWMs overlapping)

        # ideal
        u = u_cmd

        # if input norm too small, then set to zero
        # u = np.heaviside(u-u_min, 0.)*u + np.heaviside(-u-u_min, 0.)*u
        def np_sigmoid_sharp(x, k):
            # smooth approximation to np.heaviside (step function)
            # alternatively, sharper variant of sigmoid.
            # k=1 corresponds to sigmoid, and k=\infty is step function.
            return 1./(1. + np.exp(-k*x))

        u = np_sigmoid_sharp(u-u_min, self.gamma_min)*u + np_sigmoid_sharp(-u-u_min, self.gamma_min)*u

        # increase close to 100%
        u = u + np_sigmoid_sharp(u-d_max, self.gamma_max)*u + np_sigmoid_sharp(-u-d_max, self.gamma_max)*u

        # saturate
        u = np.maximum(-u_max,
                       np.minimum(u_max, u))

        return u

    def get_rotmatrix_body_to_world(self, theta):
        R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        return R

    def f_dynamics_continuous_time(self, x, u):
        """
        Continuous time dynamics.
        Returns f, s.t. \dot(x) = f(x,u)
        """
        m, J, p0, F_tilt  = self.m, self.J, self.p0, self.F_tilt

        # Apply nonlinear actuators mapping
        if not self.B_ideal:
            u = self.actuators_mapping(u)


        # Extract state and control
        r, theta, v, thetadot = x[0:2], x[2], x[3:5], x[5]
        F, M                  = u[0:2], u[2]

        # Rotation matrix
        R = self.get_rotmatrix_body_to_world(theta)

        f = np.zeros(6)
        f[0:2] = v
        f[2]   = thetadot
        thetaddot = (M - F[1]*p0[0] + F[0]*p0[1]) / J
        f[3:5] = np.matmul(R, (F/m -
                               (thetaddot*np.array([-p0[1],p0[0]]) - thetadot**2*p0) ) )
        f[5]   = thetaddot

        # add constant force due to table tilt
        f[3:5] = f[3:5] + F_tilt/m


        return f

    def compute_dynamics_dt(self, x_k, u_k, dt, discretization="Euler"):
        # discretization = {"Euler", "RungeKutta"}

        if discretization == "Euler":
            x_next = x_k + dt * self.f_dynamics_continuous_time(x_k, u_k)

        elif discretization == "RungeKutta":
            k1 = self.f_dynamics_continuous_time(x_k, u_k)
            x2 = x_k + 0.5*dt*k1
            k2 = self.f_dynamics_continuous_time(x2,  u_k)
            x3 = x_k + 0.5*dt*k2
            k3 = self.f_dynamics_continuous_time(x3,  u_k)
            x4 = x_k + dt*k3
            k4 = self.f_dynamics_continuous_time(x4,  u_k)

            x_next = x_k + (1./6.)*dt*(k1 + 2.*k2 + 2.*k3 + k4)

        else:
            print("[FreeFlyerSimulator::compute_dynamics_dt]: Unknown Discretization Scheme.")

        return x_next


def main():
    rclpy.init()
    ff_sim = FreeFlyerSimulator()
    rclpy.spin(ff_sim)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
