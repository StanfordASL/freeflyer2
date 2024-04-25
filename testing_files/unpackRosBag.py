#!/usr/bin/env python
# MIT License
#
# Copyright (c) 2024 Stanford Autonomous Systems Lab
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

# Adapted from: https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import math
import numpy as np
from pathlib import Path
import os

typestore = get_typestore(Stores.ROS2_HUMBLE)
add_types = {}

prefix = "./ff_msgs/msg/"
names = [
    "FreeFlyerStateStamped",
    "ControllerMetrics",
    "FreeFlyerState",
    "ThrusterCommand",
    "Pose2D",
    "Twist2D",
]
msgfiles = [prefix + name + ".msg" for name in names]
msgnames = ["ff_msgs/msg/" + name for name in names]

for i, pathstr in enumerate(msgfiles):
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding="utf-8")
    add_types.update(get_types_from_msg(msgdef, msgnames[i]))

typestore.register(add_types)


START_POINT = [0.5, 0.5, -math.pi / 2]
folders = [
    "./testing_files/experiment_data/exp_pd_turnaround/",
    "./testing_files/experiment_data/exp_opt_turnaround/",
    "./testing_files/experiment_data/exp_pd_short/",
    "./testing_files/experiment_data/exp_opt_short/",
    "./testing_files/experiment_data/exp_pd_long/",
    "./testing_files/experiment_data/exp_opt_long/",
]
ROSBAG_NAMES = []
for i, folder in enumerate(folders):
    ROSBAG_NAMES.append([])
    filelist = os.listdir(folder)
    filelist.sort()
    for filename in filelist:
        ROSBAG_NAMES[i].append(folder + filename)

TITLES = [
    "PD Baseline Controller Turnaround",
    "Optimization Controller Turnaround",
    "PD Baseline Controller Short-Distance",
    "Optimization Controller Short-Distance",
    "PD Baseline Controller Long-Distance",
    "Optimization Controller Long-Distance",
]
EXPERIMENT = [0, 0, 1, 1, 2, 2]
COLORS = ["cornflowerblue", "orangered", "mediumseagreen"]
YLIMS_BOUNDS = [
    [(0.45, 0.55), (0.45, 0.55), (-np.pi - 0.2, np.pi + 0.2), (-10, 120)],
    [(0.4, 1.1), (0.4, 1.1), (-np.pi - 0.2, np.pi + 0.2), (-10, 140)],
    [(0.4, 2.3), (0.4, 1.65), (-np.pi - 0.2, np.pi + 0.2), (-10, 250)],
]

POS_THRESHOLD = 0.05
THETA_THRESHOLD = 0.174
NUM_STATS = 9
STATS = [
    "Pos Rise Time",
    "Pos Settling Time",
    "Pos Max Overshoot",
    "Pos RMS Settled Err",
    "Theta Rise Time",
    "Theta Settling Time",
    "Theta Max Overshoot",
    "Theta RMS Settled Err",
    "Settling Gas Used",
]


def quat2euler(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def define_experiment(exp_number):
    FIRST_GOAL, RETURN_GOAL = None, None
    if exp_number == 0:
        FIRST_GOAL = [0.5, 0.5, math.pi / 2]
        RETURN_GOAL = START_POINT
    elif exp_number == 1:
        FIRST_GOAL = [1, 1, math.pi / 2]
        RETURN_GOAL = START_POINT
    elif exp_number == 2:
        FIRST_GOAL = [2.0, 1.5, math.pi / 2]
        RETURN_GOAL = START_POINT
    else:
        print("ERR: Invalid Experiment mode!")
    return FIRST_GOAL, RETURN_GOAL


def unpack_rosbag(rosbag_name, exp_number):
    FIRST_GOAL, RETURN_GOAL = define_experiment(exp_number)

    t0 = 0
    thruster_0 = 0
    experiment_start = False
    track_thruster_start = False

    pos_time = []
    xlist = []
    ylist = []
    thlist = []

    metrics_time = []
    totalgaslist = []

    goal_time = []
    goalxlist = []
    goalylist = []
    goalthlist = []

    thruster_time = []
    thruster_outputs = []

    # create reader instance and open for reading
    with Reader(rosbag_name) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if experiment_start:
                try:
                    if connection.topic == "/vrpn_mocap/robot/pose":
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        pos_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0)
                        xlist.append(msg.pose.position.x)
                        ylist.append(msg.pose.position.y)
                        th = quat2euler(
                            msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w,
                        )
                        thlist.append(th[2])  # Yaw
                    elif connection.topic == "/robot/ctrl/state":
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        goal_time.append(
                            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0
                        )
                        goalxlist.append(msg.state.pose.x)
                        goalylist.append(msg.state.pose.y)
                        goalthlist.append(msg.state.pose.theta)  # Yaw
                    elif connection.topic == "/robot/metrics/controller":
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        if not track_thruster_start:
                            thruster_0 = msg.total_gas_time
                            track_thruster_start = True
                        metrics_time.append(
                            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0
                        )
                        totalgaslist.append(msg.total_gas_time - thruster_0)
                    elif connection.topic == "/robot/commands/binary_thrust":
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        thruster_time.append(
                            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0
                        )
                        thruster_outputs.append(msg.switches)

                except KeyError:
                    print("Err: Not ready to process these messages yet!")
            else:
                if connection.topic == "/robot/ctrl/state":
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    pos = [msg.state.pose.x, msg.state.pose.y, msg.state.pose.theta]
                    if pos == FIRST_GOAL:
                        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                        experiment_start = True
                        goal_time.append(0)
                        goalxlist.append(pos[0])
                        goalylist.append(pos[1])
                        goalthlist.append(pos[2])  # Yaw

    return (
        pos_time,
        xlist,
        ylist,
        thlist,
        goal_time,
        goalxlist,
        goalylist,
        goalthlist,
        metrics_time,
        totalgaslist,
        thruster_time,
        thruster_outputs,
    )


def plot_experiment_results(unpacked_vals, color_ind, exp_number, fig=None, axs=None):
    (
        pos_time,
        xlist,
        ylist,
        thlist,
        goal_time,
        goalxlist,
        goalylist,
        goalthlist,
        metrics_time,
        totalgaslist,
        thruster_time,
        thruster_outputs,
    ) = unpacked_vals

    if fig is None or axs is None:
        fig, axs = plt.subplots(4, 1, constrained_layout=True, sharex=True)
        fig.set_figheight(9)
        fig.set_figwidth(7)
    # Artificially add extra goal command to data to make goal step function look right
    goal_time.append(pos_time[-1])
    goalxlist.append(goalxlist[-1])
    goalylist.append(goalylist[-1])
    goalthlist.append(goalthlist[-1])

    axs[0].plot(pos_time, xlist, COLORS[color_ind])
    axs[0].plot(goal_time, goalxlist, COLORS[color_ind], linestyle="--", drawstyle="steps-post")
    axs[0].set_ylabel("X-Position [m]")
    axs[0].grid(True, which="major", axis="y")
    axs[0].set_ylim(YLIMS_BOUNDS[exp_number][0])

    axs[1].plot(pos_time, ylist, COLORS[color_ind])
    axs[1].plot(goal_time, goalylist, COLORS[color_ind], linestyle="--", drawstyle="steps-post")
    axs[1].set_ylabel("Y-Position [m]")
    axs[1].grid(True, which="major", axis="y")
    axs[1].set_ylim(YLIMS_BOUNDS[exp_number][1])

    axs[2].plot(pos_time, thlist, COLORS[color_ind])
    axs[2].plot(goal_time, goalthlist, COLORS[color_ind], linestyle="--", drawstyle="steps-post")
    axs[2].set_ylabel("Orientation [rad]")
    axs[2].grid(True, which="major", axis="y")
    axs[2].set_ylim(YLIMS_BOUNDS[exp_number][2])

    axs[3].plot(metrics_time, totalgaslist, COLORS[color_ind])
    axs[3].set_ylabel("Total Time Thrusters On [s]")
    axs[3].grid(True, which="major", axis="y")
    axs[3].set_xlabel("Time [s]")
    axs[3].set_ylim(YLIMS_BOUNDS[exp_number][3])

    return fig, axs


def extract_performance_metrics(unpacked_vals, exp_number):
    FIRST_GOAL, RETURN_GOAL = define_experiment(exp_number)

    (
        pos_time,
        xlist,
        ylist,
        thlist,
        goal_time,
        goalxlist,
        goalylist,
        goalthlist,
        metrics_time,
        totalgaslist,
        thruster_time,
        thruster_outputs,
    ) = unpacked_vals
    xchange = goalxlist.index(RETURN_GOAL[0])
    ychange = goalylist.index(RETURN_GOAL[1])
    thchange = goalthlist.index(RETURN_GOAL[2])
    goal_change_time = goal_time[max(xchange, ychange, thchange)]

    # Divide data into two phases
    pos_index = np.where(np.array(pos_time) > goal_change_time)[0][0]
    metrics_index = np.where(np.array(metrics_time) > goal_change_time)[0][0]

    # Process first phase (START->FIRST_GOAL)
    first_slice = (
        pos_time[:pos_index],
        xlist[:pos_index],
        ylist[:pos_index],
        thlist[:pos_index],
        metrics_time[:metrics_index],
        totalgaslist[:metrics_index],
    )
    calcmetrics_1 = experiment_analysis(RETURN_GOAL, FIRST_GOAL, first_slice, exp_number)

    # Process second phase (FIRST_GOAL->START)
    second_slice = (
        pos_time[pos_index:],
        xlist[pos_index:],
        ylist[pos_index:],
        thlist[pos_index:],
        metrics_time[metrics_index:],
        totalgaslist[metrics_index:],
    )
    calcmetrics_2 = experiment_analysis(FIRST_GOAL, RETURN_GOAL, second_slice, exp_number)

    return calcmetrics_1, calcmetrics_2


def experiment_analysis(start, goal, sliced_data, exp_number):
    pos_time, xlist, ylist, thlist, metrics_time, totalgaslist = sliced_data
    position_matrix = np.hstack(
        (np.array(xlist).reshape((-1, 1)), np.array(ylist).reshape((-1, 1)))
    )
    pos_err = np.linalg.norm(position_matrix - np.array(goal[:2]), axis=1)
    th_err = np.abs(np.array(thlist) - goal[2])

    pos_t0 = pos_time[0]
    pos_time = [t - pos_t0 for t in pos_time]
    metrics_t0 = metrics_time[0]
    metrics_time = [t - metrics_t0 for t in metrics_time]

    # Process position metrics
    if exp_number == 0:  # No settling/rising when just holding position
        posRiseInd = 0
        posRiseTime = pos_time[posRiseInd]
        posSettledInd = 0
        posSettlingTime = pos_time[posSettledInd]
    else:
        posRiseInd = np.where(pos_err < POS_THRESHOLD)[0][0]
        posRiseTime = pos_time[posRiseInd]

        if np.any(pos_err > POS_THRESHOLD):
            posSettledInd = np.where(pos_err > POS_THRESHOLD)[0][-1]
            posSettlingTime = pos_time[posSettledInd]
            if posSettlingTime == pos_time[-1]:
                print("Position Never settles!", position_matrix[-1, :], pos_err[-1])
                posSettlingTime = None
        else:
            print("Position Never diverges!")
            posSettledInd = 0
            posSettlingTime = pos_time[posSettledInd]

    if posSettlingTime is None:
        posRMSSettledErr = None
        print("No valid settled error to look at")
    else:
        posRMSSettledErr = np.linalg.norm(pos_err[posSettledInd:]) / np.sqrt(
            pos_err.shape[0] - posSettledInd
        )

    # print(pos_err[:5], np.where(pos_err < POS_THRESHOLD)[0][0])

    posMaxOvershoot = np.max(pos_err[posRiseInd:])

    # Process orientation metrics
    thRiseInd = np.where(th_err < THETA_THRESHOLD)[0][0]
    thRiseTime = pos_time[thRiseInd]
    thMaxOvershoot = np.max(th_err[thRiseInd:])
    if np.any(th_err > THETA_THRESHOLD):
        thSettledInd = np.where(th_err > THETA_THRESHOLD)[0][-1]
        thSettlingTime = pos_time[thSettledInd]
        if thSettlingTime == pos_time[-1]:
            print("Theta Never settles!", th_err[-1])
            thSettlingTime = None
    else:
        print("Theta Never diverges!")
        thSettlingTime = 0

    if thSettlingTime is None:
        thRMSSettledErr = None
        print("No valid settled error to look at")
    else:
        thRMSSettledErr = np.linalg.norm(th_err[thSettledInd:]) / np.sqrt(
            th_err.shape[0] - thSettledInd
        )

    # Process gas usage metrics
    settling_time_candidates = [t for t in [thSettlingTime, posSettlingTime] if t is not None]
    settling_time = max(settling_time_candidates) if len(settling_time_candidates) != 0 else -1
    if settling_time != -1:
        settling_gas = (
            totalgaslist[np.where(np.array(metrics_time) > settling_time)[0][0]] - totalgaslist[0]
        )
    else:
        print("No valid settling time to use!")
        settling_gas = None

    print("Position Rise Time:", posRiseTime)
    print("Position Settling Time:", posSettlingTime, "out of", pos_time[-1])
    print("Position Settled Error:", posRMSSettledErr)
    print("Position Max Overshoot:", posMaxOvershoot)
    print("Theta Rise Time:", thRiseTime)
    print("Theta Settling Time:", thSettlingTime, "out of", pos_time[-1])
    print("Theta Settled Error:", thRMSSettledErr)
    print("Theta Max Overshoot:", thMaxOvershoot)
    print("Settling Gas Usage:", settling_gas)

    return (
        posRiseTime,
        posSettlingTime,
        posMaxOvershoot,
        posRMSSettledErr,
        thRiseTime,
        thSettlingTime,
        thMaxOvershoot,
        thRMSSettledErr,
        settling_gas,
    )


def main():
    average_stats = [[0] * NUM_STATS for i in range(len(EXPERIMENT))]
    std_stats = [[0] * NUM_STATS for i in range(len(EXPERIMENT))]
    min_stats = [[0] * NUM_STATS for i in range(len(EXPERIMENT))]
    max_stats = [[0] * NUM_STATS for i in range(len(EXPERIMENT))]

    for i in range(len(ROSBAG_NAMES)):
        print("====================== " + TITLES[i] + " ======================")
        fig, axs = None, None
        stats = [[] for i in range(NUM_STATS)]
        for j, ROSBAG_NAME in enumerate(ROSBAG_NAMES[i]):
            print("Working on " + ROSBAG_NAME)
            unpacked = unpack_rosbag(ROSBAG_NAME, EXPERIMENT[i])
            metrics_1, metrics_2 = extract_performance_metrics(unpacked, EXPERIMENT[i])
            for k in range(NUM_STATS):
                if metrics_1[k] is not None:
                    stats[k].append(metrics_1[k])
                if metrics_2[k] is not None:
                    stats[k].append(metrics_2[k])
            fig, axs = plot_experiment_results(unpacked, j, EXPERIMENT[i], fig, axs)
            # axs[0].vlines(posRiseTime, 0.4, 0.6) if posRiseTime is not None else print("hi")
            # axs[0].vlines(posSettlingTime, 0.4, 0.6) if posSettlingTime is not None else print("hi")
            # axs[2].vlines(thRiseTime, -2,2) if thRiseTime is not None else print("hi")
            # axs[2].vlines(thSettlingTime, -2,2) if thSettlingTime is not None else print("hi")
        fig.suptitle(TITLES[i])
        fig.legend(
            [
                "Trial 1 Actual",
                "Trial 1 Setpoint",
                "Trial 2 Actual",
                "Trial 2 Setpoint",
                "Trial 3 Actual",
                "Trial 3 Setpoint",
            ]
        )
        for k in range(NUM_STATS):
            average_stats[i][k] = np.around(np.mean(stats[k]), 5)
            std_stats[i][k] = np.around(np.std(stats[k]), 5)
            min_stats[i][k] = np.around(np.min(stats[k]), 5)
            max_stats[i][k] = np.around(np.max(stats[k]), 5)

    print(STATS)
    for i in range(len(EXPERIMENT)):
        print("====================== " + TITLES[i] + " ======================")
        print("Avg:", average_stats[i])
        print("StDev:", std_stats[i])
        print("Min:", min_stats[i])
        print("Max:", max_stats[i])
    plt.show()


if __name__ == "__main__":
    main()
