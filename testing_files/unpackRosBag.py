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
import matplotlib.pyplot as plt
import math

EXPERIMENT_MODE = 0 # 0: Turnaround, 1:ShortDist, 2:LongDist
START_POINT = [0.5, 0.5, -math.pi/2]
ROSBAG_NAME = '/Users/andrewwang/Downloads/ROSBagProcessing/testexp_pd_turnaround/'
FIRST_GOAL = None

if EXPERIMENT_MODE == 0:
    FIRST_GOAL = [0.5, 0.5, math.pi/2]
elif EXPERIMENT_MODE == 1:
    FIRST_GOAL = [1, 1, -math.pi/2]
elif EXPERIMENT_MODE == 2:
    FIRST_GOAL = [3, 2, -math.pi/2]
else:
    print("ERR: Invalid Experiment mode!")


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
    
    return roll_x, pitch_y, yaw_z # in radians

def plot_experiment_results(pos_time, xlist, ylist, thlist,
                            goal_time, goalxlist, goalylist, goalthlist,
                            metrics_time, totalgaslist):
    fig, axs = plt.subplots(3, 1, constrained_layout=True, sharex = True)
    axs[0].plot(pos_time, xlist)
    axs[0].plot(goal_time, goalxlist)
    axs[0].set_ylabel("X-Position (m)")
    axs[0].hlines(0.5, min(pos_time), max(pos_time), colors='k', linestyles='dashed')
    axs[0].minorticks_on()
    axs[0].grid(True, which='both', axis='y')
    axs[0].legend(["Actual","Goal"])

    axs[1].plot(pos_time, ylist)
    axs[1].plot(goal_time, goalylist)
    axs[1].set_ylabel("Y-Position (m)")
    axs[1].hlines(0.5, min(pos_time), max(pos_time), colors='k', linestyles='dashed')
    axs[1].minorticks_on()
    axs[1].grid(True, which='both', axis='y')
    axs[1].legend(["Actual","Goal"])

    axs[2].plot(pos_time, thlist)
    axs[2].plot(goal_time, goalthlist)
    axs[2].set_ylabel("Orientation (rad)")
    axs[2].minorticks_on()
    axs[2].grid(True, which='both', axis='y')
    axs[2].legend(["Actual","Goal"])

    plt.show()


def main():
    t0 = 0
    experiment_start = False

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

    # create reader instance and open for reading
    with Reader(ROSBAG_NAME) as reader:
        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)
        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            if (experiment_start):
                try:
                    if connection.topic == '/vrpn_mocap/robot/pose':
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        pos_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0)
                        xlist.append(msg.pose.position.x)
                        ylist.append(msg.pose.position.y)
                        th = quat2euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
                        thlist.append(th[2]) # Yaw
                    elif connection.topic == '/robot/robot/goal':
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        goal_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0)
                        goalxlist.append(msg.pose.position.x)
                        goalylist.append(msg.pose.position.y)
                        th = quat2euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
                        goalthlist.append(th[2]) # Yaw     
                    elif connection.topic == '/robot/controller/metrics':
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        metrics_time.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0)
                        totalgas.append(msg.total_gas_time)
                except KeyError:
                    print("Err: Not ready to process these messages yet!")
            else:
                if connection.topic == '/robot/robot/goal':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    pos = [msg.pose.position.x, msg.pose.position.y, quat2euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)[2]]
                    if (pos == FIRST_GOAL):
                        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                        experiment_start = True
                        goal_time.append(0)
                        goalxlist.append(pos[0])
                        goalylist.append(pos[1])
                        goalthlist.append(pos[2]) # Yaw 
                
    
    
    plot_experiment_results(pos_time, xlist, ylist, thlist, goal_time, goalxlist, goalylist, goalthlist, metrics_time, totalgaslist)

if __name__ == "__main__":
    main()
