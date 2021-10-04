"""Make the car circle around coordinate (0, 0).

Installing dependencies: pip3 install dubins squaternion attrs

Usage:
- Running in simulator: python3 draw_circle.py simulate
- Running on car: python3 draw_circle.py car
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescCtrlStamped

import sys
import time
import numpy as np
import dubins
from squaternion import Quaternion


class CircleDriver:
    def __init__(self, pose_topic, drive_topic, center, target_radius, target_speed, dt, horizon):
        if "odom" in pose_topic:
            self.odom_sub = rospy.Subscriber(pose_topic, Odometry, self.odom_callback, queue_size=1)
        else:
            self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, VescCtrlStamped, queue_size=1)
        self.ack = AckermannDriveStamped()
        self.pose_topic = pose_topic
        self.center = center
        self.target_radius = target_radius
        self.target_speed = target_speed*10000
        self.dt = dt
        self.horizon = horizon
        self.initialized = False

        # make waypoints on the circle
        n_points = int(2 * np.pi * target_radius / (dt * target_speed))
        delta_angle = 2 * np.pi / n_points
        waypoints = np.zeros((n_points, 3))
        for i in range(n_points):
            angle = i * delta_angle
            psi = angle + np.pi / 2
            if psi > np.pi:
                psi -= 2 * np.pi
            x = center[0] + target_radius * np.cos(angle)
            y = center[1] + target_radius * np.sin(angle)
            waypoints[i] = [x, y, psi]
        self.waypoints = waypoints


    def make_ref(self, x0, y0, psi0):
        ref = np.zeros((self.horizon, 3))

        # navigate to the furtherest waypoint using Dubins path
        initial_xy = np.array([x0, y0]).reshape((1, 2))
        waypoint_xy = self.waypoints[:, :2]
        distances = np.sqrt(np.sum((waypoint_xy - initial_xy) ** 2, axis=1))
        furthest_idx = np.argmax(distances)
        q0 = (x0, y0, psi0)
        q1 = tuple(self.waypoints[furthest_idx])
        dubins_path = dubins.shortest_path(q0, q1, self.target_radius)
        configurations, _ = dubins_path.sample_many(self.dt * self.target_speed)
        for i in range(min(len(configurations), self.horizon)):
            ref[i, :] = configurations[i]
            if ref[i, 2] > np.pi:
                ref[i, 2] -= 2 * np.pi
        if len(configurations) < self.horizon:
            for i in range(self.horizon - len(configurations)):
                ref[i + len(configurations), :] = self.waypoints[(furthest_idx + i) % len(self.waypoints)]
        return ref

    def odom_callback(self, msg):
        self.pose_callback(msg.pose)

    def pose_callback(self, msg):
        # read state
        x = msg.pose.position.x
        y = msg.pose.position.y
        ori = msg.pose.orientation
        q = [ori.w, ori.x, ori.y, ori.z]
        _, _, psi = Quaternion(*q).to_euler()
        print(x, y, psi)
        self.update_control(x, y, psi)

    def update_control(self, x, y, psi, mpc_solver=[]):
        # initialize MPC on first call
        if not mpc_solver:
            from julia import Main
            Main.include("mpc.jl")
            mpc_step = Main.mpc_step
            mpc_solver.append(mpc_step)
            mpc_step(np.zeros((3,)), np.zeros((self.horizon, 3)))
            print("MPC initialized")

        # update reference trajectory
        ref = self.make_ref(x, y, psi)
        print(ref)

        # mpc step
        state = np.array([x, y, psi])
        delta, v = mpc_solver[0](state, ref)
        print(delta, v)
        # print(x + self.dt * v * np.cos(delta + psi), y + self.dt * v * np.sin(delta + psi), psi + self.dt * np.sin(delta) / 0.5)

        # publish control input
        self.ack.drive.steering_angle = delta
        self.ack.drive.speed = v
        self.drive_pub.publish(self.ack)

        time.sleep(0.1)

rospy.init_node('Auto')

mode = sys.argv[-1]
if mode == "simulate":
    pose_topic = "/odom"
    drive_topic = "/drive"
elif mode == "car":
    pose_topic = "/vrpn_client_node/car/pose"
    drive_topic = "/vesc/ctrl"

# dt and horizon should be the same as in mpc.jl
driver = CircleDriver(pose_topic, drive_topic, center=(0., 0.), target_radius=1, target_speed=0.5, dt=0.1, horizon=5)
# driver.update_control(0, 0, 0)

rospy.spin()
