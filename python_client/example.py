#!/usr/bin/env python
# -*- coding:utf-8 -*-

import random
import math
import rospy
import tf
from tf.transformations import quaternion_from_euler, quaternion_multiply

from sensor_msgs.msg import JointState
import std_msgs.msg
import geometry_msgs.msg
import kimm_joint_planner_ros_interface.srv
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

RED = std_msgs.msg.ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 1.0)
WHITE = std_msgs.msg.ColorRGBA(1.0, 1.0, 1.0, 1.0)
t_BLUE = std_msgs.msg.ColorRGBA(0.0, 0.0, 1.0, 0.5)

DEFAULT_QUAT = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)


def _create_marker_init():
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time()
    marker.ns = "my_namespace"
    marker.id = random.randint(0, 2048)
    return marker


def create_marker(position, orientation, scale, color_rgba, shape):
    # http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
    marker = _create_marker_init()
    marker.type = shape
    marker.action = marker.ADD
    marker.color = color_rgba
    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale = scale
    return marker


def create_line(point_list, color_rgba):
    marker = _create_marker_init()
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.color = color_rgba
    marker.pose.position = geometry_msgs.msg.Point(0, 0, 0)
    marker.pose.orientation = DEFAULT_QUAT
    marker.points = [geometry_msgs.msg.Point(p[0], p[1], p[2]) for p in point_list]
    marker.scale = geometry_msgs.msg.Vector3(0.005, 0, 0)
    return marker


def joint_trajectory_deg2rad(joint_trajectory):
    for p in joint_trajectory.points:
        p.positions = [math.radians(t) for t in p.positions]
    return joint_trajectory


class Example:
    def __init__(self):
        self.listener = tf.TransformListener()
        rospy.wait_for_service("ns0/kimm_joint_planner_ros_interface_server/plan_joint_path")
        self.plan_joint_motion = rospy.ServiceProxy(
            "ns0/kimm_joint_planner_ros_interface_server/plan_joint_path",
            kimm_joint_planner_ros_interface.srv.plan_joint_path,
        )
        self.req = kimm_joint_planner_ros_interface.srv.plan_joint_pathRequest()
        self.resp = kimm_joint_planner_ros_interface.srv.plan_joint_pathResponse()

    def test(self):
#         sensor_msgs/JointState current_joint
# sensor_msgs/JointState[] target_joint
# float64 duration
# int16 traj_type
# float64[] kp
# float64[] kv
# std_msgs/Bool[] mask
# float64 vel_limit
# float64 acc_limit

        c_joint = JointState()
        c_joint.position = np.array([0, 0, 0, 0, 0, 0])
        self.req.current_joint = c_joint
        self.req.target_joint = []
        t_joint = JointState()
        t_joint.position = np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
        self.req.target_joint.append(t_joint)

        self.req.duration = 0.1
        self.req.traj_type = 1
        kp = []
        kv = []
        self.req.mask = []
        for i in range(0,6):
            kp.append(400.0)
            kv.append(40.0)
            bool_tmp = std_msgs.msg.Bool()
            bool_tmp.data = True
            self.req.mask.append(bool_tmp)

        self.req.kp = kp
        self.req.kv = kv
        self.req.mask[0].data = True
        self.req.mask[1].data = True
        self.req.mask[2].data = True
        self.req.mask[3].data = True
        self.req.mask[4].data = True
        self.req.mask[5].data = True
        self.req.vel_limit = 1.0
        self.req.acc_limit = 1.0      
    
        try:
            self.resp = self.plan_joint_motion(self.req)
            for i in range(len(self.resp.res_traj)):
                print(self.resp.res_traj[i].position)
            rospy.logwarn("========== Done! ==========")
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

if __name__ == "__main__":
    rospy.init_node("Example")
    ex = Example()
    ex.test()

    rospy.spin()
