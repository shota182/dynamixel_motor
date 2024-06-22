#!/usr/bin/env python3
# -*- coding: utf-8 -*

import math
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def degToRad(deg):
    return math.radians(deg)

def motorPub(joint_name, joint_angle, execute_time=1.0):
    # Publisherの定義
    motor_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=10)
    rospy.sleep(0.5)
    
    # publishするデータの定義
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = joint_name
    msg.points = [JointTrajectoryPoint()]
    msg.points[0].positions = list(map(degToRad, joint_angle))
    # msg.points[0].velocities = list(map(degToRad, joint_angle))
    # msg.points[0].accelerations = list(map(degToRad, joint_angle))
    # msg.points[0].effort = list(map(degToRad, joint_angle))
    msg.points[0].time_from_start = rospy.Time(execute_time)
    # publish
    motor_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('dynamixel_by_topic')
    try:
        while True:
            motorPub(['one', "four"], [300, 100])
            rospy.sleep(2.0)
            motorPub(['two', 'three'], [300, 300])
            rospy.sleep(2.0)
            motorPub(['one', 'two', "three", "four"], [0, 0, 0, -300])
            rospy.sleep(2.0)
    except KeyboardInterrupt:
        motorPub(['one', 'two', "three", "four"], [0, 0, 0, 0])