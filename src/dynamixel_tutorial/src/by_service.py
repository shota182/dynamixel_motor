#!/usr/bin/env python3
# -*- coding: utf-8 -*

import math
import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Service制御
def degToStep(deg):
    return int((deg+180)/360.0*4095)

def setPosition(motor_id, position):
    # Service Clientの定義
    motor_client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
    
    # degreesを4096段階のstep(12bit)に変換
    step = degToStep(position)
    
    # 許容電圧を指定
    motor_client('', motor_id, 'Goal_Current', 200)
    # 目標角度を指定
    motor_client('', motor_id, 'Goal_Position', step)
    
if __name__ == '__main__':
    rospy.init_node('dynamixel_by_service')
    setPosition(0, 30)
    rospy.sleep(2.0)
    setPosition(1, 30)
    rospy.sleep(2.0)
    setPosition(0, 0)
    setPosition(1, 0)