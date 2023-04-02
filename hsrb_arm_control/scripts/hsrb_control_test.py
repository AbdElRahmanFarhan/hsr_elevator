#!/usr/bin/env python3

from copy import deepcopy
from math import pi
import sys

from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown, os
import rospy


rospy.init_node("moveit_demo", anonymous=True)
pub = rospy.Publisher("button_pose", PoseStamped, queue_size=1)
rospy.sleep(1)

msg = PoseStamped()
msg.header.frame_id = "base_link"
msg.header.stamp = rospy.Time().now()
T = [0.5, 0.078, 0.9]
msg.pose.position.x = T[0]
msg.pose.position.y = T[1]
msg.pose.position.z = T[2]

msg.pose.orientation.x = 0.7071041547445843
msg.pose.orientation.y = -0.00023536349309100706
msg.pose.orientation.z = 0.707109292943619
msg.pose.orientation.w = 0.00032677170748223686

pub.publish(msg)
rospy.sleep(1)