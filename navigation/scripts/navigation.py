#!/usr/bin/env python3

from copy import deepcopy
from math import pi
import sys
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
import rospy


rospy.init_node("navigation", anonymous=True)
pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)
rospy.sleep(1)
rospy.set_param("/tmc_map_merger/inputs/base_scan/enable", 1)
rospy.set_param("/tmc_map_merger/inputs/head_rgbd_sensor/enable", 0)
rospy.set_param("/tmc_map_merger/inputs/base_scan/obstacle_circle/forbid_radius", 0.1)
rospy.set_param("/tmc_map_merger/inputs/base_scan/obstacle_circle/obstacle_occupancy", 10)
rospy.set_param("/tmc_map_merger/inputs/base_scan/obstacle_circle/obstacle_radius", 0.1)
# rospy.set_param("/tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle/forbid_radius", 0.1)
# rospy.set_param("/tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle/obstacle_occupancy", 5)
# rospy.set_param("/tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle/obstacle_radius", 0.1)
rospy.sleep(1)

in_goal = PoseStamped()
in_goal.header.frame_id ="map"
in_goal.pose.position = Point( 0.10304762469478401, 0.16109864578751412, 0.)
in_goal.pose.orientation = Quaternion(0., 0., 0.11890534084051745, 0.9929055946662806)

out_goal = PoseStamped()
out_goal.header.frame_id ="odom"
out_goal.pose.position = Point(0., 0., 0.)
out_goal.pose.orientation = Quaternion(0., 0. ,0., 1.)

print("if the robot is localizaed press enter")
input()
while not rospy.is_shutdown():
    print("please enter i for inside and o for outside")
    c = input()
    if c == "i":
        pub.publish(in_goal)
    elif c == "o":
        pub.publish(out_goal)
    else:
        print("please enter valid character")
