#!/usr/bin/env python3

from copy import deepcopy
from math import pi
import sys
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TwistStamped, WrenchStamped, PoseStamped, Point, Quaternion
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown, os
import rospy


class Arm:
    def __init__(self):
        self.transformer_listener = tf.TransformListener()
        self.arm = MoveGroupCommander("arm", wait_for_servers=0.0)
        self.arm.set_max_acceleration_scaling_factor(0.2)
        self.arm.set_max_velocity_scaling_factor(0.2)
        self.planning_frame = "base_link"
        self.arm.set_pose_reference_frame(self.planning_frame)
        self.x_const = 0.005
        self.x_vel = 0.01
        self.y_const = 0.07
        self.z_const = 0.673
        self.q = [0.707, 0.0, 0.707, 0.0]

        self.base = MoveGroupCommander("base", wait_for_servers=0.)
        self.base.set_max_acceleration_scaling_factor(0.2)
        self.base.set_max_velocity_scaling_factor(0.2)
        
        self.head = MoveGroupCommander("head",  wait_for_servers=0.0)
        #self.gripper = MoveGroupCommander('gripper', wait_for_servers=0.0)
        self.go_to_home()
        self.cartesian_vel_pub = rospy.Publisher("/arm_1/arm_controller/cartesian_velocity_command", TwistStamped, queue_size=1)
        rospy.Subscriber("/hsrb/wrist_wrench/compensated", WrenchStamped, self.wrench_cb)
        self.current_wrench = WrenchStamped()
        self.force_thresh = 6.
        rospy.sleep(1)
        rospy.Subscriber("button_pose", PoseStamped, self.pose_cb)
        rospy.sleep(1)
        print("Start")
    
    def go_to_home(self):
        self.arm_target = {'arm_lift_joint': 0.0,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -pi/2.,
            'wrist_roll_joint': 0.,
            'wrist_ft_sensor_frame_joint': 0.0}

        head_target = {
            'head_pan_joint': 0.0,
            'head_tilt_joint': -0.200000}
        
        self.arm.set_joint_value_target(self.arm_target)
        self.arm.go()
        self.head.set_joint_value_target(head_target)
        self.head.go()
        #self.gripper.set_joint_value_target("hand_motor_joint", -0.2)
        #self.gripper.go()

    def wrench_cb(self, msg):
        self.current_wrench = msg
        
    def pose_cb(self, button_pose):
        self.transformer_listener.waitForTransform(self.planning_frame, button_pose.header.frame_id, button_pose.header.stamp, rospy.Duration(1))
        button_pose = self.transformer_listener.transformPose(self.planning_frame, button_pose)
        
        y_base = button_pose.pose.position.y - self.y_const
        base_pose = PoseStamped()
        base_pose.header.frame_id = "base_footprint"
        base_pose.pose.position = Point(0.0, y_base, 0.)
        base_pose.pose.orientation = Quaternion(0., 0., 0., 1.)
        rospy.sleep(2)
        self.transformer_listener.waitForTransform("odom", base_pose.header.frame_id, rospy.Time(), rospy.Duration(1))
        base_pose = self.transformer_listener.transformPose("odom", base_pose)
        _, _, theta = euler_from_quaternion([base_pose.pose.orientation.x, base_pose.pose.orientation.y, base_pose.pose.orientation.z, base_pose.pose.orientation.w])
        print(self.base.get_current_pose())
        print([base_pose.pose.position.x, base_pose.pose.position.y, theta])
        self.base.set_joint_value_target("world_joint", [base_pose.pose.position.x, base_pose.pose.position.y, theta])
        self.base.go()
        
        # move the robot close to the pose goal
        print("Start Planning")
        print(button_pose)
        arm_target =  {'arm_lift_joint': button_pose.pose.position.z - self.z_const,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -pi/2.,
            'wrist_roll_joint': 0.,
            'wrist_ft_sensor_frame_joint': 0.0}
        self.arm.set_joint_value_target(arm_target)
        result = self.arm.go()
        # move the arm strait until it touches the button
        if result:
            print("Moving to touch the button")
            rate = rospy.Rate(20)
            msg = TwistStamped()
            msg.header.frame_id = "base_link"
            while self.current_wrench.wrench.force.z < self.force_thresh:
                msg.header.stamp = rospy.Time().now()
                msg.twist.linear.x = self.x_vel
                self.cartesian_vel_pub.publish(msg)
                rate.sleep()
            print("touched")
            # stop moving when the arm touches the button
            msg.twist.linear.x = 0.0
            self.cartesian_vel_pub.publish(msg)
            rospy.sleep(3)
            # retreat the arm in straght line
            print("retreating")
            msg.twist.linear.x = -self.x_vel
            for i in range(500):
                self.cartesian_vel_pub.publish(msg)
                rospy.sleep(0.01)
            msg.twist.linear.x = 0.0
            self.cartesian_vel_pub.publish(msg)
            # go to the home position
            print("going home")
            rospy.sleep(2)
            self.go_to_home()
                    
                    
if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node("moveit_demo", anonymous=True)
    Arm()
    rospy.spin()
    roscpp_shutdown()
    os._exit(0)
    


    
    
    
    
    
