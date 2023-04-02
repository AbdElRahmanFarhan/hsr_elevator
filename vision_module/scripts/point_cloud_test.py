#!/usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import Image, PointCloud2
import tf
import rospkg
# from mas_perception_libs.utils import transform_cloud_with_listener

rospy.init_node("test")
# pkg = rospkg.RosPack().get_path("vision_module")
bag = rosbag.Bag('6.bag')
pc_pub = rospy.Publisher("/habibi/pc", PointCloud2, queue_size=1)
img_pub = rospy.Publisher("habibi/img", Image, queue_size=1)
listener = tf.TransformListener()


for topic, msg, t in bag.read_messages(topics=['/hsrb/head_rgbd_sensor/rgb/image_raw', '/hsrb/head_rgbd_sensor/depth_registered/rectified_points']):
    if topic == '/hsrb/head_rgbd_sensor/rgb/image_raw':
        img_msg = msg
        
    else:
        pc_msg = msg
        
while not rospy.is_shutdown():
    pc_msg.header.stamp = rospy.Time().now()
    img_msg.header.stamp = rospy.Time().now()
    # pc_msg = transform_cloud_with_listener(pc_msg, "base_footprint", listener)
    pc_pub.publish(pc_msg)   
    img_pub.publish(img_msg)
    rospy.sleep(0.1)
bag.close()
