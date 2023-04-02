#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Point, Quaternion
import cv2
import tf2_ros
import pickle
# import open3d
import torch.nn as nn
import torchvision.transforms as transforms
import torchvision.models as models
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import math
from mas_perception_libs.utils import crop_cloud_to_xyz, BoundingBox2D, crop_organized_cloud_msg, cloud_msg_to_ndarray
from mas_perception_libs.visualization import draw_labeled_boxes
from cv_bridge import CvBridge, CvBridgeError
from circle_find import circle_find
import numpy as np
import sys
from std_msgs.msg import Header
from sklearn.impute import SimpleImputer 
import rospkg
from sklearn.svm import SVC
from skimage.feature import hog
from keras.models import load_model
from model import ElevatorButtonClassifier
import torch
import PIL
from roboflow import Roboflow

# np.set_printoptions(threshold=sys.maxsize)
bridge = CvBridge()

#defining parameters and extract hog function 
orientations = 9
pixels_per_cell = (8, 8)
cells_per_block = (3, 3)
visualize = True
transform_sqrt = True

# define a function to extract HOG features from an image
def extract_hog_features(image):
    fd, _ = hog(image, orientations=orientations, pixels_per_cell=pixels_per_cell,
                        cells_per_block=cells_per_block, visualize=visualize, transform_sqrt=transform_sqrt,
                        block_norm='L2-Hys', feature_vector=True)
    return fd
######################################################

class button_detector:

    def __init__(self):
        # self.camera_sub_topic = "/hsrb/head_rgbd_sensor/rgb/image_raw"
        # self.point_sub_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"
        
        self.camera_sub_topic = "/habibi/img"
        self.point_sub_topic = "/habibi/pc"
        self.test_cloud_pub = rospy.Publisher("habibi/test_cloud", PointCloud2, queue_size=1)
        self.button_pub = rospy.Publisher("button_pose", PoseStamped, queue_size=1)
        self.transformer_broadcaster = tf2_ros.StaticTransformBroadcaster()
        pkg = rospkg.RosPack().get_path("vision_module")
        # path = pkg + '/classification_models/SVM_linear.pkl'
        # self.button_classifier = pickle.load(open(path, 'rb'))
        path = pkg + '/classification_models/cnn.h5'
        self.button_classifier = load_model(path)
        
        # path = pkg + '/classification_models/my_model.pt'
        # self.button_classifier = models.resnet18(pretrained=True)
        # ft = self.button_classifier.fc.in_features
        # self.button_classifier.fc = nn.Linear(ft, 5)
        # self.button_classifier.load_state_dict(torch.load(path))
        # self.button_classifier.eval()
        # device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        # self.button_classifier.to(device)
        # self.transform = transforms.Compose([transforms.Resize((224, 224)),
        #                                 transforms.ToTensor(),
        #                                 transforms.Normalize(mean=[0.5, 0.5, 0.5],std=[0.5, 0.5, 0.5])
        #                                 ])
        # rf = Roboflow(api_key="9lGfAwlMtXmAIyO2JVYs")
        # project = rf.workspace().project("elevator-buttons-scpv6")
        # self.button_classifier = project.version(2).model
        self.transformer_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
    def create_frame(self, new_frame_id, ps):
        
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = ps.header.frame_id
        static_transformStamped.child_frame_id = new_frame_id

        static_transformStamped.transform.translation.x = ps.pose.position.x
        static_transformStamped.transform.translation.y = ps.pose.position.y
        static_transformStamped.transform.translation.z = ps.pose.position.z

        static_transformStamped.transform.rotation.x = ps.pose.orientation.x
        static_transformStamped.transform.rotation.y = ps.pose.orientation.y
        static_transformStamped.transform.rotation.z = ps.pose.orientation.z
        static_transformStamped.transform.rotation.w = ps.pose.orientation.w
        self.transformer_broadcaster.sendTransform(static_transformStamped)

    def bounding_boxes_poses(self, bounding_boxes, pred, point_cloud):
        buttons_dict = {}
        poses = []
        counter = 0
        for boxes, pred_button_label in zip(bounding_boxes, pred):

            label = (pred_button_label)
            for i in range(len(boxes)):
                boxes[i] = float(boxes[i])
            bbox = (boxes[0], boxes[1], boxes[2], boxes[3])
            color = (0,0,255)
            bounding_box = BoundingBox2D(label, color, bbox)
            test_cloud = crop_organized_cloud_msg(point_cloud, bounding_box)
            obj_coords = cloud_msg_to_ndarray(test_cloud, fields=['x', 'y', 'z'])
            mean_coords = np.nanmean(obj_coords.reshape(-1, 3), axis=0)
            mean_ps = PoseStamped()
            mean_ps.header = Header(frame_id=point_cloud.header.frame_id)
            mean_ps.pose = Pose(Point(mean_coords[0], mean_coords[1], mean_coords[2]), Quaternion(0., 0., 0., 1.))
            button_name = label
            buttons_dict[button_name] = mean_ps
            # self.create_frame(button_name, mean_ps)
            # rospy.sleep(4)
            counter +=1
        return buttons_dict
        
    def detect_buttons(self):
        print("Press enter to detect")
        input()
        img = rospy.wait_for_message(self.camera_sub_topic, Image)
        point_cloud = rospy.wait_for_message(self.point_sub_topic, PointCloud2)
        
        # detect the bounding boxes
        rgb_img = bridge.imgmsg_to_cv2(img, "bgr8")
        circle_finder = circle_find()
        bounding_boxes, buttons = circle_finder.image_processing(rgb_img)
        # ones  = np.array([cv2.resize(one , (28,28)) for one in buttons])
        # ones  = np.array([cv2.cvtColor(one, cv2.COLOR_RGB2GRAY) for one in ones])
        # ones = np.array([extract_hog_features(one) for one in ones])
        # # self.button_classifier.probability = False
        # preds = self.button_classifier.predict(ones) 
        # print(preds)
        

        # self.button_classifier.probability = False
        # cv2.imwrite("test.png", rgb_img)
        # self.button_classifier.predict("test.png", confidence=80, overlap=10).save(("result.jpg"))
        # preds = self.button_classifier.predict(rgb_img, confidence=80, overlap=10).json()["predictions"]
        # # print(len(preds))
        # bounding_boxes = []
        # pred = []
        # for p in preds:
        #     bounding_boxes.append([p['x'], p['y'], p['width'], p['height']])
        #     pred.append(p['class'])
        # print(pred)
        # classify the bounding boxes
        buttons  = np.array([cv2.resize(button , (28,28)) for button in buttons])
        preds = self.button_classifier.predict(buttons)
        
        # with torch.no_grad():
        #     tensors = torch.zeros((len(buttons), 3, 224, 224), dtype=torch.float32)
        #     for i, button in enumerate(buttons):
        #         tensors[i, :, :, :] = self.transform(PIL.Image.fromarray(button))
        #     preds = self.button_classifier(tensors)
        #     preds = preds.cpu().detach().numpy()
        #     print(preds)
            
        # dic ={0: '1', 1: '2', 2: 'e', 3: 'oc', 4: 'bell', 5: 'empty', 6:'key'}
        dic ={0: '1', 1: '2', 2: 'e', 3: 'oc', 4: 'bell', 5: 'empty'}

        pred = [dic[key] for key in np.argmax(preds, axis=1)]       
        print(pred)
               
        # # # get the poses of the bouding boxes
        buttons_dict = self.bounding_boxes_poses(bounding_boxes, pred, point_cloud)
        print("Choose the button to press 1, 2 or e")
        button = input()
        pose = buttons_dict[button]
        self.button_pub.publish(pose)
        
            
        
if __name__=='__main__':
    node_name = "button_detection"
    rospy.init_node(node_name)
    bd = button_detector()
    while not rospy.is_shutdown():

        bd.detect_buttons()
        
