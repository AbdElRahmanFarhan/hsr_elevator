import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image

import pickle
from sklearn.svm import SVC
from sklearn.ensemble import RandomForestClassifier as RFC
from sklearn.model_selection import train_test_split
# from sklearn.metrics import plot_confusion_matrix, accuracy_score

from sensor_msgs.msg import Image

class circle_find:




    ## ref: https://towardsdatascience.com/non-maxima-suppression-139f7e00f0b5

    def NMS(self, boxes, overlapThresh = 0.02):
        # Return an empty list, if no boxes given
        if len(boxes) == 0:
            return []
        x1 = boxes[:, 0]  # x coordinate of the top-left corner
        y1 = boxes[:, 1]  # y coordinate of the top-left corner
        x2 = boxes[:, 2]  # x coordinate of the bottom-right corner
        y2 = boxes[:, 3]  # y coordinate of the bottom-right corner
        # Compute the area of the bounding boxes and sort the bounding
        # Boxes by the bottom-right y-coordinate of the bounding box
        areas = (x2 - x1 + 1) * (y2 - y1 + 1) # We add 1, because the pixel at the start as well as at the end counts
        # The indices of all boxes at start. We will redundant indices one by one.
        indices = np.arange(len(x1))
        for i,box in enumerate(boxes):
            # Create temporary indices  
            temp_indices = indices[indices!=i]
            # Find out the coordinates of the intersection box
            xx1 = np.maximum(box[0], boxes[temp_indices,0])
            yy1 = np.maximum(box[1], boxes[temp_indices,1])
            xx2 = np.minimum(box[2], boxes[temp_indices,2])
            yy2 = np.minimum(box[3], boxes[temp_indices,3])
            # Find out the width and the height of the intersection box
            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)
            # compute the ratio of overlap
            overlap = (w * h) / areas[temp_indices]
            # if the actual boungding box has an overlap bigger than treshold with any other box, remove it's index  
            if np.any(overlap) > overlapThresh:
                indices = indices[indices != i]
        #return only the boxes at the remaining indices
        return boxes[indices].astype(int)
    


    def image_processing(self, img):
        img_copy = img.copy()
        original = img.copy()

        boxes = np.array([1, 2, 3, 4])
        individual_images = []
        bounding_boxes_data = []   # x , y, w, h
        ROI_num = 0

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        bilateral_filtered_image = cv2.bilateralFilter(gray, 5, 175, 175)
        cv2.imwrite('b-filtered.png', bilateral_filtered_image)
        # bilateral_filtered_image = gray
        edge_detected_image = cv2.Canny(bilateral_filtered_image, 75, 200)
        # cv2.imshow('edge_detected_image', edge_detected_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.imshow('edge_detected', edge_detected_image)

        contours, hier = cv2.findContours(edge_detected_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contour_list = []

        for contour in contours:
            approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
            area = cv2.contourArea(contour)
            if ((len(approx) > 6) & (len(approx) < 25) & (area > 20) ): # Here we are checking for the edges more than 8 and less than 20
                contour_list.append(contour)
                

        for contour in contour_list:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 15 and h > 15:
                boxes = np.vstack((boxes, [x, y, x+w, y+h]))

        boxes = np.delete(boxes, 0, 0)
        # print(boxes)
        remaining_boxes = self.NMS(boxes)
        # print(f"remaining boxes = \n{remaining_boxes}")

        for box in remaining_boxes:
            x1, y1 = box[0], box[1]
            x2, y2 = box[2], box[3]
            rec_start_point = (x1, y1)
            rec_end_point = (x2, y2)
            cv2.rectangle(img, rec_start_point, rec_end_point, (255, 0, 0), 1)
            ROI = original[y1:y2, x1:x2]
            individual_images.append(ROI)
            bounding_boxes_data.append([x1, y1, x2-x1, y2-y1])
            # bounding_boxes_data.append([x1, y1, x2, y2])
            string = str(ROI_num) + ".png"
            cv2.imwrite(string, ROI)
            ROI_num += 1


        # cv2.imshow('img', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return bounding_boxes_data, individual_images
