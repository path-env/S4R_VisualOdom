#!/usr/bin/python3
import cv2 as cv
import numpy as np
import rospy
import matplotlib.pyplot as plt
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image

plt.ion()

class motion_estimate():
    def __init__(self, topic):

        self.prev_img = None
        self.topic = topic

        self.bridge = CvBridge()
        self.old_t = 0
        
        self.tag_msg = rospy.Subscriber(self.topic,Image, self.motionest_callback)

        self.lk_params = { 'winSize'  :(20,20),
                           'maxLevel' :3,
                           'criteria' : (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03)}
        self.flow = []
    
    def point_to_track(self, image, spacing):
        ''' Function to select equispaced random  points to apply Lucas Kanade  '''
        
        p2tr = []
        for x in range(0,image.shape[0],spacing):
            for y in range(0,image.shape[1],spacing):
                new_point = [y, x]
                p2tr.append(new_point)
        p2tr = np.array(p2tr, dtype=np.float32) 
        p2tr = p2tr.reshape(p2tr.shape[0], 1, p2tr.shape[1]) 
        return p2tr

    def edge_detector(self,image):
        ''' function to selecct the edge in the image to apply lucas kanade'''
        edged = cv.Canny(image, 50, 100)
        contour = edged.copy()
        cntr,hierarchy = cv.findContours(contour, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        cntr = np.array(cntr[0], dtype=np.float32)
        return cntr

    def create_ofd(self,gray_image, points, flow):
        ''' Draw points on the image on which lucas kanade is to be applied '''
        img = cv.cvtColor(gray_image, cv.COLOR_GRAY2BGR)
        red = [0,0,255] 
        width = 1
        for i, point in enumerate(points):
            x = point[0,0]
            y = point[0,1]
            vx = flow[i][0,0]
            vy = flow[i][0,1]
            cv.line(img, (x,y), (x+vx, y+vy), red, width) 
        
        cv.imshow('optic_flow_field',img)
        cv.waitKey(1)

    def motionest_callback(self, img):
        
        ''' callback function for the topic /rover/raspicam/image_color'''
        
        curr_img = self.bridge.imgmsg_to_cv2(img, desired_encoding="mono8")
        if self.prev_img is None:
                self.prev_img = curr_img
                self.pt2trck = self.point_to_track(curr_img, 200)
                return 
        pt2trck_newpose, stat,err = cv.calcOpticalFlowPyrLK(self.prev_img, curr_img, self.pt2trck, None, **self.lk_params)
        flow= pt2trck_newpose - self.pt2trck
        self.create_ofd( curr_img, self.pt2trck, flow)
        self.flow = flow.reshape(-1,2)
        self.prev_img = curr_img
        
    def get_flow(self):
        return self.flow
    

