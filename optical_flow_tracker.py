#%%
import cv2 as cv
import numpy as np
#import matplotlib.pyplot as plt
from cv_bridge import CvBridge
#from rospy.msg import apriltagmsg
# Find the best points potentially a feature.
tag_img = cv.imread('apriltag-36h11-id12.png')
tag_img = tag_img[:tag_img.shape[1]-40,:]
# cv.imshow('tag',tag_img)
#%%
gray = cv.cvtColor(tag_img, cv.COLOR_BGR2GRAY)
# cv.imshow('Gray',gray)

edged = cv.Canny(gray, 50, 100)
# cv.imshow('edged',edged)

contour = edged.copy()
cntr,hierarchy = cv.findContours(contour, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)

cv.drawContours(edged, cntr, -1, (255,255,0), 10)
cv.imshow('Contour Drawn',edged)
#print(cntr)

cv.waitKey(0)
#plt.scatter(cntr[0][:,:,0],cntr[0][:,:,1],s=25,c ='r',marker='*') 
#plt.show()
cv.destroyAllWindows()

# Apply Lucas Kannade to track those points
# class motion_estimate():
#     def __init__(self, topic):
#         self.prev_img = None
#         self.topic = topic

#         self.bridge = CvBridge()
#         self.old_t = 0
#         self.motion_est = rospy.Publisher('motion_est',msg, queue_size=10)
#         self.tag_msg - rospy.Subscriber(self.topic, self.motionest_callback)

#         # Lucas Kanade Optic Flow parameters
#         self.lk_params = dict( winSize  = (15,15),
#                                maxLevel = 2,
#                                criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

#     def motionest_callback(self, img):
#         curr_img = self.bridge.imgmsg_to_cv2(img, desired_encoding="mono8")
#         curr_t = float(img.header.stamp.secs) + float(img.header.stamp.nsecs)*1e-9

#         pt2trck_newpose, stat,err = cv.calcOpticalFlowPyrLK(self.prev_pt, curr_img, pt2trck, None, **self.lk_params)

#         dt = curr_t -self.old_t

#         self.old_t = curr_t
#         self.prev_img = img

