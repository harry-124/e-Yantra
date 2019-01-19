#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy, cv2, cv_bridge
from plutodrone.msg import *
from pid_tune.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64

class detect():

	def __init__(self):

		rospy.init_node('ros_bridge')
		
		# Create a ROS Bridge
		self.ros_bridge =CvBridge()

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
	
	def image_callback(self,msg):

		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		ll_g = np.array([33,75,100])
                ul_g = np.array([102,255,255])

		ll_b = np.array([110,100,100])
                ul_b = np.array([130,255,255])

		ll_r = np.array([0,100,100])
                ul_r = np.array([10,125,175])


		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask_g = cv2.inRange(hsv, ll_g, ul_g)
		res_g = cv2.bitwise_or(hsv,hsv, mask=mask_g)
		gray_g = cv2.cvtColor(res_g,cv2.COLOR_BGR2GRAY)
		ret,gray_t_g = cv2.threshold(gray_g,10,255,0)
     		contours_g = cv2.findContours(gray_t_g,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]
		x,y,w,h = cv2.boundingRect(contours_g)
		img = cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),-1)
		
		mask_b = cv2.inRange(hsv, ll_b, ul_b)
		res_b = cv2.bitwise_or(hsv,hsv, mask=mask_b)
		gray_b = cv2.cvtColor(res_b,cv2.COLOR_BGR2GRAY)
		ret,gray_t_b = cv2.threshold(gray_b,10,255,0)
     		contours_b = cv2.findContours(gray_t_b,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]
		x,y,w,h = cv2.boundingRect(contours_b)
		img1 = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),-1)
		
		mask_r = cv2.inRange(hsv, ll_r, ul_r)
		res_r = cv2.bitwise_or(hsv,hsv, mask=mask_r)
		gray_r = cv2.cvtColor(res_r,cv2.COLOR_BGR2GRAY)
		ret,gray_t_r = cv2.threshold(gray_r,10,255,0)
     		contours_r = cv2.findContours(gray_t_r,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]
		x,y,w,h = cv2.boundingRect(contours_r)
		img2 = cv2.rectangle(img1,(x,y),(x+w,y+h),(0,0,255),-1)
	
		cv2.imshow('img', img)
		cv2.waitKey(5)
	
		

if __name__ == '__main__':
	test = detect()
	rospy.spin()
