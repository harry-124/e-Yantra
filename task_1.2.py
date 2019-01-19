#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from plutodrone.msg import *
from pid_tune.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import time
from cv_bridge import CvBridge

class WayPoint:
	
	def __init__(self):

		rospy.init_node('ros_bridge')
		
		# Create a ROS Bridge
		self.ros_bridge =CvBridge()

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('/visionSensor/image_rect', Image, self.image_callback)

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		self.red_pub   = rospy.Publisher('/red',Int32,queue_size=10)
		self.blue_pub   = rospy.Publisher('/blue',Int32,queue_size=10)
		self.green_pub   = rospy.Publisher('/green',Int32,queue_size=10)
                
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('/drone_yaw',Float64,self.yaw_error)

		self.cmd = PlutoMsg()
		

    
   
		self.wp_x = np.array([0.0,-5.63,5.57,5.55,-5.6,0])
		self.wp_y = np.array([0.0,-5.63,-5.63,5.54,5.54,0])
		self.wp_z = np.array([30.0,30.0,30.0,30.0,30.0,30.0])
		self.i=0
		self.j=0

		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0
		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		#for correction purpose
		self.error_x=0.0
		self.error_y=0.0
		self.error_z=0.0
		self.error_yaw=0.0
		self.integrator_yaw=0.0
                self.integrator_x=0.0
		self.integrator_y=0.0
		self.integrator_z=0.0
		self.derivative_x=0.0
		self.derivative_y=0.0
		self.derivative_z=0.0
		self.derivative_yaw=0.0

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.04
		self.ct_time=0.0
		self.maxtime=15.0

		self.blue_ct= 0
		self.green_ct= 0
		self.red_ct= 0

		rospy.sleep(.1)
                
	

	
	def image_callback(self,msg):

		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		   
		# define range of  color in BGR
    		lower_blue = np.array([210,0,0])
    		upper_blue = np.array([220,10,10])
		lower_red  = np.array([0,0,210])
		upper_red  = np.array([10,10,220])
		lower_green= np.array([0,210,0])
		upper_green= np.array([10,220,10])

    		# Threshold the BGR image to get only desired colors and grayscaling
    		blue = cv2.inRange(image, lower_blue, upper_blue)
		blue1=cv2.bitwise_and(image,image,mask=blue)
		blue_gray = cv2.cvtColor(blue1,cv2.COLOR_BGR2GRAY)
                green = cv2.inRange(image, lower_green, upper_green)
		green1=cv2.bitwise_and(image,image,mask=green)
		green_gray = cv2.cvtColor(green1,cv2.COLOR_BGR2GRAY)
		red = cv2.inRange(image, lower_red, upper_red)
		red1=cv2.bitwise_and(image,image,mask=red)
		red_gray = cv2.cvtColor(red1,cv2.COLOR_BGR2GRAY)
			
		
		#thresholding
		ret,blue_t = cv2.threshold(blue_gray,10,255,0)
		ret,green_t = cv2.threshold(green_gray,10,255,0)
		ret,red_t = cv2.threshold(red_gray,10,255,0)

		#contour detection and centroid
		_,blue_contours,_ = cv2.findContours(blue_t,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		_,green_contours,_ = cv2.findContours(green_t,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		_,red_contours,_ = cv2.findContours(red_t,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		
		
		#displaying pts		
		self.blue_ct= len(blue_contours)
		
		self.green_ct= len(green_contours)
		
		self.red_ct= len(red_contours)
		

    		
	
	
	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)

		while True:
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
															
			throt_value = int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)

			yaw_value=int(1500 + self.correct_yaw)
			self.cmd.rcYaw = self.limit(yaw_value, 1750,1350)
		

			#for stablizing the flight
			if(not(self.i)):
				if(not(self.j)):
					self.ct_time=time.time()
					self.j=self.j+1
				if((time.time()-self.ct_time)>self.maxtime):
   				        self.i=self.i+1

                        #visiting the waypoints
			if((self.i)):
				if((self.error_x<0.2)and(self.error_x>-0.2)):
					if((self.error_y<0.2)and(self.error_y>-0.2)):
						if((self.error_z<1.5)and(self.error_z>-1.5)):
							if(self.i<5):
								self.i=self.i+1
								rospy.sleep(0.2)
			
			self.pluto_cmd.publish(self.cmd)
			self.blue_pub.publish(self.blue_ct)
			self.green_pub.publish(self.green_ct)
			self.red_pub.publish(self.red_ct)
	

	def calc_pid(self):
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			self.publish_plot_data()
			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			self.pid_yaw()
			
			self.last_time = self.seconds


	def pid_pitch(self):
                #Compute Pitch PID here
		self.error_x=(self.wp_x[self.i]-self.drone_x)
                self.P_value=(7)*(self.error_x)
		self.D_value=(1)*(self.error_x-self.derivative_x)
		self.derivative_x=self.error_x
                self.integrator_x=self.integrator_x +(self.error_x*self.loop_time)
		if self.integrator_x >(100):
			self.integrator_x = 100
		elif self.integrator_x <(-100):
			self.integrator_x = -100

                self.I_value=(0)*(self.integrator_x)/100 
		self.correct_pitch=self.P_value+self.D_value+self.I_value
               

                
	def pid_roll(self):
                #Compute roll PID here
		self.error_y=(self.wp_y[self.i]-self.drone_y)
                self.P_value=(10)*(self.error_y)
		self.D_value=(2)*(self.drone_y-self.derivative_y)
		self.derivative_y=self.drone_y
                self.integrator_y=self.integrator_y + (self.error_y*self.loop_time)
		if self.integrator_y >(500):
			self.integrator_y = 500
		elif self.integrator_y <(100):
			self.integrator_y = 100
                self.integrator_y=self.integrator_y + self.error_y
                self.I_value=(0)*(self.integrator_y)/100
		self.correct_roll=self.P_value+self.D_value+self.I_value


	def pid_throt(self):
                #Compute Throttle PID here
		self.error_z=(self.wp_z[self.i]-self.drone_z)
                self.P_value=(27)*(self.error_z)
		self.D_value=(75)*(self.error_z-self.derivative_z)
		self.derivative_z=self.error_z
                self.integrator_z=self.integrator_z + (self.error_z*self.loop_time)
		if self.integrator_z >(100):
			self.integrator_z = 100
		elif self.integrator_z <(-100):
			self.integrator_z = -100
                self.I_value=(20)*(self.integrator_z)/100
		self.correct_throt=self.P_value+self.D_value+self.I_value

	def pid_yaw(self):
                #Compute YAW PID here
                self.P_value=(10)*(self.error_yaw)
		self.D_value=(8)*(self.error_yaw-self.derivative_yaw)
		self.derivative_yaw=self.error_yaw
                self.integrator_yaw=self.integrator_yaw + (self.error_yaw*self.loop_time)
		if self.integrator_yaw >(100):
			self.integrator_yaw = 100
		elif self.integrator_yaw <(-100):
			self.integrator_yaw = -100
                self.I_value=(10)*(self.integrator_yaw)/100
		self.correct_yaw=self.P_value+self.D_value+self.I_value



	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

	#You can use this function to publish different information for your plots
	def publish_plot_data(self):
		pub_y=rospy.Publisher('/Error_y',Float64,queue_size=10)
		pub_y.publish(self.error_y)
		pub_x=rospy.Publisher('/Error_x',Float64,queue_size=10)		     
                pub_x.publish(self.error_x)
		pub_z=rospy.Publisher('/Error_z',Float64,queue_size=10)
		pub_z.publish(self.error_z)

	def yaw_error(self,data):
		
		self.error_yaw=-(data.data)
		
	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
	
              
					
 			


		
if __name__ == '__main__':
	test = WayPoint()
	test.position_hold()
	rospy.spin()

		
