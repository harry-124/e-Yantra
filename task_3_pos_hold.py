#!/usr/bin/env python

#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time


class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self):
		
		rospy.init_node('pluto_fly', disable_signals = True)

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
	
                
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
                rospy.Subscriber('/dron_yaw',Float64,self.yaw_error)
		
		self.cmd = PlutoMsg()

		# Position to hold.
		self.wp_x = 0.0
		self.wp_y = 0.0
		self.wp_z = 20.0
		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0

		#PID constants for Roll
		self.kp_roll = 0.0
		self.ki_roll = 0.0
		self.kd_roll = 0.0

		#PID constants for Pitch
		self.kp_pitch = 0.0
		self.ki_pitch = 0.0
		self.kd_pitch = 0.0
		
		#PID constants for Yaw
		self.kp_yaw = 0.0
		self.ki_yaw = 0.0
		self.kd_yaw = 0.0

		#PID constants for Throttle
		self.kp_throt = 0.0
		self.ki_throt = 0.0
		self.kd_throt = 0.0

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		#for correction purpose
		self.error_x=0.0
		self.error_y=0.0
		self.error_z=0.0
		self.error_n=0.0
		self.error_m=0.0
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

		rospy.sleep(.1)
                

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
															
			roll_value = int(1500 + self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
															
			throt_value = int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1800,1350)

			yaw_value=int(1500 - self.correct_yaw)
			self.cmd.rcYaw = self.limit(yaw_value, 1750,1350)
			
   												
			
			
			self.pluto_cmd.publish(self.cmd)
			

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
                self.error_y=(self.wp_y-self.drone_y)
		#self.error_x=(self.wp_x-self.drone_x)
		#self.error_n=(self.error_x-self.error_y)/1.414
                self.P_value=(11+self.kp_pitch)*(self.error_y)
		self.D_value=(11+self.kd_pitch)*(self.error_y-self.derivative_y)/self.loop_time
		self.derivative_y=self.error_y
                self.integrator_y=self.integrator_y +(self.error_y)
		if self.integrator_y >(100):
			self.integrator_y = 100
		elif self.integrator_y <(-100):
			self.integrator_y = -100

                self.I_value=(self.ki_pitch)*(self.integrator_y)/100		
		self.correct_pitch=self.P_value+self.D_value+self.I_value
               

                
	def pid_roll(self):
                #Compute roll PID here
		self.error_x=(self.wp_x-self.drone_x)
		#self.error_y=(self.wp_y-self.drone_y)
		#self.error_m=(self.error_x+self.error_y)/1.414
                self.P_value=(10+self.kp_roll)*(self.error_x)
		self.D_value=(11+self.kd_roll)*(self.error_x-self.derivative_x)/self.loop_time
		self.derivative_x=self.error_x
                self.integrator_x=self.integrator_x + (self.error_x)
		if self.integrator_x >(100):
			self.integrator_x = 100
		elif self.integrator_x <(-100):
			self.integrator_x = -100
                self.integrator_x=self.integrator_x + self.error_x
                self.I_value=(self.ki_roll)*(self.integrator_x)/100
		self.correct_roll=self.P_value+self.D_value+self.I_value


	def pid_throt(self):
                #Compute Throttle PID here
		self.error_z=(self.wp_z-self.drone_z)
                self.P_value=(20.0+self.kp_throt)*(self.error_z)
		self.D_value=(100.0+self.kd_throt)*(self.error_z-self.derivative_z)
		self.derivative_z=self.error_z
                self.integrator_z=self.integrator_z + (self.error_z)
		if self.integrator_z >(100):
			self.integrator_z = 100
		elif self.integrator_z <(-100):
			self.integrator_z = -100
                self.I_value=(self.ki_throt)*(self.integrator_z)
		self.correct_throt=self.P_value+self.D_value+self.I_value

	def pid_yaw(self):
                #Compute YAW PID here
		
                self.P_value=(8.0)*(self.error_yaw)
		self.D_value=(5.0)*(self.error_yaw-self.derivative_yaw)/self.loop_time
		self.derivative_yaw=self.error_yaw
                self.integrator_yaw=self.integrator_yaw + (self.error_yaw)*(self.loop_time)
		if self.integrator_yaw >(100):
			self.integrator_yaw = 100
		elif self.integrator_yaw <(-100):
			self.integrator_yaw = -100
                self.I_value=(self.ki_yaw)*(self.integrator_yaw)
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
	
	def set_pid_alt(self,pid_val):
		
		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd
		
	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd
		
	def set_pid_yaw(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

		self.kp_yaw = pid_val.Kp
		self.ki_yaw = pid_val.Ki
		self.kd_yaw = pid_val.Kd

	def yaw_error(self,data):
		
		self.error_yaw=288 -(data.data)
		
	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
	



if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()
