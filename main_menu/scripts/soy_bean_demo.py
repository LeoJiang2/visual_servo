#!/usr/bin/env python
"""
soy_bean_demo.py
Function: This program is for doing a brief demo of scanning a soy bean plant
Author: Benjamin Walt
Date: 10/1/2021
Purpose: SoftAgBot system project
Version: 0.1
"""


"""
General Imports
"""
import rospy
# import RPi.GPIO as GPIO
# import sys
import numpy as np
import copy
import json
import time
import random
import os

"""
Equipment classes
"""
# sys.path.append('/home/ubuntu/catkin_ts/src/Stepper')
# import StepperMotor as SM

"""
Messages
"""
from std_msgs.msg import Bool
from pi_comm_pkg.msg import gripper_reply
from pi_comm_pkg.msg import soft_arm
from pi_comm_pkg.msg import servo_arm
from pi_comm_pkg.msg import theta_4_arm
from pi_comm_pkg.msg import servo_camera
from pi_comm_pkg.msg import soft_arm_pressure
from pi_comm_pkg.msg import gripper_pressure
from pi_comm_pkg.msg import ir_reply
from task_space_pkg.msg import rigid_arm_position_desired
from pi_comm_pkg.msg import arm_state
from pi_comm_pkg.msg import extrusion
from patriot_pkg.msg import arm_pose


# scan_height = 0.48
# x_pos = 0.4

POSE = [0.6, 0.0, 0.45]

_CONFIG_REST = [90, 90, 90]
_CONFIG_REST_PRE = [90, 45, 90]


# _SOFT_CONFIG_ZERO = [0, 0, 0]
# _SOFT_CONFIG_1 = [25.0, 30.0, 0.0]
# _SOFT_CONFIG_2 = [0, 20.0, 0]
# _SOFT_CONFIG_3 = [0, 0, 25.0]


class Demo:
	def __init__(self):

		"""
		Initilize
		"""
		self._cur_state = []
		self._cur_patriot_data = []
		self._data = []

		"""
		Subscribers
		"""
		rospy.Subscriber("pi_comm/arm_state", arm_state, self._state_update_callback)
		rospy.Subscriber("patriot/arm_pose", arm_pose, self._pose_update_callback)

		"""
		Publishers
		"""
		self._servo_arm_pub = rospy.Publisher('pi_comm/servo_arm', servo_arm, queue_size=10)
		self._soft_arm_pub = rospy.Publisher('pi_comm/soft_arm', soft_arm, queue_size=10)
		self._compressor_control_pub = rospy.Publisher('accumulator_enable', Bool,queue_size=10)
		self._rigid_arm_position_desired_pub = rospy.Publisher('task_space/rigid_arm_position_desired', rigid_arm_position_desired, queue_size=10)
		self._stepper_control_pub = rospy.Publisher('pi_comm/set_extrusion', extrusion, queue_size=10)
		self._servo_theta_4_pub = rospy.Publisher('pi_comm/theta_4_arm', theta_4_arm, queue_size=10)
		rospy.sleep(2) # Lets the publishers start

		# #self._compressor_control_pub.publish(True)
		self._compressor_control_pub.publish(True)
		# #self._rate = rospy.Rate(_SPIN_RATE) # 20hz
		self._cur_rigid_arm_config = copy.copy(_CONFIG_REST)
		self._cur_theta_4_rad = 0
		self._cur_soft_arm_config = [0.0, 0.0, 0.0]
		self._cur_rigid_arm_config_ts = [0.0, 0.0, 0.0]
		if(not os.path.isdir("data/")):
			print("Creating new data directory")
			os.system("mkdir data")
		self.run_demo()
		


	def _position_rigid_arm(self, config):
		self._cur_rigid_arm_config = config
		msg = servo_arm()
		msg.servo_arm_rad = [np.deg2rad(config[0]), np.deg2rad(config[1]), np.deg2rad(config[2])]
		self._servo_arm_pub.publish(msg)

	def _position_theta_4(self, config):
		self._cur_theta_4 = np.deg2rad(config)
		msg = theta_4_arm()
		msg.theta_4_rad = np.deg2rad(config)
		self._servo_theta_4_pub.publish(msg)


	def _position_rigid_arm_ts(self, config):
		# #print("Publishing in main")
		self._cur_rigid_arm_config_ts = copy.copy(config)
		msg = rigid_arm_position_desired()
		msg.rigid_arm_position_desired = [config[0], config[1], config[2]]
		self._rigid_arm_position_desired_pub.publish(msg)

	def _position_soft_arm(self, config):
		# #print(config)
		self._cur_soft_arm_config = config
		msg = soft_arm()
		msg.bend_press = config[0]
		msg.rot_1_press = config[1]
		msg.rot_2_press = config[2]
		self._soft_arm_pub.publish(msg)

	def _extrude(self, length):
		# #input_val = float(raw_input("Length (cm)(Cur: {}): ".format(self._cur_extrusion)) or self._cur_extrusion)
		# #input_val = int(input_val)
		msg = extrusion()
		msg.length_cm = length
		self._stepper_control_pub.publish(msg)
		# #self._step.move_stepper(input_val)
		self._cur_extrusion = length


	def _state_update_callback(self, msg):
		# #print("State update Callback")
		# self._cur_rigid_arm_config = [msg.joints_rad[0]*180.0/3.14, msg.joints_rad[1]*180.0/3.14, msg.joints_rad[2]*180.0/3.14, msg.theta_4_rad*180.0/3.14]
		# #self._state_update_flag = False
		self._cur_state = [msg.joints_rad, msg.theta_4_rad, msg.soft_arm_pressure, msg.extrusion_length_cm]

	def _pose_update_callback(self, msg):
		self._cur_patriot_data = [msg.soft_arm_pose[0]/100.0 , msg.soft_arm_pose[1]/100.0 , msg.soft_arm_pose[2]/100.0, msg.soft_arm_pose[3], msg.soft_arm_pose[4], msg.soft_arm_pose[5], msg.soft_arm_pose[6] ]


	def _pan_arm(self, point_start, point_end):
		start = np.asarray(point_start)
		print(start)
		end = np.asarray(point_end)
		slope = end - start
		step = np.linspace(0.0, 1.0, num=20)
		for i in step:
			goal_point = start + i*slope
			self._position_rigid_arm_ts([goal_point[0], goal_point[1], goal_point[2]])
			rospy.sleep(0.2)

	def _zoom(self, length):
		step = 3
		move = 0.0
		while(length > 0):
			move += min(length,step)
			self._extrude(move)
			print(move)
			self._position_rigid_arm_ts([self._cur_rigid_arm_config_ts[0], self._cur_rigid_arm_config_ts[1], self._cur_rigid_arm_config_ts[2] + 0.01])
			rospy.sleep(.75)
			length += -step


	def run_demo(self):
		# run = True
		raw_input("Press Enter to continue to next configuration 0...")
		self._position_rigid_arm(_CONFIG_REST)
		raw_input("Press Enter to continue to next configuration 1...")
		# #self._position_rigid_arm_ts([0.3, 0.0, scan_height])
		self._position_rigid_arm(_CONFIG_REST_PRE)
		raw_input("Press Enter to continue to next configuration 3...")
		self._position_rigid_arm_ts(POSE)
		raw_input("Press Enter to continue to extrude...")
		self._extrude(10.0)
		raw_input("Press Enter to continue to random motion...")
		a = 0
		for b in range(5,21,5):
			for r in range(-30,31,6):
				if r > 0:
					r1 = r
					r2 =0
				else:
					r1 = 0
					r2 = -1*r

				self._position_soft_arm([b,r1, r2])
				rospy.sleep(3)

				# Take a image
				current_img_path = "data/" + "img_" + str(a) + ".jpg"
				command_img = "fswebcam -d/dev/video0 -q -r 640x480 --no-banner --jpeg 95 -F 1 " + current_img_path
				os.system(command_img) # current image
				rospy.sleep(2)
				
				self._data.append([a, self._cur_state, self._cur_patriot_data])
				a+=1
		
		# for a in range(25):
			# # generate a random configuration
			# B = np.random.randint(10,25,1)
			# R = np.random.randint(-30,30,1)
			# #Move the soft arm_pose
			# if R[0] > 0:
				# r1 = R[0]
				# r2 =0
			# else:
				# r1 = 0
				# r2 = -1*R[0]

			# self._position_soft_arm([B[0],r1, r2])
			# rospy.sleep(3)

			# # Take a image
			# current_img_path = "data/" + "img_" + str(a) + ".jpg"
			# command_img = "fswebcam -d/dev/video0 -q -r 640x480 --no-banner --jpeg 95 -F 5 " + current_img_path
			# os.system(command_img) # current image
			# rospy.sleep(1)



			# Store image and data
			#self._data.append([a, self._cur_state, self._cur_patriot_data])













			# raw_input("Press Enter to continue to next configuration 5...")
			# self._position_rigid_arm(_CONFIG_REST_PRE)
			# raw_input("Press Enter to continue to next configuration 5...")
			# self._position_rigid_arm(_CONFIG_REST)
			# print("Done")
			# # #self._compressor_control_pub.publish(False)
			# run = False

		with open("data/data.json", "w") as file1:
			# Writing data to a file
			json.dump(self._data, file1)

		raw_input("Press Enter to continue to next configuration 5...")
		self._position_soft_arm([0,0,0])
		raw_input("Press Enter to continue to next configuration 5...")
		self._extrude(0)
		
		raw_input("Press Enter to continue to next configuration 0...")
		self._position_rigid_arm(_CONFIG_REST_PRE)
		raw_input("Press Enter to continue to next configuration 1...")
		# #self._position_rigid_arm_ts([0.3, 0.0, scan_height])
		self._position_rigid_arm(_CONFIG_REST)
		self._compressor_control_pub.publish(False)




if __name__ == '__main__':
	rospy.init_node('demo_exec_node', anonymous=True)
	Demo()
	#rospy.spin()
