#!/usr/bin/env python
"""
calibration_node_exec.py
Function: This program is for calibration of the robot arm and camera transform
Author: Benjamin Walt
Date: 6/8/2021
Purpose: SoftAgBot system integration project
Version: 0.1
"""

"""
General Imports
"""
import rospy
import sys
import numpy as np
import numpy.linalg as la
import json
import os
import time

"""
Equipment classes
"""

"""
Messages
"""
from std_msgs.msg import Bool
from std_msgs.msg import String
from pi_comm_pkg.msg import servo_camera
from pi_comm_pkg.msg import arm_state
from patriot_pkg.msg import arm_pose
from task_space_pkg.msg import rigid_arm_position_desired
from pi_comm_pkg.msg import servo_arm
from pi_comm_pkg.msg import theta_4_arm

"""
The launch file sets the CWD to:
/home/ubuntu/clean_up/src/calibration_pkg/scripts
(i.e. the directory of this file)
(without this the CWD is /home/ubuntu/.ros)
This changes the CWD to:
/home/ubuntu/clean_up/src/calibration_pkg/data
where the collected data is saved.
"""
os.chdir("../data")	


class Calibration:
	def __init__(self):
		
		"""
		Initilize
		"""
			
		"""
		Subscribers
		"""
		rospy.Subscriber("patriot/arm_pose", arm_pose, self._pose_update_callback)
		rospy.Subscriber("calibration/image_return", String, self._image_return_callback)
		rospy.Subscriber('pi_comm/arm_state', arm_state, self._arm_state_callback)

		
		"""
		Publishers
		"""
		self._servo_camera_pub = rospy.Publisher('pi_comm/servo_camera', servo_camera, queue_size=10)
		self._camera_image_call_pub = rospy.Publisher('calibration/image_call', Bool, queue_size=10)
		self._move_arm_pub = rospy.Publisher('task_space/rigid_arm_position_desired', rigid_arm_position_desired, queue_size=1)
		self._servo_arm_pub = rospy.Publisher('pi_comm/servo_arm', servo_arm, queue_size=10)
		self._servo_theta_4_pub = rospy.Publisher('pi_comm/theta_4_arm', theta_4_arm, queue_size=10)

		rospy.sleep(2) # Lets the publishers start
		

		self._sensor_pose = [0.0 ,0.0 ,0.0]
		self._cur_pan = 0.0
		self._cur_tilt = 0.0

		self._position_camera(0,0)
		self._rate = rospy.Rate(20)
		self._current_image_name = ""
		self._current_camera_point = [0.5, 0, 0.5]
		self._current_joint_state = [0, 0, 0]
		self._current_theta_4_state = 0
		self._image_update_flag = False
		self.menu_loop()
				
	"""
	Prints out the option menu
	"""
	def _menu_print(self):
		print("\n \n")
		print("*******************************************************************************")
		print("Camera: Pan: {} Tilt: {}".format(self._cur_pan, self._cur_tilt))
		print("*******************************************************************************")
		print("\n")
		#print("0: quit")
		print("1: Begin Calibration")
		print("2: Go home")
		
	def _arm_state_callback(self, msg):
		self._current_joint_state = msg.joints_rad
		self._current_theta_4_state = msg.theta_4_rad
		
	
	def _pose_update_callback(self, msg):
		self._sensor_pose = [msg.soft_arm_pose[0]/100.0, msg.soft_arm_pose[1]/100.0 , msg.soft_arm_pose[2]/100.0]
		# #print("X: {} Y: {} Z: {}".format(self._sensor_pose[0], self._sensor_pose[1], self._sensor_pose[2]))

	def _image_return_callback(self, msg):
		self._current_image_name = msg.data
		self._image_update_flag = True

	
	def _position_camera(self, pan_deg, tilt_deg):
		msg = servo_camera()
		msg.camera_pan_rad = np.deg2rad(pan_deg)
		msg.camera_tilt_rad = np.deg2rad(tilt_deg)
		self._servo_camera_pub.publish(msg)
	
			
	def _position_rigid_arm(self, config_deg):
		msg = servo_arm()
		msg.servo_arm_rad = [np.deg2rad(config_deg[0]), np.deg2rad(config_deg[1]), np.deg2rad(config_deg[2])]
		self._servo_arm_pub.publish(msg)
		
	def _position_theta_4(self, config_deg):
		msg = theta_4_arm()
		msg.theta_4_rad = np.deg2rad(config_deg)
		self._servo_theta_4_pub.publish(msg)
	
	
	def _camera2arm_base(self, point_camera, pan_d, tilt_d):
		point_camera_h = np.array([point_camera[0], point_camera[1], point_camera[2], 1])
		# #print(point_camera_h)
		theta_pan = np.deg2rad(pan_d)
		theta_tilt = np.deg2rad(tilt_d)
		transform = np.array([[ -np.sin(theta_pan),  np.cos(theta_pan)*np.sin(theta_tilt),  np.cos(theta_pan)*np.cos(theta_tilt), 0.0530*np.sin(theta_pan) - 0.0360*np.cos(theta_pan) + 0.1343*np.cos(theta_pan)*np.cos(theta_tilt) - 0.0670*np.cos(theta_pan)*np.sin(theta_tilt) + np.cos(theta_pan)*(0.0670*np.sin(theta_tilt) - 0.0560*np.cos(theta_tilt) + 0.0560) + 0.0360], 
							[ -np.cos(theta_pan), -np.sin(theta_pan)*np.sin(theta_tilt), -np.cos(theta_tilt)*np.sin(theta_pan), 0.0530*np.cos(theta_pan) + 0.0360*np.sin(theta_pan) - 0.1343*np.cos(theta_tilt)*np.sin(theta_pan) + 0.0670*np.sin(theta_pan)*np.sin(theta_tilt) - np.sin(theta_pan)*(0.0670*np.sin(theta_tilt) - 0.0560*np.cos(theta_tilt) + 0.0560) + 0.1435], 
							[0, -np.cos(theta_tilt), np.sin(theta_tilt), 0.0784*np.sin(theta_tilt) + 0.0670], 
							[0, 0, 0, 1]])

		point_base_h = np.matmul(transform, point_camera_h)
		print(point_base_h)
		return [point_base_h[0], point_base_h[1], point_base_h[2]]


	def _calibration(self):
		data = []
		
		# Set the camera position
		pan_value = int(raw_input("Enter Pan Value in Degrees: "))
		tilt_value = int(raw_input("Enter tilt Value in Degrees: "))
		self._position_camera(pan_value, tilt_value)
		config_num = 0
		# Loop though a several points
		while config_num < 1:
			print("Position target in configuration {}.".format(str(config_num)))
			# #raw_input("Press enter to collect photo data")
			# Request data from NUC
			# #self._camera_image_call_pub.publish(1)
			# Wait for data
			# #while(not self._image_update_flag):
				# #pass
			
			# Measure target
			raw_input("Press enter to collect target position data")
			target_pose = self._sensor_pose
			print("Done!")
			
			x_c = float(raw_input("Enter camera X Value in meters: "))
			y_c = float(raw_input("Enter camera Y Value in meters: "))
			z_c = float(raw_input("Enter camera Z Value in meters: "))

			self._current_camera_point = self._camera2arm_base([x_c, y_c, z_c], pan_value, tilt_value)
			skip = False
			use_value = raw_input("Use point? y/n Enter equals 'y'") or "y"
			if use_value != "y":
				skip = True
			
			if (la.norm(self._current_camera_point) < 0.3):
				print("Point is very close! Skipping...")
				print("Norm is {}".format(la.norm(self._current_camera_point)))
				skip = True
				
			if (skip == False):
				# Move arm
				raw_input("Press enter to move arm")
				msg = rigid_arm_position_desired()
				msg.rigid_arm_position_desired = self._current_camera_point
				self._move_arm_pub.publish(msg)
				
				
				
				# Measure tip
				raw_input("Press enter to collect arm tip position data")
				tip_pose = self._sensor_pose
				print("Done!")
				
				# Record Data
				data.append([pan_value, tilt_value, config_num, target_pose, tip_pose, self._current_joint_state, self._current_theta_4_state])
				self._image_update_flag = False
				
				self._position_rigid_arm([0, 0, 90])
				config_num += 1
				rospy.sleep(2)
			else:
				print("Redoing point")
				# config_num not incremented
				

					

		# Save data 
		

		timestr = time.strftime("%Y%m%d-%H%M%S")	
		filename = "test_" + str(pan_value) + "_" + str(tilt_value) + "_" + timestr + ".json"
		with open(filename, "wb") as filehandle:
			json.dump(data, filehandle)	
		self._position_camera(0, 0)
	
	def _go_home(self):
		self._position_theta_4(0)
		self._position_camera(0, 0)
		self._position_rigid_arm([90, 45, 90])
		rospy.sleep(3)
		self._position_rigid_arm([90, 90, 90])
		
			
	def menu_loop(self):
		while not rospy.is_shutdown():
			self._menu_print()
			selection = input("Input selection: ")
			selection = int(selection)
			if selection == 1: # gripper
				self._position_rigid_arm([90, 45, 90])
				rospy.sleep(2)
				self._position_rigid_arm([0, 0, 90])
				self._calibration()
			elif selection == 2:
				self._go_home()
			else:
				print("Please pick a selection on the list")
			self._rate.sleep()

		

if __name__ == '__main__':
	rospy.init_node('calibration_node_exec', anonymous=True)
	Calibration()
	
	#rospy.spin()
