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
import json
import os

"""
Equipment classes
"""

"""
Messages
"""
from std_msgs.msg import Bool
from std_msgs.msg import String
from pi_comm_pkg.msg import servo_camera
from patriot_pkg.msg import arm_pose

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

		
		"""
		Publishers
		"""
		self._servo_camera_pub = rospy.Publisher('pi_comm/servo_camera', servo_camera, queue_size=10)
		self._camera_image_call_pub = rospy.Publisher('calibration/image_call', Bool, queue_size=10)

		rospy.sleep(2) # Lets the publishers start
		

		self._sensor_pose = [0.0 ,0.0 ,0.0]
		self._cur_pan = 0.0
		self._cur_tilt = 0.0

		self._position_camera(0,0)
		self._rate = rospy.Rate(20)
		self._current_image_name = ""
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
	
	def _camera_calibration(self):
		pan_value = int(raw_input("Enter Pan Value in Degrees: "))
		tilt_value = int(raw_input("Enter tilt Value in Degrees: "))
		data = []
		# #pan = [-60, -30, 0]
		# #tilt = [0, 25, 50]

		self._position_camera(pan_value, tilt_value)
		raw_input("Press enter to base position data")
		camera_base_pose = [self._sensor_pose[0], self._sensor_pose[1] - 0.03,self._sensor_pose[2]]
		print("Done!")
		for config_num in range(1):
			print("Position target in configuration {}.".format(str(config_num)))
			raw_input("Press enter to collect photo data")
			# Request data from NUC
			self._camera_image_call_pub.publish(1)
			# Wait for data
			while(not self._image_update_flag):
				pass
			raw_input("Press enter to target position data")
			target_pose = self._sensor_pose
			print("Done!")
			
			# Record Data
			data.append([pan_value, tilt_value, config_num, target_pose, camera_base_pose, self._current_image_name])
			self._image_update_flag = False
		# #raw_input("Prepare to move to next camera pose.  Press enter to reposition camera")
				
		filename = "test_" + str(pan_value) + "_" + str(tilt_value) + "_" +  self._current_image_name + ".json"
		with open(filename, "wb") as filehandle:
			json.dump(data, filehandle)	
		self._position_camera(0, 0)
	
			
	def menu_loop(self):
		while not rospy.is_shutdown():
			self._menu_print()
			selection = input("Input selection: ")
			selection = int(selection)
			if selection == 1: # gripper
				self._camera_calibration()
			else:
				print("Please pick a selection on the list")
			self._rate.sleep()

		

if __name__ == '__main__':
	rospy.init_node('calibration_node_exec', anonymous=True)
	Calibration()
	
	#rospy.spin()
