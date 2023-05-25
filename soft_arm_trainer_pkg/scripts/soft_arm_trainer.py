#!/usr/bin/env python
"""
soft_arm_trainer.py
Function: A package to collect training data for the soft arm
Author: Benjamin Walt
Date: 6/21/2021
Purpose: SoftAgBot system integration project
Version: 0.2
"""


import rospy
import RPi.GPIO as GPIO
import random
import os
import pickle
import numpy as np


"""
Equipment classes
"""


"""
Messages
"""
from std_msgs.msg import Bool
from pi_comm_pkg.msg import soft_arm
from pi_comm_pkg.msg import servo_arm
from pi_comm_pkg.msg import theta_4_arm
from pi_comm_pkg.msg import servo_camera
from pi_comm_pkg.msg import soft_arm_pressure
from pi_comm_pkg.msg import arm_state
from pi_comm_pkg.msg import extrusion
from patriot_pkg.msg import arm_pose
from pi_comm_pkg.msg import arm_encoders


"""
The launch file sets the CWD to:
/home/ubuntu/catkin_ts/src/soft_arm_trainer_pkg/scripts
(i.e. the directory of this file)
(without this the CWD is /home/ubuntu/.ros)
This changes the CWD to:
/home/ubuntu/catkin_ts/src/soft_arm_trainer_pkg/data
where the collected data is saved.
"""
os.chdir("../data")	

_COMPRESSOR_RELAY_PIN = rospy.get_param("/compressor_relay")

_TRAJECTORY_SIZE = rospy.get_param("/trajectory_size")
_NUMB_TRAJECTORY = rospy.get_param("/numb_trajectories")
_WAIT = rospy.get_param("/wait")
_T1_OFFSET = 0#rospy.get_param("/offset_theta_1")
_T2_OFFSET = 0#rospy.get_param("/offset_theta_2")
_T3_OFFSET = 0#rospy.get_param("/offset_theta_3")
_T4_OFFSET = 0#rospy.get_param("/offset_theta_4")
_START_PLAN_COUNT = rospy.get_param("/start_plan_count")
_ZERO_SOFT_CONFIG = [0.0, 0.0, 0,0]

class ArmData():
	def __init__(self, config, length, bending, r1, r2, arm_config, soft_arm_end,
					dbending, drotation, dtheta_4, dlength):
		self.config = config
		self.length = length
		self.bending = bending
		self.r1 = r1
		self.r2 = r2
		self.arm_config = arm_config
		self.soft_arm_end = soft_arm_end
		self.dbending = dbending
		self.drotation = drotation
		self.dtheta_4 = dtheta_4
		self.dlength = dlength

class PatriotData():
	def __init__(self, x, y, z, qx, qy, qz, qw):
		self.x = x
		self.y = y
		self.z = z
		self.qx = qx
		self.qy = qy
		self.qz = qz
		self.qw = qw


class Soft_Trainer:

	def __init__(self):
		self._arm_angle = 0
		self._cur_rigid_arm_config = [0, 0, 0]
		self._rest_arm_config = [90, 90, 90]
		self._zero_arm_config = [0, 0, 0]#[_T1_OFFSET, _T2_OFFSET, _T3_OFFSET, _T4_OFFSET]
		self._test_start_config = [0, 0, 90]#[_T1_OFFSET, _T2_OFFSET, _T3_OFFSET, _T4_OFFSET]
		self._test_config = [0, 0, 0]
		self._sensor_pose  = PatriotData(0, 0, 0, 0, 0, 0, 1)
		self._rate = rospy.Rate(20)
		
		
		"""
		Subscribers
		"""
		# rospy.Subscriber("pi_comm/arm_state", arm_state, self._state_update_callback)
		rospy.Subscriber("patriot/arm_pose", arm_pose, self._pose_update_callback)
		rospy.Subscriber('pi_comm/extruder_status', Bool, self._extruder_status_callback)
		
		"""
		Publishers
		"""
		self._servo_arm_pub = rospy.Publisher('pi_comm/servo_arm', servo_arm, queue_size=10)
		self._servo_theta_4_pub = rospy.Publisher('pi_comm/theta_4_arm', theta_4_arm, queue_size=10)
		self._soft_arm_pub = rospy.Publisher('pi_comm/soft_arm', soft_arm, queue_size=10)
		self._compressor_control_pub = rospy.Publisher('accumulator_enable', Bool, queue_size=10)
		self._stepper_control_pub = rospy.Publisher('pi_comm/set_extrusion', extrusion, queue_size=10)
		
		rospy.sleep(2) # Lets the publishers start
		self._setup()






	def _setup(self):
		self._position_rigid_arm(self._test_start_config)
		self._compressor_control_pub.publish(True)
		angle = raw_input("Link 4 angle (Horizontal is 0, positive is down)")
		self._arm_angle = float(angle)
		angle_corrected = self._cur_rigid_arm_config[2] + float(angle)
		self._test_config = [self._cur_rigid_arm_config[0], self._cur_rigid_arm_config[1], angle_corrected]
		self._position_rigid_arm(self._test_config)
		# Start training
		self._train()
		# Turn off compressor
		self._compressor_control_pub.publish(False)
		self._position_soft_arm(_ZERO_SOFT_CONFIG)
		self._extrude(0)
		self._go_home()
		exit()


	def _pose_update_callback(self, msg):
		self._sensor_pose.x = msg.soft_arm_pose[0]/100.0 # Convert to meters
		self._sensor_pose.y = msg.soft_arm_pose[1]/100.0
		self._sensor_pose.z = msg.soft_arm_pose[2]/100.0
		self._sensor_pose.qx = msg.soft_arm_pose[3]
		self._sensor_pose.qy = msg.soft_arm_pose[4]
		self._sensor_pose.qz = msg.soft_arm_pose[5]
		self._sensor_pose.qw = msg.soft_arm_pose[6]


	def _extrude(self, length):
		msg = extrusion()
		msg.length_cm = length
		self._stepper_control_pub.publish(msg)
		self._cur_extrusion = length


	def _extruder_status_callback(self, msg):
		self._extruder_status = msg.data


	def _position_rigid_arm(self, config):
		self._cur_rigid_arm_config = config
		msg = servo_arm()
		msg.servo_arm_rad = [np.deg2rad(config[0]), np.deg2rad(config[1]), np.deg2rad(config[2])]
		self._servo_arm_pub.publish(msg)
	
	def _position_theta_4(self, config_deg):
		msg = theta_4_arm()
		msg.theta_4_rad = np.deg2rad(config_deg)
		self._servo_theta_4_pub.publish(msg)
	
	def _position_soft_arm(self, config):
		self._cur_soft_arm_config = config
		msg = soft_arm()
		msg.bend_press = config[0]
		msg.rot_1_press = config[1]
		msg.rot_2_press = config[2]
		self._soft_arm_pub.publish(msg)

		
					
	def _training_loop_trajectory(self, plan_num):
		# #plan_num += _START_PLAN_COUNT
		filename = "soft_arm_data_" + str(self._arm_angle) + "_" + str(plan_num) + ".pkl"
		data = []
		data.append([self._arm_angle, plan_num])

		length = 0
		bending = 0
		rotation = 0
		theta_4 = 0
		for config in range(_TRAJECTORY_SIZE):
			print("config " + str(config)) 
			
			last_bending = bending
			last_rotation = rotation
			last_theta_4 = theta_4
						
			dlength = np.random.uniform(0,3)
			length = np.clip(length + dlength, 0, 20)
			
			bending = random.uniform(0, 40)
			rotation = random.uniform(-40, 40)
			if rotation < 0:
				r2 = abs(rotation)
				r1 = 0
			else:
				r2 = 0
				r1 = abs(rotation)
					
			dtheta_4 = random.uniform(-40, 40)
			theta_4 = np.clip(theta_4 + dtheta_4, -90, 90)
			dbending = bending - last_bending
			drotation = rotation - last_rotation

			# extrude arm
			print("Extruding Soft Arm")
			self._extrude(length)
			while (self._extruder_status == True):
				self._rate.sleep()
			# publish pressures
			print("Move Soft Arm")
			self._position_soft_arm([bending, r1, r2])
			# Move theta_4
			print("Position Theta 4")
			self._position_theta_4(theta_4)
			# wait 
			rospy.sleep(_WAIT)

			# collect data from polhemus
			print("Collect data from polhemus")
			print("X: {} Y: {} Z: {}".format(self._sensor_pose.x, self._sensor_pose.y, self._sensor_pose.z))
			data.append(ArmData(config, length, bending, r1, r2, self._cur_rigid_arm_config, self._sensor_pose, dbending, drotation, dtheta_4, dlength))

		print("Saving data")
		with open(filename, "wb") as filehandle:
			pickle.dump(data, filehandle)
			
		self._position_soft_arm(_ZERO_SOFT_CONFIG)
		self._extrude(0)
		rospy.sleep(3)
		print(plan_num)
		raw_input("Press enter to continue")

	def _go_home(self):
		self._position_theta_4(0)
		self._position_rigid_arm([90, 45, 90])
		rospy.sleep(3)
		self._position_rigid_arm([90, 90, 90])

		
	def _train(self):
		for plan in range(_NUMB_TRAJECTORY):
			plan += _START_PLAN_COUNT
			print("Plan " + str(plan)) 
			self._training_loop_trajectory(plan)
			
		self._position_rigid_arm(self._test_start_config)

if __name__ == '__main__':
	rospy.init_node('soft_arm_trainer_node', anonymous=True)
	Soft_Trainer()



