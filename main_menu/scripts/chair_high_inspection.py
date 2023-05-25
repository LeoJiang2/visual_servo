#!/usr/bin/env python
"""
chair_high_inspection.py
Function: This program is for doing a brief demo of inspecting the neck of a person in a chair
Author: Benjamin Walt
Date: 4/27/2021
Purpose: Jump Arches Demos
Version: 0.1
"""


"""
General Imports
"""
import rospy
import RPi.GPIO as GPIO
import sys
import numpy as np
import copy

"""
Equipment classes
"""
sys.path.append('/home/ubuntu/catkin_ts/src/Stepper')
import StepperMotor as SM

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


scan_height = 0.45 # 47cm tall table + 5cm for bottle height?
x_pos = 0.4

_CONFIG_REST = [90, 90, 90]
_CONFIG_REST_PRE = [90, 45, 90]


_SOFT_CONFIG_ZERO = [0, 0, 0]
_SOFT_CONFIG_1 = [25.0, 30.0, 0.0]
_SOFT_CONFIG_2 = [0, 20.0, 0]
_SOFT_CONFIG_3 = [0, 0, 25.0]


class Demo:
	def __init__(self):
		
		"""
		Initilize
		"""
		
		"""
		Subscribers
		"""
		rospy.Subscriber("pi_comm/arm_state", arm_state, self._state_update_callback)
		
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
		self._cur_rigid_arm_config = [msg.joints_rad[0]*180.0/3.14, msg.joints_rad[1]*180.0/3.14, msg.joints_rad[2]*180.0/3.14, msg.theta_4_rad*180.0/3.14]
		# #self._state_update_flag = False
	
	
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
		run = True
		while(run == True):
			raw_input("Press Enter to continue to next configuration (rest)")
			self._position_rigid_arm(_CONFIG_REST)
		
			raw_input("Press Enter to continue to next configuration (pre-rest)")
			self._position_rigid_arm(_CONFIG_REST_PRE)
			
			# #raw_input("Press Enter to continue to next configuration 3...")
			# #self._position_rigid_arm_ts([0.4, 0.0, 0.5]) # Pre-position
			
			raw_input("Press Enter to continue to next configuration (arm high)")
			self._position_rigid_arm_ts([0.2, 0.0, 0.87]) #Up to full height
				
			raw_input("Press Enter to continue to next configuration ... (soft arm extrude)")
			self._extrude(10.0)
			
			raw_input("Press Enter to continue to next configuration ... (soft arm bend)")
			self._position_soft_arm([10.0,0.0, 0]) # Bend soft arm
			
			raw_input("Press Enter to continue to next configuration ... (soft arm zero)")
			self._position_soft_arm([0.0, 0, 0])
			
			raw_input("Press Enter to continue to next configuration ... (extrude)")
			self._extrude(0.0)
			
			# #raw_input("Press Enter to continue to next configuration ... (soft arm)")
			# #self._position_rigid_arm_ts([0.4, 0.0, 0.5]) # Pre-position
			
			raw_input("Press Enter to continue to next configuration (pre-rest)")
			self._position_rigid_arm(_CONFIG_REST_PRE)
			raw_input("Press Enter to continue to next configuration (rest)")
			self._position_rigid_arm(_CONFIG_REST)
			print("Done")
			self._compressor_control_pub.publish(False)
			run = False
	
	
		

if __name__ == '__main__':
	rospy.init_node('demo_exec_node', anonymous=True)
	Demo()
	#rospy.spin()
