#!/usr/bin/env python
"""
main_simple_ts.py
Function: This program is a simple menu to control and test the robot
Author: Benjamin Walt
Date: 2/8/2021
Purpose: SoftAgBot system integration project
Version: 0.2
"""

"""
General Imports
"""
import rospy
import sys
import numpy as np
import copy

"""
Equipment classes
"""

"""
Messages
"""
from std_msgs.msg import Bool
from pi_comm_pkg.msg import gripper_reply
from pi_comm_pkg.msg import soft_arm
from pi_comm_pkg.msg import soft_arm_ramp
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
# #from pi_comm_pkg.msg import arm_encoders


_CONFIG_REST = [90, 90, 90]
_CONFIG_VERT = [0, 0, 0, 0]


class Menu:
	def __init__(self):
		# #rospy.init_node('menu_exec_node', anonymous=True)
		# #rospy.on_shutdown(self._shutdown_hook)

		self._cur_rigid_arm_config = [90.0, 90.0, 90.0] #create early to avoid errors when subscriber starts
		self._cur_theta_4 = 0
		self._cur_arm_feedback_deg = [0.0, 0.0, 0.0]
		"""
		Initilize
		"""

		"""
		Subscribers
		"""
		rospy.Subscriber("pi_comm/arm_state", arm_state, self._state_update_callback)
		# #rospy.Subscriber("patriot/arm_pose", arm_pose, self._pose_update_callback)
		# #rospy.Subscriber('pi_comm/arm_encoders', arm_encoders, self._arm_encoder_callback)

		"""
		Publishers
		"""
		self._servo_arm_pub = rospy.Publisher('pi_comm/servo_arm', servo_arm, queue_size=10)
		self._servo_theta_4_pub = rospy.Publisher('pi_comm/theta_4_arm', theta_4_arm, queue_size=10)
		self._servo_camera_pub = rospy.Publisher('pi_comm/servo_camera', servo_camera, queue_size=10)
		self._soft_arm_pub = rospy.Publisher('pi_comm/soft_arm', soft_arm, queue_size=10)
		self._soft_arm_ramp_pub = rospy.Publisher('pi_comm/soft_arm_ramp', soft_arm_ramp, queue_size=10)
		self._compressor_control_pub = rospy.Publisher('accumulator_enable', Bool, queue_size=10)
		self._gripper_control_pub = rospy.Publisher('pi_comm/gripper_control', Bool, queue_size=10)
		self._stepper_control_pub = rospy.Publisher('pi_comm/set_extrusion', extrusion, queue_size=10)
		# #self._pwm_shutdown_pub = rospy.Publisher('pi_comm/pwm_shutdown', Bool, queue_size=10)
		self._rigid_arm_position_desired_pub = rospy.Publisher('task_space/rigid_arm_position_desired', rigid_arm_position_desired, queue_size=10)

		rospy.sleep(2) # Lets the publishers start

		self._compressor_control_pub.publish(True)
		self._compressor_status = "On"
		self._gripper_status = "Open"
		# #self._rate = rospy.Rate(_SPIN_RATE) # 20hz

		self._cur_soft_arm_config = [0.0, 0.0, 0.0]
		self._cur_extrusion = 0.0
		self._cur_rigid_arm_config_ts = [0.0, 0.0, 0.0]
		self._state_update_flag = False
		# #self._sensor_pose_update_flag = False
		self._sensor_pose = [0.0 ,0.0 ,0.0]
		self._cur_sensor_pose_calc = [0.0, 0.0, 0.0]
		self._error = 0.0
		self._cur_pan = 0.0
		self._cur_tilt = 0.0
		# Move arm to rest
		self._position_rigid_arm(_CONFIG_REST)
		self._position_theta_4(0)
		self._position_camera(0,0)
		self._rate = rospy.Rate(20)
		self.menu_loop()

	# #def _shutdown_hook(self):
		# #print("Shutdown!")
		# #print("Currently not working...")
		#self._pwm_shutdown_pub.publish(True)
	"""
	Prints out the option menu
	"""
	def _menu_print(self):
		print("\n \n")
		print("*******************************************************************************")
		print("Bending: {} r1: {} r2: {}".format(self._cur_soft_arm_config[0], self._cur_soft_arm_config[1], self._cur_soft_arm_config[2]))
		print("Theta 1: {} Theta 2: {} Theta 3: {} Theta 4: {}".format(self._cur_rigid_arm_config[0], self._cur_rigid_arm_config[1], self._cur_rigid_arm_config[2], self._cur_theta_4))
		print("Feedback: Theta 1: {} Theta 2: {} Theta 3: {}".format(self._cur_arm_feedback_deg[0], self._cur_arm_feedback_deg[1], self._cur_arm_feedback_deg[2]))
		print("Gripper: {}".format(self._gripper_status))
		print("Compressor: {}".format(self._compressor_status))
		print("Extrusion: {}".format(self._cur_extrusion))
		print("Position: X: {} Y: {} Z: {}".format(self._cur_rigid_arm_config_ts[0], self._cur_rigid_arm_config_ts[1], self._cur_rigid_arm_config_ts[2]))
		print("NOTE: Joint space control does not update position values")
		# #print("Sensor Pose: X: {} Y: {} Z: {}".format(self._sensor_pose[0], self._sensor_pose[1], self._sensor_pose[2]))
		# #print("Sensor Pose Calc: X: {} Y: {} Z: {}".format(self._cur_sensor_pose_calc[0], self._cur_sensor_pose_calc[1], self._cur_sensor_pose_calc[2]))
		# #print("Euclidean Error: {}".format(self._error))
		print("Camera: Pan: {} Tilt: {}".format(self._cur_pan, self._cur_tilt))
		print("*******************************************************************************")
		print("\n")
		#print("0: quit")
		print("1: Gripper open/close")
		print("2: Move arm - Joint Space")
		print("3: Move arm - Theta 4")
		print("4: Pressurize Soft Arm")
		print("5: Extrude Soft Arm")
		print("6: Compressor On/Off")
		print("7: Move arm - Task Space")
		print("8: Move Camera")
		print("9: Home the rigid arm")
		print("10: Ramp Soft Arm")
		print("11: Ramp Joint Space")

	# #def _pose_update_callback(self, msg):
		# #self._sensor_pose = [msg.rigid_arm_pose_w[0]/100.0 , msg.rigid_arm_pose_w[1]/100.0 , msg.rigid_arm_pose_w[2]/100.0 ]# Note this is not really world...
		# #theta1 = np.deg2rad(self._cur_rigid_arm_config[0])
		# #theta2 = np.deg2rad(self._cur_rigid_arm_config[1])
		# #theta3 = np.deg2rad(self._cur_rigid_arm_config[2])
		# #theta4 = np.deg2rad(self._cur_theta_4)

		# #x = (203.0*np.cos(theta1)*np.sin(theta2))/1000.0 - (213*np.cos(theta4)*np.sin(theta1))/10000.0 + (127.0*np.sin(theta1)*np.sin(theta4))/40000.0 - (127.0*np.cos(theta4)*(np.cos(theta1)*np.sin(theta2)*np.sin(theta3) + np.cos(theta1)*np.cos(theta2)*np.cos(theta3)))/40000.0 + np.sin(theta1)*((213.0*np.cos(theta4))/10000.0 - 213.0/10000.0) + np.cos(theta1)*np.sin(theta2)*((47.0*np.cos(theta3))/100.0 - 47.0/100.0) + (7473.0*np.cos(theta1)*np.cos(theta2)*np.sin(theta3))/20000.0 - (16873.0*np.cos(theta1)*np.cos(theta3)*np.sin(theta2))/20000.0
		# #y = (213.0*np.cos(theta1)*np.cos(theta4))/10000.0 - (127.0*np.cos(theta1)*np.sin(theta4))/40000.0 + (203.0*np.sin(theta1)*np.sin(theta2))/1000.0 - (127.0*np.cos(theta4)*(np.sin(theta1)*np.sin(theta2)*np.sin(theta3) + np.cos(theta2)*np.cos(theta3)*np.sin(theta1)))/40000.0 - np.cos(theta1)*((213.0*np.cos(theta4))/10000.0 - 213.0/10000.0) + (7473.0*np.cos(theta2)*np.sin(theta1)*np.sin(theta3))/20000.0 - (16873.0*np.cos(theta3)*np.sin(theta1)*np.sin(theta2))/20000.0 + np.sin(theta1)*np.sin(theta2)*((47.0*np.cos(theta3))/100.0 - 47.0/100.0)
		# #z = (16873.0*np.cos(theta2)*np.cos(theta3))/20000.0 - (203.0*np.cos(theta2))/1000.0 + (7473.0*np.sin(theta2)*np.sin(theta3))/20000.0 - np.cos(theta2)*((47.0*np.cos(theta3))/100.0 - 47.0/100.0) + (127.0*np.cos(theta4)*(np.cos(theta2)*np.sin(theta3) - np.cos(theta3)*np.sin(theta2)))/40000.0 + 203.0/1000.0


		# #self._cur_sensor_pose_calc = [x - 0.068, -1*y + 0.1225, -1*z + 0.0305]
		# #self._error = np.sqrt((self._sensor_pose[0]-self._cur_sensor_pose_calc[0])**2 + (self._sensor_pose[1]-self._cur_sensor_pose_calc[1])**2 + (self._sensor_pose[2]-self._cur_sensor_pose_calc[2])**2)
		# #self._sensor_pose_update_flag = False

	# #def _arm_encoder_callback(self, msg):
		# #self._cur_arm_feedback_deg = [np.rad2deg(msg.arm_encoders_rad[0]), np.rad2deg(msg.arm_encoders_rad[1]), np.rad2deg(msg.arm_encoders_rad[2])]

	def _state_update_callback(self, msg):
		# #print("State update Callback")
		self._cur_rigid_arm_config = [np.rad2deg(msg.joints_rad[0]), np.rad2deg(msg.joints_rad[1]), np.rad2deg(msg.joints_rad[2]), np.rad2deg(msg.theta_4_rad)]
		self._state_update_flag = False

	def _position_rigid_arm(self, config_deg):
		# #print("Publish Message")
		self._cur_rigid_arm_config = copy.copy(config_deg)
		msg = servo_arm()
		msg.servo_arm_rad = [np.deg2rad(config_deg[0]), np.deg2rad(config_deg[1]), np.deg2rad(config_deg[2])]
		self._servo_arm_pub.publish(msg)
		# #print("Done with positioning")


	def _ramp_rigid_arm(self, config_deg, ramp_size, sleep_time):
		# #print("Publish Message")
		# #ramp_size = 30
		start_t1 = self._cur_rigid_arm_config[0]
		start_t2 = self._cur_rigid_arm_config[1]
		start_t3 = self._cur_rigid_arm_config[2]
		step_t1 = (config_deg[0]-start_t1)/ramp_size
		step_t2 = (config_deg[1]-start_t2)/ramp_size
		step_t3 = (config_deg[2]-start_t3)/ramp_size
		self._cur_rigid_arm_config = copy.copy(config_deg)
		for i in range(ramp_size):
			msg = servo_arm()
			t1 = start_t1 + (i+1)*step_t1
			t2 = start_t2 + (i+1)*step_t2
			t3 = start_t3 + (i+1)*step_t3
			msg.servo_arm_rad = [np.deg2rad(t1), np.deg2rad(t2), np.deg2rad(t3)]
			self._servo_arm_pub.publish(msg)
			rospy.sleep(sleep_time)

	def _position_theta_4(self, config_deg):
		self._cur_theta_4 = config_deg
		msg = theta_4_arm()
		msg.theta_4_rad = np.deg2rad(config_deg)
		self._servo_theta_4_pub.publish(msg)

	def _position_rigid_arm_ts(self, config):
		# #print("Publishing in main")
		self._cur_rigid_arm_config_ts = copy.copy(config)
		msg = rigid_arm_position_desired()
		msg.rigid_arm_position_desired = [config[0], config[1], config[2]]
		self._rigid_arm_position_desired_pub.publish(msg)

	def _position_soft_arm(self, config):
		# #print(config)
		self._cur_soft_arm_config = copy.copy(config)
		msg = soft_arm()
		msg.bend_press = config[0]
		msg.rot_1_press = config[1]
		msg.rot_2_press = config[2]
		self._soft_arm_pub.publish(msg)

	def _ramp_soft_arm(self, config):
		self._cur_soft_arm_config = copy.copy(config)
		msg = soft_arm_ramp()
		msg.bend_press = config[0]
		msg.rot_1_press = config[1]
		msg.rot_2_press = config[2]
		self._soft_arm_ramp_pub.publish(msg)


	def _extrude(self):
		input_val = float(raw_input("Length (cm)(Cur: {}): ".format(self._cur_extrusion)) or self._cur_extrusion)
		# #input_val = int(input_val)
		msg = extrusion()
		msg.length_cm = input_val
		self._stepper_control_pub.publish(msg)
		# #self._step.move_stepper(input_val)
		self._cur_extrusion = input_val

	def _position_camera(self, pan_deg, tilt_deg):
		print("Publishing in pi_comm")
		msg = servo_camera()
		msg.camera_pan_rad = np.deg2rad(pan_deg)
		msg.camera_tilt_rad = np.deg2rad(tilt_deg)
		self._servo_camera_pub.publish(msg)


	def _go_home(self):
		self._position_theta_4(0)
		self._position_camera(0, 0)
		self._position_rigid_arm([90, 45, 90])
		rospy.sleep(3)
		self._position_rigid_arm([90, 90, 90])


	def menu_loop(self):
		while not rospy.is_shutdown():
			while(self._state_update_flag == True):
				self._rate.sleep()
			# #while(self._sensor_pose_update_flag == True):
				# #self._rate.sleep()
			self._menu_print()
			selection = raw_input("Input selection: ") or -1 #This will cause the menu to loop if enter is pressed
			selection = int(selection)
			if selection == 1: # gripper
				if(self._gripper_status == "Open"):
					self._gripper_control_pub.publish(1)
					self._gripper_status = "Closed"
					print("Closing gripper")
				else:
					self._gripper_control_pub.publish(0)
					self._gripper_status = "Open"
					print("Opening gripper")
			elif selection == 2: # Move arm
				print("Enter maintains current value")
				t1_input_val = float(raw_input("Theta 1 Value (Cur: {}): ".format(self._cur_rigid_arm_config[0])) or self._cur_rigid_arm_config[0])
				t2_input_val = float(raw_input("Theta 2 Value (Cur: {}): ".format(self._cur_rigid_arm_config[1])) or self._cur_rigid_arm_config[1])
				t3_input_val = float(raw_input("Theta 3 Value (Cur: {}): ".format(self._cur_rigid_arm_config[2])) or self._cur_rigid_arm_config[2])
				# #t4_input_val = float(raw_input("Theta 4 Value (Cur: {}): ".format(self._cur_rigid_arm_config[3])) or self._cur_rigid_arm_config[3])
				self._position_rigid_arm([t1_input_val, t2_input_val, t3_input_val])
			elif selection == 3:
				print("Enter maintains current value")
				t4_input_val = float(raw_input("Theta 4 Value (Cur: {}): ".format(self._cur_theta_4)) or self._cur_theta_4)
				self._position_theta_4(t4_input_val)

			elif selection == 4: # pressurize soft arm
				while (self._cur_extrusion < 2):
					print("Need to extrude!")
					self._extrude()
				print("Enter maintains current value")
				input_bend = float(raw_input("Bend 0-40.0psi (Cur: {}): ".format(self._cur_soft_arm_config[0])) or self._cur_soft_arm_config[0])
				input_r1 = float(raw_input("R1 0-40.0psi (Cur: {}): ".format(self._cur_soft_arm_config[1])) or self._cur_soft_arm_config[1])
				input_r2 = float(raw_input("R2 0-40.0psi (Cur: {}): ".format(self._cur_soft_arm_config[2])) or self._cur_soft_arm_config[2])
				self._position_soft_arm([input_bend, input_r1, input_r2])

			elif selection == 5: # Extrusion
				self._extrude()

			elif selection == 6: # Turn on compressor
				if (self._compressor_status == "Off"):
					self._compressor_control_pub.publish(True)
					self._compressor_status = "On"
					print("Turning compressor On")
				else:
					self._compressor_control_pub.publish(False)
					self._compressor_status = "Off"
					print("Turning compressor Off")

			elif selection == 7:
				x_input_val = float(raw_input("X Value (Cur: {}): ".format(self._cur_rigid_arm_config_ts[0])) or self._cur_rigid_arm_config_ts[0])
				y_input_val = float(raw_input("Y Value (Cur: {}): ".format(self._cur_rigid_arm_config_ts[1])) or self._cur_rigid_arm_config_ts[1])
				z_input_val = float(raw_input("Z Value (Cur: {}): ".format(self._cur_rigid_arm_config_ts[2])) or self._cur_rigid_arm_config_ts[2])
				self._state_update_flag = True
				# #self._sensor_pose_update_flag = True
				self._position_rigid_arm_ts([x_input_val, y_input_val, z_input_val])

			elif selection == 8:
				pan_val_deg= float(raw_input("Pan Value (Cur: {}): ".format(self._cur_pan)) or self._cur_pan)
				tilt_val_deg = float(raw_input("Tilt Value (Cur: {}): ".format(self._cur_tilt)) or self._cur_tilt)
				self._cur_pan = pan_val_deg
				self._cur_tilt = tilt_val_deg
				self._position_camera(pan_val_deg, tilt_val_deg)
			elif selection == 9:
				self._go_home()

			elif selection == 10:
				while (self._cur_extrusion < 2):
					print("Need to extrude!")
					self._extrude()
				print("Enter maintains current value")
				input_bend = float(raw_input("Bend 0-40.0psi (Cur: {}): ".format(self._cur_soft_arm_config[0])) or self._cur_soft_arm_config[0])
				input_r1 = float(raw_input("R1 0-40.0psi (Cur: {}): ".format(self._cur_soft_arm_config[1])) or self._cur_soft_arm_config[1])
				input_r2 = float(raw_input("R2 0-40.0psi (Cur: {}): ".format(self._cur_soft_arm_config[2])) or self._cur_soft_arm_config[2])

				self._ramp_soft_arm([input_bend, input_r1, input_r2])
			elif selection == 11:
				print("Enter maintains current value")
				t1_input_val = float(raw_input("Theta 1 Value (Cur: {}): ".format(self._cur_rigid_arm_config[0])) or self._cur_rigid_arm_config[0])
				t2_input_val = float(raw_input("Theta 2 Value (Cur: {}): ".format(self._cur_rigid_arm_config[1])) or self._cur_rigid_arm_config[1])
				t3_input_val = float(raw_input("Theta 3 Value (Cur: {}): ".format(self._cur_rigid_arm_config[2])) or self._cur_rigid_arm_config[2])
				ramp_size = int(raw_input("Ramp size: "))
				sleep_time = float(raw_input("Sleep time: "))
				# #t4_input_val = float(raw_input("Theta 4 Value (Cur: {}): ".format(self._cur_rigid_arm_config[3])) or self._cur_rigid_arm_config[3])
				self._ramp_rigid_arm([t1_input_val, t2_input_val, t3_input_val], ramp_size, sleep_time)

			else:
				print("Please pick a selection on the list")
			self._rate.sleep()



if __name__ == '__main__':
	rospy.init_node('menu_exec_node', anonymous=True)
	# #rospy.on_shutdown(myhook)
	Menu()

	#rospy.spin()
