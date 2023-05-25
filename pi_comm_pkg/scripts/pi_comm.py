#!/usr/bin/env python
"""
pi_comm.py
Function: This program handles all of the i2c and spi communication for the SoftAgBot.
As commands arrive, they are queued up to prevent more than one device trying to use the bus at the same time.
This is a bigger issue due to our use of the multiplexor that only allows one device to talk at a time.
Author: Benjamin Walt
Date: 02/23/2021
Purpose: SoftAgBot system integration project
Version: 0.3
"""


"""
General Imports 
"""
import rospy
import Queue as queue # This creates a queue that is protected with locks
import sys
import RPi.GPIO as GPIO
import numpy as np
import time

"""
Equipment classes
"""
from pi_comm_pkg import TCA9548A as TCA # This is the multiplexor
from pi_comm_pkg import PCA9685 as PCA # PWM driver
from pi_comm_pkg import MCP4725 as MCP_DAC # DAC
from pi_comm_pkg import MCP3008 as MCP_ADC # This is the ADC
# #from pi_comm_pkg import ADS1115 as ADS # This is the feedback ADC
from pi_comm_pkg import StepperMotor as SM

"""
Messages
"""
from pi_comm_pkg.msg import gripper_reply
from pi_comm_pkg.msg import soft_arm
from pi_comm_pkg.msg import soft_arm_ramp
from pi_comm_pkg.msg import servo_arm
from pi_comm_pkg.msg import theta_4_arm
from pi_comm_pkg.msg import servo_camera
from pi_comm_pkg.msg import soft_arm_pressure
from pi_comm_pkg.msg import gripper_pressure
from pi_comm_pkg.msg import accumulator_pressure
from pi_comm_pkg.msg import extrusion
from pi_comm_pkg.msg import arm_state
# #from pi_comm_pkg.msg import arm_encoders
from std_msgs.msg import Bool

"""
Setting up pin data from pin_out.yaml
"""
# Multiplexor Channels
_DAC_1_channel = rospy.get_param("/DAC_1")
_DAC_2_channel = rospy.get_param("/DAC_2")
_DAC_3_channel = rospy.get_param("/DAC_3")
_DAC_4_channel = rospy.get_param("/DAC_4")
_SERVO_channel = rospy.get_param("/SERVO")
_ADS1115_channel = rospy.get_param("/ADS1115_ADC")
# Servo Channels
_CAMERA_PAN_channel = rospy.get_param("/CAMERA_PAN")
_CAMERA_TILT_channel = rospy.get_param("/CAMERA_TILT")
_THETA_1_channel = rospy.get_param("/THETA_1") #Body
_THETA_2_channel = rospy.get_param("/THETA_2") #Shoulder
_THETA_3_channel = rospy.get_param("/THETA_3") #Elbow
_THETA_4_channel = rospy.get_param("/THETA_4") #Wrist
# ADC Channels
_REG_1_channel = rospy.get_param("/reg_1")
_REG_2_channel = rospy.get_param("/reg_2")
_REG_3_channel = rospy.get_param("/reg_3")
_REG_4_channel = rospy.get_param("/reg_4") # reserved for gripper, is it needed?
_ACCUMULATOR_channel = rospy.get_param("/accumulator") 

"""
Setting up pin data from parameters.yaml
"""
_OSCILLATOR_FREQ = rospy.get_param("/oscillator_freq") # For PCA9685
_GRIPPER_PRESSURE = rospy.get_param("/gripper_pressure") # For gripper pressure


_STEP_PIN = rospy.get_param("/bed_step")
_DIRECTION_PIN = rospy.get_param("/bed_dir")
_SWITCH_PIN = rospy.get_param("/limit_switch")


_THETA_1_CORRECTION = np.deg2rad(rospy.get_param("/theta_1_correction"))
_THETA_2_CORRECTION = np.deg2rad(rospy.get_param("/theta_2_correction"))
_THETA_3_CORRECTION = np.deg2rad(rospy.get_param("/theta_3_correction"))
_THETA_4_CORRECTION = np.deg2rad(rospy.get_param("/theta_4_correction"))

_PAN_CORRECTION = np.deg2rad(rospy.get_param("/pan_correction"))
_TILT_CORRECTION = np.deg2rad(rospy.get_param("/tilt_correction"))

_DAC_ADDRESS = rospy.get_param("/dac_address")

class Pi_Communications:

	def __init__(self):
		rospy.init_node('pi_comm_node', anonymous=True)
		rospy.on_shutdown(self._shutdown_hook)
		self._step = SM.StepperMotor(_STEP_PIN, _DIRECTION_PIN, _SWITCH_PIN)
			
		self._queue = queue.Queue() # This is a FILO used to store actions in the order they are recieved
		
		# Initilize all the classes
		self._tca = TCA.TCA9548A()
		# The DACs
		self._tca.select_i2c_device(_DAC_1_channel)
		self._MCP_1 = MCP_DAC.MCP4725(_DAC_ADDRESS)
		self._tca.select_i2c_device(_DAC_2_channel)
		self._MCP_2 = MCP_DAC.MCP4725(_DAC_ADDRESS)
		self._tca.select_i2c_device(_DAC_3_channel)
		self._MCP_3 = MCP_DAC.MCP4725(_DAC_ADDRESS)
		self._tca.select_i2c_device(_DAC_4_channel)
		self._MCP_4 = MCP_DAC.MCP4725(_DAC_ADDRESS)

		# The servo controller
		self._tca.select_i2c_device(_SERVO_channel)
		self._pwm = PCA.PCA9685(_OSCILLATOR_FREQ)
		pan_pwm = self._camera_servo_rad2pwm(0 + _PAN_CORRECTION) # Correction fixes any disparity in the zero position
		tilt_pwm = self._camera_servo_rad2pwm(0 + _TILT_CORRECTION)
		self._pwm.set_pwm(_CAMERA_PAN_channel, 0, pan_pwm)
		self._pwm.set_pwm(_CAMERA_TILT_channel, 0, tilt_pwm)
		
		# The feedback ADC
		# #self._tca.select_i2c_device(_ADS1115_channel)
		# #self._ads = ADS.ADS1115()
		
		# The ADC
		self._mcp = MCP_ADC.MCP3008()
		
		#######################
		# Subscribers
		#######################
		rospy.Subscriber("gripper/sensor", Bool, self._sensor_callback)
		rospy.Subscriber('pi_comm/servo_arm', servo_arm, self._servo_arm_callback)
		rospy.Subscriber('pi_comm/theta_4_arm', theta_4_arm, self._theta_4_arm_callback)
		rospy.Subscriber('pi_comm/gripper_control', Bool, self._gripper_callback)
		rospy.Subscriber('pi_comm/servo_camera', servo_camera, self._camera_callback)
		rospy.Subscriber('pi_comm/soft_arm', soft_arm, self._soft_arm_callback)
		rospy.Subscriber('pi_comm/soft_arm_ramp', soft_arm_ramp, self._soft_arm_ramp_callback)
		rospy.Subscriber('pi_comm/set_extrusion', extrusion, self._extrusion_callback)
		rospy.Subscriber('pi_comm/compressor', Bool, self._compressor_callback)
		rospy.Subscriber('pi_comm/arm_pressure_call', Bool, self._soft_arm_pressure_callback)
		rospy.Subscriber('gripper/pressure', Bool, self._gripper_pressure_callback)
		rospy.Subscriber('pi_comm/accumulator_pressure_call', Bool, self._accumulator_pressure_callback)
		
		
		#######################
		# Publishers
		#######################
		self._gripper_sensor_reply_pub = rospy.Publisher('pi_comm/gripper_sensor_reply', gripper_reply, queue_size = 10)	
		self._soft_arm_pressure_pub = rospy.Publisher('pi_comm/soft_arm_pressure', soft_arm_pressure, queue_size = 10)
		self._gripper_pressure_pub = rospy.Publisher('pi_comm/gripper_pressure', gripper_pressure, queue_size = 10)
		self._accumulator_pressure_pub = rospy.Publisher('pi_comm/accumulator_pressure', accumulator_pressure, queue_size = 10)
		self._arm_state_pub = rospy.Publisher('pi_comm/arm_state', arm_state, queue_size = 10)
		# #self._arm_encoders_pub = rospy.Publisher('pi_comm/arm_encoders', arm_encoders, queue_size = 10)
		self._extruder_status_pub = rospy.Publisher('pi_comm/extruder_status', Bool, queue_size = 10)
		
		#######################
		# State Data
		#######################
		self._joints = [0, 0, 0]
		self._theta_4 = 0
		self._cur_bending_pressure = 0.0
		self._cur_r1_pressure = 0.0
		self._cur_r2_pressure = 0.0
		self._extrusion_length_cm = 0.0
		self._gripper_status = False
		self._update_flag = False
		self._extruder_state_flag = True
		self._arm_feedback = [0, 0, 0]
		self._soft_ramp_queue = []
		self._soft_ramp_flag = False
		self._step_start_time = 0.0
		
		self._rate = rospy.Rate(20)
		self._start_queue()

	"""
	This code is run when ros shuts down.  It is selected to help return the robot to a stable condition.
	It is not guaranteed to run as other parts may shutdown before it runs.
	"""
	def _shutdown_hook(self):
		print("pi_comm Shutting Down")
		self._tca.select_i2c_device(_SERVO_channel)
		# Does the order help?  I want to avoid spurious movement during shutdown
		# Especially for theta 1 as it tends to crash
		# I assume this happens due to improper shutdown - i.e. mixed signals as things shutdown
		# My hope is that by doing theta 4 first, it will experience any spurious motion
		# Which is not a big deal
		self._pwm.set_pwm(_THETA_4_channel, 0, 0) 
		self._pwm.set_pwm(_THETA_3_channel, 0, 0)
		self._pwm.set_pwm(_THETA_2_channel, 0, 0)
		self._pwm.set_pwm(_THETA_1_channel, 0, 0)
		self._tca.select_i2c_device(_DAC_1_channel)
		self._MCP_1.set_dac_voltage(0.0)
		self._tca.select_i2c_device(_DAC_2_channel)
		self._MCP_2.set_dac_voltage(0.0)
		self._tca.select_i2c_device(_DAC_3_channel)
		self._MCP_3.set_dac_voltage(0.0)
		self._tca.select_i2c_device(_DAC_4_channel)
		self._MCP_4.set_dac_voltage(0.0)
		
	"""
	Callback that controls the length of the softarm.
	It also updates and publishes the arm state topic.
	"""
	def _extrusion_callback(self, msg):
		# #print("Extrusion")
		self._step.move_stepper(msg.length_cm)
		self._extrusion_length_cm = msg.length_cm
		msg = arm_state()
		msg.joints_rad = self._joints
		msg.theta_4_rad = self._theta_4
		msg.soft_arm_pressure = [self._cur_bending_pressure, self._cur_r1_pressure, self._cur_r2_pressure]
		msg.extrusion_length_cm =  self._extrusion_length_cm
		msg.gripper_status = self._gripper_status
		self._arm_state_pub.publish(msg)

	"""
	Callback controls the compressor
	"""
	def _compressor_callback(self, msg):
		GPIO.output(_COMPRESSOR_RELAY_PIN, msg.data)

	"""
	Callback that adds a gripper sesnor call to the queue.
	"""
	def _sensor_callback(self, msg):
		# Add to queue
		self._queue.put(["SENSORS"])
		
	"""
	Callback that adds a call to the queue that changes the soft arm pressure.
	Updates the arm state
	"""
	def _soft_arm_callback(self, msg):
		bend = self._pressure2volts(msg.bend_press)
		rot_1 = self._pressure2volts(msg.rot_1_press)
		rot_2 = self._pressure2volts(msg.rot_2_press)
		# Update the state
		self._cur_bending_pressure = msg.bend_press
		self._cur_r1_pressure = msg.rot_1_press
		self._cur_r2_pressure = msg.rot_2_press
		# Raise flag to publish the update
		self._update_flag = True
		# Add to queue
		self._queue.put(["SOFT_ARM", bend, rot_1, rot_2])  
		
		"""
	Callback that initiates a ramping action for the soft arm.  This is not part of the general queue.
	It creates a new queue that is executed when the general queue is not busy.  This prevents it from
	tying up the general queue.  It executes based on elapsed time.
	Updates the arm state
	"""
	def _soft_arm_ramp_callback(self, msg):
		# #print("Call Back")
		goal_bend = msg.bend_press
		goal_rot_1 = msg.rot_1_press
		goal_rot_2 = msg.rot_2_press
		# Create List
		delta_bending = goal_bend - self._cur_bending_pressure
		delta_r1 = goal_rot_1 - self._cur_r1_pressure
		delta_r2 = goal_rot_2 - self._cur_r2_pressure
		max_delta = abs(max([delta_bending, delta_r1, delta_r2], key=abs)) #Returns the max absolute value of the changes
		# #print("Max Delta: {}".format(max_delta))
		step_size = 0.4 # 1psi
		steps = int(max_delta/step_size)
		bend_steps = np.linspace(step_size, delta_bending, steps)
		r1_steps = np.linspace(step_size, delta_r1, steps)
		r2_steps = np.linspace(step_size, delta_r2, steps)
		for i in range(steps):
			bending_volts = self._pressure2volts(self._cur_bending_pressure+bend_steps[i])
			r1_volts = self._pressure2volts(self._cur_r1_pressure+r1_steps[i])
			r2_volts = self._pressure2volts(self._cur_r2_pressure+r2_steps[i])
			self._soft_ramp_queue.append([bending_volts, r1_volts, r2_volts])

		# Set soft_ramp_flag
		self._soft_ramp_flag = True
		
		# Update the state
		self._cur_bending_pressure = msg.bend_press
		self._cur_r1_pressure = msg.rot_1_press
		self._cur_r2_pressure = msg.rot_2_press

		
	"""
	Callback that adds a call to the queue to change the arm position
	Updates the arm state
	"""
	def _servo_arm_callback(self, msg):
		theta_1_pwm = self._arm_servo_rad2pwm(msg.servo_arm_rad[0] + _THETA_1_CORRECTION) # Correction fixes any disparity in the zero position
		theta_2_pwm = self._arm_servo_rad2pwm(msg.servo_arm_rad[1] + _THETA_2_CORRECTION)
		theta_3_pwm = self._arm_servo_rad2pwm(msg.servo_arm_rad[2] + _THETA_3_CORRECTION)
		# Update the state
		self._joints = [msg.servo_arm_rad[0], msg.servo_arm_rad[1], msg.servo_arm_rad[2]]
		self._update_flag = True
		# Add to queue
		self._queue.put(["ARM_SERVO",theta_1_pwm, theta_2_pwm, theta_3_pwm])

	"""
	Callback that adds a call to the queue to change the theta 4 position
	Updates the arm state
	"""		
	def _theta_4_arm_callback(self, msg):
		# theta_4 is a different kind of servo
		theta_4_pwm = self._theta_4_servo_rad2pwm(msg.theta_4_rad + _THETA_4_CORRECTION) # Correction fixes any disparity in the zero position
		self._theta_4 = msg.theta_4_rad
		self._update_flag = True
		# Add to queue
		self._queue.put(["THETA_4_SERVO",theta_4_pwm])

	"""
	Callback that adds a call to the queue to open or close the gripper depending on the boolean value sent.
	"""
	def _gripper_callback(self, msg):
		if msg.data == True:
			voltage = self._pressure2volts(_GRIPPER_PRESSURE) # Gripper shut
		else:
			voltage = 0.0 # Gripper open
		# Add to queue
		self._queue.put(["GRIPPER",voltage])


	"""
	Callback that adds a call to the queue to change the camera pose
	"""
	def _camera_callback(self, msg):
		pan_pwm = self._camera_servo_rad2pwm(msg.camera_pan_rad + _PAN_CORRECTION) # Correction fixes any disparity in the zero position
		tilt_pwm = self._camera_servo_rad2pwm(msg.camera_tilt_rad + _TILT_CORRECTION)
		# Add to queue
		self._queue.put(["CAMERA_SERVO", pan_pwm, tilt_pwm])


	"""
	Converts desired psi to a voltage between 0-5VDC for the DAC
	Assumes a 70 psi pressure regulator
	"""
	def _pressure2volts(self, pressure_psi):
		MAX_REG_PRESSURE = 70.0
		DAC_VOLTAGE = 5.0
		volts = (pressure_psi/MAX_REG_PRESSURE)*DAC_VOLTAGE
		volts = max(0.0, min(DAC_VOLTAGE, volts))
		return volts

	"""
	Pan and Tilt Servos only
	Coverts the desired position of the servo in radians to a PWM signal
	Only works for the specific type of servo used
	"""
	def _camera_servo_rad2pwm(self, radians):
		# For HS488HB Servo
		#MIN_USEC = 553.0
		#MAX_USEC = 2425.0
		#RANGE_RAD = 3.3161
		# For HS-7950TH Servo
		MIN_USEC = 750.0
		MAX_USEC = 2250.0
		RANGE_RAD = np.deg2rad(150.)
		
		
		usec_per_rad = (MAX_USEC-MIN_USEC)/RANGE_RAD
		usec = ((MAX_USEC-MIN_USEC)/2.0) + MIN_USEC + radians*usec_per_rad
		usec = max(MIN_USEC, min(MAX_USEC, usec))
		pwm = int(round((usec/20000.0)*4095)) # 50hz -> 20ms pulse width
		return pwm

	"""
	Theta_1 through Theta_3 Servos only
	Coverts the desired position of the servo in radians to a PWM signal
	Only works for the specific type of servo used
	"""
	def _arm_servo_rad2pwm(self, radians):
		# For cm-d950tw-400 Servo
		MIN_USEC = 750.0
		MAX_USEC = 2250.0
		RANGE_RAD = 5.5676
		usec_per_rad = (MAX_USEC-MIN_USEC)/RANGE_RAD
		usec = ((MAX_USEC-MIN_USEC)/2.0) + MIN_USEC + radians*usec_per_rad
		usec = max(MIN_USEC, min(MAX_USEC, usec))
		pwm = int(round((usec/20000.0)*4095)) # 50hz -> 20ms pulse width
		return pwm
	

	"""
	Theta_4 Servo only
	Coverts the desired position of the servo in radians to a PWM signal
	Only works for the specific type of servo used
	"""
	def _theta_4_servo_rad2pwm(self, radians):
		# For DS 3218 Servo
		MIN_USEC = 500.0
		MAX_USEC = 2500.0
		RANGE_RAD = 4.7124
		usec_per_rad = (MAX_USEC-MIN_USEC)/RANGE_RAD
		usec = ((MAX_USEC-MIN_USEC)/2.0) + MIN_USEC + radians*usec_per_rad
		usec = max(MIN_USEC, min(MAX_USEC, usec))
		pwm = int(round((usec/20000.0)*4095)) # 50hz -> 20ms pulse width
		return pwm
	
	"""
	The following are for the MCP3008 ADC
	"""
	"""
	Callback that adds a call to the queue to obtain the ADC readings of the 3 pressure regulators used for the soft arm
	"""
	def _soft_arm_pressure_callback(self, msg):
		self._queue.put(["SOFT_ARM_PRESS"])  

	"""
	Callback that adds a call to the queue to obtain the ADC readings of the pressure regulator used for the gripper
	"""
	def _gripper_pressure_callback(self, msg):
		self._queue.put(["GRIPPER_PRESS"])

	"""
	Callback that adds a call to the queue to obtain the ADC readings of the pressure sensor used in the accumulator
	"""
	def _accumulator_pressure_callback(self, msg):
		self._queue.put(["ACCUMULATOR"])

	"""
	Converts the digital output of the ADC to a pressure value in PSI
	"""
	def _dig2psi(self, value):
		return (value/1023.0)*70.0
		
	"""
	Converts the digital output of the ADS1115 ADC to a radians value
	"""	
	def _adc2rad(self, values):
		theta_1 = 0.00051289209919442*values[0] + -4.11321820331221
		theta_2 = 0.000519335887852482*values[1] + -4.15762037945415
		theta_3 = 0.000513872620129838*values[2] + -4.12141779794543
		return [theta_1, theta_2, theta_3]
		
	"""
	Publishes the status of the extrusion - if it is moving or stationary
	"""	
	def _extrusion_status_update(self):
		msg = self._step.get_status() # False is stopped, True is moving
		# Publish message
		self._extruder_status_pub.publish(msg)
		
	"""
	Publishes the arm encoder vales from the ADS1115 ADC
	"""	
	def _get_arm_encoders(self):
		self._tca.select_i2c_device(_ADS1115_channel)
		feedback = [self._ads.read_adc_difference(1), self._ads.read_adc_difference(2), self._ads.read_adc_difference(3)]
		self._arm_feedback = self._adc2rad(feedback)
		# #print(self._arm_feedback)
		msg = arm_encoders()
		msg.arm_encoders_rad = self._arm_feedback
		self._arm_encoders_pub.publish(msg)

	"""
	This loop operates based on the queus. As items are added to the queue the are executed in FILO order.  When empty, it only does some housekeeping tasks and is otherwise
	sleeping.
	"""
	def _start_queue(self):
		while(not rospy.is_shutdown()):
			while(not self._queue.empty()):
				action = self._queue.get() # Should be a list of at least the device name and also any command values if needed
				device = action[0] # First element should contain the device name
								
				if(device == "SOFT_ARM"): # Control soft arm
					if(self._soft_ramp_flag):
						print("Error: Currently ramping soft arm")
					else:
						self._tca.select_i2c_device(_DAC_1_channel)
						self._MCP_1.set_dac_voltage(action[1])
						self._tca.select_i2c_device(_DAC_2_channel)
						self._MCP_2.set_dac_voltage(action[2])
						self._tca.select_i2c_device(_DAC_3_channel)
						self._MCP_3.set_dac_voltage(action[3])
					
				elif(device == "GRIPPER"): # Control gripper
					self._tca.select_i2c_device(_DAC_4_channel)
					self._MCP_4.set_dac_voltage(action[1])	
					
				elif(device == "ARM_SERVO"): # Control rigid arm
					# #print("Move Arm")
					self._tca.select_i2c_device(_SERVO_channel)
					self._pwm.set_pwm(_THETA_1_channel, 0, action[1])
					self._pwm.set_pwm(_THETA_2_channel, 0, action[2])
					self._pwm.set_pwm(_THETA_3_channel, 0, action[3])
					
				elif(device == "THETA_4_SERVO"): # Control rigid arm theta 4
					# print("Moving camera")
					self._tca.select_i2c_device(_SERVO_channel)
					self._pwm.set_pwm(_THETA_4_channel, 0, action[1])
					
				elif(device == "CAMERA_SERVO"): # Control camera P&T
					self._tca.select_i2c_device(_SERVO_channel)
					self._pwm.set_pwm(_CAMERA_PAN_channel, 0, action[1])
					self._pwm.set_pwm(_CAMERA_TILT_channel, 0, action[2])
						
				elif(device == "SOFT_ARM_PRESS"): # Read soft arm pressures
					temp = [-1.0,-1.0,-1.0]
					msg = soft_arm_pressure()
					i = 0 # Used to iterate properly through message array
					for reg in [_REG_1_channel, _REG_2_channel, _REG_3_channel]:
						temp[i] = self._dig2psi(self._mcp.read_analog_input(reg))
						print(i)
						print(temp[i])
						i += 1
					msg.soft_arm_pressure_psi = temp
					self._soft_arm_pressure_pub.publish(msg)
					
				elif(device == "GRIPPER_PRESS"): # Read gripper pressure
					msg = gripper_pressure()
					msg.gripper_pressure_psi = self._dig2psi(self._mcp.read_analog_input(_REG_4_channel))
					self._gripper_pressure_pub.publish(msg)
					
				elif(device == "ACCUMULATOR"): # Read accumulator pressure
					msg = accumulator_pressure()
					press = self._mcp.read_analog_input(_ACCUMULATOR_channel)
					#print(press)
					msg.accumulator_pressure_digital = press #self._mcp.read_analog_input(_ACCUMULATOR_channel)
					self._accumulator_pressure_pub.publish(msg)
				
				if(self._update_flag): # Update arm state if flag is set
					msg = arm_state()
					msg.joints_rad = self._joints
					msg.theta_4_rad = self._theta_4
					msg.soft_arm_pressure = [self._cur_bending_pressure, self._cur_r1_pressure, self._cur_r2_pressure]
					msg.extrusion_length_cm =  self._extrusion_length_cm
					msg.gripper_status = self._gripper_status
					self._arm_state_pub.publish(msg)
					update_flag = False
			"""
			This is outside the queue and only happens when the queue is empty.
			Until the ramp queue is empty, it acts every 0.x seconds based on elapsed time.
			"""	
			if(self._soft_ramp_flag): # Take a step on the ramp based on timing
				# Check if queue is empty
				# #print("Ramping soft")
				if(len(self._soft_ramp_queue) == 0):
					self._step_start_time = 0
					self._soft_ramp_flag = False
				else:
					# Check if enough time has passed
					delta_time = time.time()- self._step_start_time
					# #print(delta_time)
					if(delta_time > 0.05): #Only act every tenth of a second
						
						#Take a step if okay
						# #print("Stepping")
						cur_step = self._soft_ramp_queue.pop(0)
						self._tca.select_i2c_device(_DAC_1_channel)
						self._MCP_1.set_dac_voltage(cur_step[0])
						self._tca.select_i2c_device(_DAC_2_channel)
						self._MCP_2.set_dac_voltage(cur_step[1])
						self._tca.select_i2c_device(_DAC_3_channel)
						self._MCP_3.set_dac_voltage(cur_step[2])
						self._step_start_time = time.time()
							
							
					
					
					
					
			
			# #self._get_arm_encoders()
			self._extrusion_status_update()
						
			self._rate.sleep()



if __name__ == '__main__':
	Pi_Communications()
