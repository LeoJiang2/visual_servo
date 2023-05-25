#!/usr/bin/env python
"""
accumulator.py
Function: This program maintains the pressure in the accumulator in a specified band.  If pressure is very low,
it will cycle the pump on and off periodically so as not to run too long
It is designed to run at a lower rate (2hz) so as not to eat up too much processing time.
Author: Benjamin Walt
Date: 1/27/2021
Purpose: SoftAgBot system integration project
Version: 0.2
"""


"""
General Imports
"""
import rospy
import RPi.GPIO as GPIO

"""
Messages
"""
from pi_comm_pkg.msg import accumulator_pressure
from std_msgs.msg import Bool


_COMPRESSOR_RELAY_PIN = rospy.get_param("/compressor_relay")
_UPPER_LIMIT = 515 # Should correspond to around 50psi
_LOWER_LIMIT = 470 # Should correspond to around 45psi
_RAMP_LOWER_LIMIT = 430
_RAMP_UPPER_LIMIT = 500

class Accumulator_Control:
	"""A class to control the accumulator system and maintain pressure in the desired band."""
	def __init__(self):
		rospy.init_node('accumulator_node', anonymous=True) # This was moved here to allow for the shutdown hook
		rospy.on_shutdown(self._shutdown_hook)
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(_COMPRESSOR_RELAY_PIN, GPIO.OUT)
		GPIO.output(_COMPRESSOR_RELAY_PIN, 0)
		self._compressor_status = False # Compressor is off
		self._pressure = -1
		self._maintain_flag = False
		self._ramp_done = True # Not ramping
			
		#######################
		# Subscribers
		#######################
		rospy.Subscriber('pi_comm/accumulator_pressure', accumulator_pressure, self._accumulator_pressure_callback)
		rospy.Subscriber('accumulator_enable', Bool, self._accumulator_enable_callback)
		
		#######################
		# Publishers
		#######################
		self._accumulator_pressure_call_pub = rospy.Publisher('pi_comm/accumulator_pressure_call', Bool, queue_size=10)
		self._rate = rospy.Rate(2)
		self._maintain_pressure()
	
	"""
	This hook is called when the system shuts down to ensure that the compressor is
	turned off.  If it fails or is not used the compressor will stay on when the 
	program is shutdown.
	"""
	def _shutdown_hook(self):
		GPIO.output(_COMPRESSOR_RELAY_PIN, 0)
		GPIO.cleanup()
		print("Compressor Shutdown!")
	
	"""
	This callback gets the ADC reading from the pressure sensor.
	"""
	def _accumulator_pressure_callback(self, msg):
		# This doesn't get the actual pressure, but just the ADC reading
		self._pressure = msg.accumulator_pressure_digital

	"""
	This gently increase pressure so that the compressor does not operate for 
	an extended period of time.  Especially as pressure increases, the current 
	draw can be quite high.  It is used on start up and if pressure drops very low.
	"""
	def _ramp_up(self):
		print("Ramping Pressure")
		counter = 0
		self._ramp_done = False
		compressor_call = False
		start_time = rospy.get_time()
		while(self._ramp_done == False):
			if((rospy.get_time() - start_time) > 2):
				#read pressure
				if(counter <= 5):
					self._pressure = -1
					self._accumulator_pressure_call_pub.publish(1) # Call for pressure reading
					while(self._pressure == -1):
						pass
				start_time = rospy.get_time()
				counter += 1
			
			if(counter <= 5):
				if(self._pressure > _RAMP_UPPER_LIMIT):
					self._ramp_done = True
					compressor_call = False
				else:
					compressor_call = True
				
			elif(counter > 5 and counter <=7):
				compressor_call = False
			elif(counter > 7):
				counter = 0
					
			if(compressor_call != self._compressor_status):
				GPIO.output(_COMPRESSOR_RELAY_PIN, compressor_call)
				self._compressor_status = compressor_call
			self._rate.sleep()
			
	"""
	This is the core of the program.  It maintains pressure in the system between two points.
	The points are based on ADC readings and not the specific pressures.  They have been selected 
	to maintain pressure between about 45 and 50 PSI. 
	The loop is limited to ensure it is not wasting time.  Pressure is only checked every 2 seconds
	and loop rate is 2Hz
	"""
	def _maintain_pressure(self):
		
		start_time = rospy.get_time()
		compressor_call = False
		while(not rospy.is_shutdown()):
			while(self._maintain_flag == True):
				if((rospy.get_time() - start_time) > 2):
					self._pressure = -1
					self._accumulator_pressure_call_pub.publish(1) # Call for pressure reading
					while(self._pressure == -1):
						pass 
					if(self._pressure < _RAMP_LOWER_LIMIT):
						self._ramp_up()
						self._pressure = -1
						self._accumulator_pressure_call_pub.publish(1) # Call for pressure reading
						while(self._pressure == -1):
							pass
					if(self._pressure < _LOWER_LIMIT):
						compressor_call = True
					elif(self._pressure > _UPPER_LIMIT):
						compressor_call = False
					if(compressor_call != self._compressor_status):
						GPIO.output(_COMPRESSOR_RELAY_PIN, compressor_call)
						self._compressor_status = compressor_call
					start_time = rospy.get_time()
					# #print(self._pressure)
				self._rate.sleep()
			self._rate.sleep()

		
	"""
	This callback allows other programs to start and stop the compressor system
	"""
	def _accumulator_enable_callback(self, msg):
		if(msg.data == True):
			print("Maintaining Pressure")
			self._maintain_flag = True
		else:
			self._ramp_done = True # Stop ramp as well
			self._maintain_flag = False # Should end the _maintain_pressure loop
			rospy.sleep(1)
			GPIO.output(_COMPRESSOR_RELAY_PIN, 0)
			self._compressor_status = False


if __name__ == '__main__':
	Accumulator_Control()

