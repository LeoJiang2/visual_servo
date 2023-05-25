#!/usr/bin/env python
"""
TCA9548A.py
Function: A simple class to operate a TCA9548A I2C multiplexor on an RPi with Python
Author: Benjamin Walt
Date: 9/30/2020
Purpose: SoftAgBot system integration project
Version: 0.2
This is a rework of Adafruit's software:
https://github.com/adafruit/Adafruit_CircuitPython_TCA9548A
https://www.adafruit.com/product/2717
"""
import smbus

_DEVICE_ADDRESS = 0x70


class TCA9548A():
	"""Class to control the TCA9548A I2C multiplexor"""
	def __init__(self):
		self._address = _DEVICE_ADDRESS
		try:
			self._bus = smbus.SMBus(1) # Channel = 1
			if(self._bus.read_byte(self._address) == 0):
				print("Connected to TCA9548A")
		except:
			print("Failed to connect to TCA9548A")
		
		self._current_channel = 0
		
	def __del__(self):
		self._bus.close()

	def select_i2c_device(self, i2c_channel):
		"""Select the desired channel 0-7"""
		if i2c_channel < 0 or i2c_channel >7:
			print("TCA9548A channel out of range.  Cannot set to {}".format(i2c_channel))
			i2c_channel = self._current_channel
		else:
			self._channel = i2c_channel
		new_device = 0x01 << i2c_channel
		self._bus.write_byte(self._address, new_device)
