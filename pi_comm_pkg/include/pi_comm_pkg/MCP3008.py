#!/usr/bin/env python
"""
MCP3008.py
Function: A simple class to operate a MCP3008 on SPI on an RPi with Python
MCP3008 is an 8-Channel, 10-bit ADC
Author: Benjamin Walt
Date: 10/05/2020
Purpose: SoftAgBot system integration project
Version: 0.1
This is based on the following software:
https://electronicshobbyists.com/raspberry-pi-analog-sensing-mcp3008-raspberry-pi-interfacing/
"""

import spidev
_BUS = 0 # Pi has multiple SPI buses, we are using 0
_CHIP_ENABLE = 0 # Pin GPIO 8(CE0)
_MAX_HZ_5V = 3600000 # Max frequency when using 5v logic
_MAX_HZ_3V = 1350000 # Max frequency when using 3v logic
_MAX_HZ = _MAX_HZ_3V
_MODE = 0
"""
MOSI GPIO 10 -> DIN
MISO GPIO 9 -> DOUT
SCLK GPIO 11 -> CLK
CE0  GPIO 8 -> CS
CE1 GPIO 7 -> CS
MSB First
SPI Mode 0 -> CPOL = 0, CPHA = 0
"""

class MCP3008():
	"""A class used to set up and control the MCP3008, an 8-Channel, 10-bit ADC"""
	def __init__(self):
		self._spi = spidev.SpiDev()
		self._spi.open(_BUS,_CHIP_ENABLE)
		self._spi.max_speed_hz = _MAX_HZ
		self._spi.mode = _MODE
	
	
	def __del__(self):
		self._spi.close()

	def read_analog_input(self, channel):
		"""Read the analog data from a given channel 0-1023"""
		# Transfer is Byte 1: 0x01
		# Byte 2: bit[7]= sing=1/diff=0, bit[6:4]= channel, bit[3:0]=0
		# Byte 3: 0x00 - This is a place holder for return message
		adc = self._spi.xfer2([1,(0x08|channel)<<4,0])
		data = ((adc[1]&3) << 8) + adc[2]
		return data
		
	def read_analog_input_diff(self, channel):
		"""Read the analog data differential between two channels 0-1023"""
		"""
		0: 0-1
		1: 1-0
		2: 2-3
		3: 3-2
		4: 4-5
		5: 5-4
		6: 6-7
		7: 7-8
		"""
		# Transfer is Byte 1: 0x01
		# Byte 2: bit[7]= sing=1/diff=0, bit[6:4]= channel, bit[3:0]=0
		# Byte 3: 0x00 - This is a place holder for return message
		adc = self._spi.xfer2([1,(0x00|channel)<<4,0])
		data = ((adc[1]&3) << 8) + adc[2]
		return data
		
	def close_SPI(self):
		"""Close the SPI connection"""
		self._spi.close()
