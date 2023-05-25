#!/usr/bin/env python
"""
MCP4725.py
Function: A simple class to operate an MCP4725 DAC on an RPi with Python
Author: Benjamin Walt
Date: 9/29/2020
Purpose: SoftAgBot system integration project
Version: 0.2
This is a rework of Adafruit's software:
https://github.com/adafruit/Adafruit_MCP4725
https://www.adafruit.com/product/935
"""
import smbus

# Address depends on manufacturer and settings (A0)
# 0x62 is adafruit
# 0x60 is sparkfun

"""
'Register' Command format
# C2|C1|C0|X|X|PD1|PD0|X
'Write DAC Register' C2 = 0, C1 = 1, C0 = 0, PD1 = 0, PD0 = 0
'Write DAC Register and EEPROM' C2 = 0, C1 = 1, C0 = 1, PD1 = 0, PD0 = 0
"""

_REG_WRITE_DAC = 0x40 # 0b01000000 - This is the 'Write DAC Register' command - it is not really a register
_REG_WRITE_EEPROM = 0x60 # 0b01100000 - This is the 'Write DAC Register and EEPROM' command - it is not really a register

class MCP4725():
	
	def __init__(self, address):
		self._bus = smbus.SMBus(1) # Channel = 1
		self._address = address
		self._write_eeprom_voltage(0.0) # Write to the EEPROM so it will always restart with 0VDC output

	def _write_reg(self, reg, value):
		self._bus.write_i2c_block_data(self._address, reg, value)
		
	"""
	This function write to the DAC register to set the output voltage.
	It is the 'Write DAC Register' command
	"""
	def set_dac_voltage(self, voltage):
		digital_val = int((voltage/5.0)*4095) # Create a value btween 0 and 4095
		digital_val = max(0, min(4095, digital_val))
		
		# Shift everything left by 4 bits and separate bytes
		upper = (digital_val & 0xff0) >> 4 # Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
		lower = (digital_val & 0xf) << 4 # Lower data bits (D3.D2.D1.D0.x.x.x.x)
		msg = [upper, lower]
		self._write_reg(_REG_WRITE_DAC, msg)

	"""
	This function writes to the DAC register and the EEPROM.  It is the 'Write DAC Register 
	and EEPROM' command  This is used so that when the DAC is powered up, it starts 
	with the given voltage output. 
	WARNING: It should not be used as the normal write command.  The EEPROM have a limited
	number of write cycles.
	"""
	def _write_eeprom_voltage(self, voltage):
		digital_val = int((voltage/5.0)*4095) # Create a value btween 0 and 4095
		digital_val = max(0, min(4095, digital_val))
		
		# Shift everything left by 4 bits and separate bytes
		upper = (digital_val & 0xff0) >> 4 # Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
		lower = (digital_val & 0xf) << 4 # Lower data bits (D3.D2.D1.D0.x.x.x.x)
		msg = [upper, lower]
		self._write_reg(_REG_WRITE_EEPROM, msg)

