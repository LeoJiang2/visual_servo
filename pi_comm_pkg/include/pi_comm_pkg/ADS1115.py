#!/usr/bin/env python
"""
ADS1115.py
Function: A simple class to operate an ADS1115 over I2C on an RPi with Python
ADS1115 is 4 channel a 16-bit ADC which operates over I2C
Author: Benjamin Walt
Date: 4/19/2021
Purpose: SoftAgBot system integration project
Version: 0.2
This is a (light) rework of Adafruit's software:
https://github.com/adafruit/Adafruit_Python_ADS1x15
https://www.adafruit.com/product/1085
There are more features which are not implemented here.
"""

import smbus
import time

_DEVICE_ADDRESS = 0x48

# Register and other configuration values:
ADS1115_POINTER_CONVERSION     = 0x00
ADS1115_POINTER_CONFIG         = 0x01
ADS1115_CONFIG_OS_SINGLE       = 0x8000
ADS1115_CONFIG_MUX_OFFSET      = 12

# Maping of gain values to config register values.
ADS1115_CONFIG_GAIN = {
    2/3: 0x0000,
    1:   0x0200,
    2:   0x0400,
    4:   0x0600,
    8:   0x0800,
    16:  0x0A00
}
# #ADS1115_CONFIG_MODE_CONTINUOUS  = 0x0000
ADS1115_CONFIG_MODE_SINGLE      = 0x0100

# Mapping of data/sample rate to config register values for ADS1115 (slower).
ADS1115_CONFIG_DR = {
    8:    0x0000,
    16:   0x0020,
    32:   0x0040,
    64:   0x0060,
    128:  0x0080,
    250:  0x00A0,
    475:  0x00C0,
    860:  0x00E0
}

ADS1115_CONFIG_COMP_QUE_DISABLE = 0x0003


"""
https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/arduino-code
GAIN_TWOTHIRDS (for an input range of +/- 6.144V)
GAIN_ONE (for an input range of +/-4.096V)
GAIN_TWO (for an input range of +/-2.048V)
GAIN_FOUR (for an input range of +/-1.024V)
GAIN_EIGHT (for an input range of +/-0.512V)
GAIN_SIXTEEN (for an input range of +/-0.256V)
"""


class ADS1115:
    """Base functionality for ADS1115 analog to digital converters."""

    def __init__(self):
        self._bus = smbus.SMBus(1) # Channel = 1
        self._address = _DEVICE_ADDRESS
        self._gain = 2/3 # 2/3 is good for values up to +/- 6.144V
        self._data_rate = 128
    
    def __del__(self):
        self._bus.close()

    """
    Write a block of data to the device at the given register
    """
    def _write_block(self, reg, data):
        self._bus.write_i2c_block_data(self._address, reg, data)

	"""
    Read a block of data from the device at the given register
    """
    def _read_block(self, reg, data_size):
        value = self._bus.read_i2c_block_data(self._address, reg, data_size)
        return value
	
	"""
	Read once from the given channel. Returns the signed integer result of the read.
	Reading will be in the range of 0-32,767 (i.e. 15-bit) becuase of sign bit
	"""
    def read_adc(self, channel):
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        # Perform a single shot read and set the mux value to the channel plus
        # the highest bit (bit 3) set.
        return self._read(channel + 0x04, ADS1115_CONFIG_MODE_SINGLE)
	
	"""
	Supposedly, this will be less noisy to read than a single read
	"""
    def read_adc_difference(self, differential):
        """Read the difference between two ADC channels and return the ADC value
        as a signed integer result.  Differential must be one of:
          - 0 = Channel 0 minus channel 1
          - 1 = Channel 0 minus channel 3
          - 2 = Channel 1 minus channel 3
          - 3 = Channel 2 minus channel 3
        """
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        # Perform a single shot read using the provided differential value
        # as the mux value (which will enable differential mode).
        return self._read(differential, ADS1115_CONFIG_MODE_SINGLE)

    """
    Perform an ADC read with the provided mux and mode
    values.  Returns the signed integer result of the read.
    """
    def _read(self, mux, mode):
        config = ADS1115_CONFIG_OS_SINGLE  # Go out of power-down mode for conversion.
        
        # Specify mux value.
        config |= (mux & 0x07) << ADS1115_CONFIG_MUX_OFFSET
        
        # Validate the passed in gain and then set it in the config.
        if self._gain not in ADS1115_CONFIG_GAIN:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        config |= ADS1115_CONFIG_GAIN[self._gain]
        config |= mode # Set the mode (continuous or single shot).
        config |= self._data_rate # Set the data rate
        config |= ADS1115_CONFIG_COMP_QUE_DISABLE  # Disble comparator mode.
        
        # Send the config value to start the ADC conversion.
        # Explicitly break the 16-bit value down to a big endian pair of bytes.
        self._write_block(ADS1115_POINTER_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])
        
        # Wait for the ADC sample to finish based on the sample rate plus a
        # small offset to be sure (0.1 millisecond).
        time.sleep(1.0/self._data_rate + 0.0001)
        
        # Retrieve the result.
        result = self._read_block(ADS1115_POINTER_CONVERSION, 2)
        return self._conversion_value(result[1], result[0])
        
    """
    Sets the rate at which data is produced.
    Must be one of a set of discrete values.
    """        
    def set_data_rate(self, rate):
        if rate not in ADS1115_CONFIG_DR:
            raise ValueError('Data rate must be one of: 8, 16, 32, 64, 128, 250, 475, 860')
        self._data_rate = rate
    """
    Sets the gain for the PGA (programable gain amplifier).
    Must be one of a set of discrete values.
    """
    def set_gain(self, gain):
        if gain not in ADS1115_CONFIG_GAIN:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        self._gain = gain

    """
    Returns the data rate value.
    """
    def get_data_rate(self):
        return self._data_rate

    """
    Returns the gain value.
    """  
    def get_gain(self):
        return self._gain    
    
    """
    Converts the returned 2 bytes into a single, 16 bit value
    """
    def _conversion_value(self, low, high):
        # Convert to 16-bit signed value.
        value = ((high & 0xFF) << 8) | (low & 0xFF)
        # Check for sign bit and turn into a negative value if set.
        if value & 0x8000 != 0:
            value -= 1 << 16
        return value

