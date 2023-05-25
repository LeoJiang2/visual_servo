#!/usr/bin/env python
"""
TCS34725.py
Function: A simple class to operate a TCS34725 color sensor on an RPi with Python
Author: Benjamin Walt
Date: 9/29/2020
Purpose: SoftAgBot system integration project
Version: 0.1
This is a rework of Adafruit's software:
https://github.com/adafruit/Adafruit_TCS34725
https://github.com/adafruit/Adafruit_CircuitPython_TCS34725
https://www.adafruit.com/product/1334
"""
import smbus
from time import sleep


"""Some of these are repeated"""
_COMMAND_BIT = 0x80
_REGISTER_ENABLE = 0x00
_REGISTER_ATIME = 0x01
_REGISTER_AILT = 0x04
_REGISTER_AIHT = 0x06
# #_REGISTER_ID = 0x12
_REGISTER_SENSORID = 0x12 #used
_REGISTER_APERS = 0x0C
_REGISTER_CONTROL = 0x0F

_REGISTER_STATUS = 0x13
_REGISTER_CDATA = 0x14
_REGISTER_RDATA = 0x16
_REGISTER_GDATA = 0x18
_REGISTER_BDATA = 0x1A
_ENABLE_AIEN = 0x10
_ENABLE_WEN = 0x08
_ENABLE_AEN = 0x02
_ENABLE_PON = 0x01

_CYCLES = (0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60)
_INTEGRATION_TIME_THRESHOLD_LOW = 2.4
_INTEGRATION_TIME_THRESHOLD_HIGH = 614.4

_TCS34725_INTEGRATIONTIME_2_4MS =  0xFF ##<  2.4ms - 1 cycle    - Max Count: 1024  */
_TCS34725_INTEGRATIONTIME_24MS =  0xF6 ##<  24ms  - 10 cycles  - Max Count: 10240 */
_TCS34725_INTEGRATIONTIME_50MS =   0xEB ##<  50ms  - 20 cycles  - Max Count: 20480 */
_TCS34725_INTEGRATIONTIME_101MS =  0xD5 ##<  101ms - 42 cycles  - Max Count: 43008 */
_TCS34725_INTEGRATIONTIME_154MS =   0xC0 ##<  154ms - 64 cycles  - Max Count: 65535 */
_TCS34725_INTEGRATIONTIME_700MS =   0x00 ##<  700ms - 256 cycles - Max Count: 65535 */

_TCS34725_GAIN_1X = 0x00 ##<  No gain  */
_TCS34725_GAIN_4X = 0x01 ##,  /**<  4x gain  */
_TCS34725_GAIN_16X = 0x02  ##, /**<  16x gain */
_TCS34725_GAIN_60X = 0x03  ##/**<  60x gain */

# Needs to be 700ms and gain x1

class TCS34725:
    """Class to control the TCS34725 color sensor"""    
    def __init__(self):
        self._bus = smbus.SMBus(1) #Channel = 1
        self._address = 0x29
        # Check sensor ID is expectd value.
        sensor_id = self._read_reg(_REGISTER_SENSORID)
        if sensor_id != 0x44:
            print("Could not find sensor, check wiring!") 


    def _read_reg(self, reg):
        """
        This performs all the reads.  It first writes the desired register to read from.
        It then reads from that register.
        """
        self._bus.write_byte(self._address, reg | _COMMAND_BIT)
        results = self._bus.read_byte(self._address)
        return results
    
    def _write_reg(self, reg,value):
        """
        This performs all the writes.  It first writes the desired register to write to.
        It then writes a value to that register.
        """
        self._bus.write_byte(self._address, reg | _COMMAND_BIT)
        self._bus.write_byte(self._address, value)

    def _valid(self):
        """Check if device is ready to read"""
        return self._read_reg(_REGISTER_STATUS) & 0x01

    def setup_color_sensor(self):
        """Does a number of set up steps"""
        # Arduino code used for training had:
        # 700 ms integration time
        # gain x1
        # Set integration time
        self._write_reg(_REGISTER_ATIME, _TCS34725_INTEGRATIONTIME_700MS) # Set based on training source
        # Set Gain
        self._write_reg(_REGISTER_CONTROL, _TCS34725_GAIN_1X) # Set based on training source
        # Enable
        self._write_reg(_REGISTER_ENABLE, _ENABLE_PON)
        sleep(0.5)
        self._write_reg(_REGISTER_ENABLE, _ENABLE_PON | _ENABLE_AEN)
        sleep(1) #actual sleep time varies with integration time, but doesn't seem critical in our use
    
    def read_color_sensor(self):
        """Reads the RGB values and retuns them as 0-255 value"""
        
        while not self._valid():
            time.sleep(50.0/1000.0) #How to set this??
        
        red_raw = self._read_reg(_REGISTER_RDATA)
        green_raw = self._read_reg(_REGISTER_BDATA)
        blue_raw = self._read_reg(_REGISTER_BDATA)
        clear_raw = self._read_reg(_REGISTER_CDATA)
        
        if clear_raw == 0:
            return [0, 0, 0] # Returns black
        
        red = int(pow((int((red_raw / clear_raw) * 256) / 255), 2.5) * 255)
        green = int(pow((int((green_raw / clear_raw) * 256) / 255), 2.5) * 255)
        blue = int(pow((int((blue_raw / clear_raw) * 256) / 255), 2.5) * 255)
        # Handle possible 8-bit overflow
        if red > 255:
            red = 255
        if green > 255:
            green = 255
        if blue > 255:
            blue = 255
        return [red, green, blue]

