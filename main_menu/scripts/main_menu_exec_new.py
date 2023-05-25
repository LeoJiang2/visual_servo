#!/usr/bin/env python

"""
General Imports
"""
import rospy
import RPi.GPIO as GPIO
import sys

"""
Equipment classes
"""
sys.path.append('/home/ubuntu/catkin_ts/src/Stepper')
import StepperMotor as SM

"""
Messages
"""
from std_msgs.msg import Bool
from i2c_comm_pkg.msg import gripper_reply
from i2c_comm_pkg.msg import soft_arm
from i2c_comm_pkg.msg import servo_arm
from i2c_comm_pkg.msg import servo_camera
from spi_comm_pkg.msg import soft_arm_pressure
from spi_comm_pkg.msg import gripper_pressure
from spi_comm_pkg.msg import ir_reply

SPIN_RATE = 20

_COMPRESSOR_RELAY_PIN = rospy.get_param("/compressor_relay")

_STEP_PIN = rospy.get_param("/bed_step")
_DIRECTION_PIN = rospy.get_param("/bed_dir")
_SWITCH_PIN = rospy.get_param("/limit_switch")

def gripper_reply_callback(msg):
	print(msg.color)
	
def soft_arm_pressure_callback(msg):
	print(msg.soft_arm_pressure_psi)

def gripper_pressure_callback(msg):
	print(msg.gripper_pressure_psi)
	
def ir_sensor_callback(msg):
	print(msg.ir_reading)

"""
Prints out the option menu
"""
def menu_print():
	print("0: quit")
	print("1: close gripper")
	print("2: open gripper")
	print("3: move arm")
	print("4: pressurize soft arm")
	print("5: move camera")
	print("6: sensor call (untested)")
	print("7: Extrude Soft Arm")
	print("8: Read arm pressure")
	print("9: Turn on compressor")
	print("10: Turn off compressor")
	print("11: Call arm pressure")
	print("12: Call gripper pressure")
	
	



def main():
	"""
	Initilize
	"""
	step = SM.StepperMotor(_STEP_PIN, _DIRECTION_PIN, _SWITCH_PIN)
	
	"""
	Publishers
	"""
	servo_arm_pub = rospy.Publisher('i2c_comm/servo_arm', servo_arm, queue_size=10)
	servo_camera_pub = rospy.Publisher('i2c_comm/servo_camera', servo_camera, queue_size=10)
	gripper_control_pub = rospy.Publisher('i2c_comm/gripper_control', Bool, queue_size=10)
	gripper_sensor_call_pub = rospy.Publisher('i2c_comm/gripper_sensor_call', Bool, queue_size=10)
	soft_arm_pub = rospy.Publisher('i2c_comm/soft_arm', soft_arm, queue_size=10)
	arm_pressure_call_pub = rospy.Publisher('spi_comm/arm_pressure_call', Bool, queue_size=10)
	gripper_pressure_call_pub = rospy.Publisher('gripper/pressure', Bool, queue_size=10)
	# #shutdown_pub = rospy.Publisher('shutdown_call', Bool, queue_size=10)
	compressor_control_pub = rospy.Publisher('accumulator_enable', Bool,queue_size=10)
	
	"""
	Subscribers
	"""    
	gripper_sensor_reply_sub = rospy.Subscriber('i2c_comm/gripper_sensor_reply', gripper_reply, gripper_reply_callback)
	soft_arm_pressure_sub = rospy.Subscriber('spi_comm/soft_arm_pressure', soft_arm_pressure, soft_arm_pressure_callback)
	gripper_pressure_sub = rospy.Subscriber('spi_comm/gripper_pressure', gripper_pressure, gripper_pressure_callback)
	ir_sensor_sub = rospy.Subscriber('spi_comm/ir_sensor', ir_reply, ir_sensor_callback)
	
	
	rospy.init_node('main_menu_node', anonymous=True)
	rate = rospy.Rate(SPIN_RATE) # 20hz
	
	# Setup to control the GPIO to control the compressor
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(_COMPRESSOR_RELAY_PIN, GPIO.OUT)
	
	while not rospy.is_shutdown():
		menu_print()
		selection = input("Input selection: ")
		selection = int(selection)
		if selection == 0: # Quit
			# #shutdown_pub.publish(True) # This should shut down the other nodes (didn't work...)
			break
		elif selection == 1: # Close gripper
			gripper_control_pub.publish(1)
		elif selection == 2:  # Open gripper
			gripper_control_pub.publish(0)
		elif selection == 3: # Move arm
			t1_input_val = input("Theta 1 Value: ")
			t1_input_val = int(t1_input_val)
			t2_input_val = input("Theta 2 Value: ")
			t2_input_val = int(t2_input_val)
			t3_input_val = input("Theta 3 Value: ")
			t3_input_val = int(t3_input_val)
			t4_input_val = input("Theta 4 Value: ")
			t4_input_val = int(t4_input_val)
			msg = servo_arm()
			msg.servo_arm_deg = [t1_input_val, t2_input_val, t3_input_val, t4_input_val]
			servo_arm_pub.publish(msg)
		elif selection == 4: # pressurize soft arm
			input_bend = input("Bend 0-40.0psi: ")
			input_bend = float(input_bend)
			input_r1 = input("R1 0-40.0psi: ")
			input_r1 = int(input_r1)
			input_r2 = input("R2 0-40.0psi: ")
			input_r2 = int(input_r2)
			msg = soft_arm()
			msg.bend_press = input_bend
			msg.rot_1_press = input_r1
			msg.rot_2_press = input_r2
			soft_arm_pub.publish(msg)
			# #print("Pub")
		elif selection == 5: # Move camera
			print("Pan -> Postive is clockwise")
			print("Tilt -> Negative is up")
			input_pan = input("Pan deg (-95 to 95): ")
			input_pan = int(input_pan)
			input_tilt = input("Tilt deg (-95 to 95):  ")
			input_tilt = int(input_tilt)
			msg = servo_camera()
			msg.camera_pan_deg = input_pan
			msg.camera_tilt_deg = input_tilt
			servo_camera_pub.publish(msg)
		elif selection == 6: # Sensor call
			pass
		elif selection == 7: # Extrusion
			input_val = input("Length (cm): ")
			input_val = int(input_val)
			step.move_stepper(input_val)

		elif selection == 8:
			arm_pressure_call_pub.publish(1)
		elif selection == 9: # Turn on compressor
			compressor_control_pub.publish(True)
			#GPIO.output(_COMPRESSOR_RELAY_PIN, 1) 
		elif selection == 10: # Turn on compressor
			compressor_control_pub.publish(False)
			# #GPIO.output(_COMPRESSOR_RELAY_PIN, 0) 
		elif selection == 11: # Call arm pressure
			arm_pressure_call_pub.publish(1)
		elif selection == 12: # Call gripper pressure
			gripper_pressure_call_pub.publish(1)
		else:
			print("Please pick a selection on the list")
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
