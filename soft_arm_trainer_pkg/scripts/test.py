#!/usr/bin/env python

import rospy
import json
import os

class Test:

	def __init__(self):
		self._data = [1,2,3,4,5]
		os.chdir("/home/ubuntu/catkin_ts/src/soft_arm_trainer_pkg/data")
		with open("a.txt", "w") as filehandle:
			json.dump(self._data, filehandle)
		print("File Saved")
			# #filehandle.write("Hello \n") 
		# #filehandle.close()
		

	# #def _setup(self):
		# #self._position_arm(self._initial_arm_config)
		# #angle = raw_input("Link 4 angle (Horizontal is 0)")
		# #angle_corrected = angle
		# #self._position_arm([self._cur_arm_config[0], self._cur_arm_config[1], angle_corrected, self._cur_arm_config[3]])
		# ## Turn on compressor
		# #GPIO.output(_COMPRESSOR_RELAY_PIN, 1)
		# ## Start training
		# #self._train()
		# ## Turn off compressor
		# #GPIO.output(_COMPRESSOR_RELAY_PIN, 0)
		# #self._step.move_stepper(0)
		# #msg = soft_arm()
		# #msg.bend_press = 0
		# #msg.rot_1_press = 0
		# #msg.rot_2_press = 0
		# #self._soft_arm_pub.publish(msg)
		# #exit()
		
	# #def _position_arm(self, config):
		# #self._cur_arm_config = config
		# #msg = servo_arm()
		# #msg.servo_arm_deg = config
		# ##servo_arm_pub.publish(msg)




	# #def _training_loop_trajectory(self, plan_num):
		# #filename = "soft_arm_data_" + str(self._arm_angle) + "_" + str(plan_num) + ".txt"
		# #data = []
		# #data.append([self._arm_angle, plan_num])


		# #for config in range(_TRAJECTORY_SIZE):
			# #print("config " + str(config)) 
			# #length = random.choice([3, 6, 9, 12, 15, 18])
			# #bending = random.uniform(0, 40)
			# #r1 = random.uniform(0, 40)
			# #r2 = random.uniform(0, 40)
			# #theta_4 = random.uniform(-90, 90)
			# ## extrude arm
			# #print("extrude")
			# #self._step.move_stepper(length)
			# ## wait?
			# ## publish pressures
			# #print("soft arm")
			# #msg = soft_arm()
			# #msg.bend_press = bending
			# #msg.rot_1_press = r1
			# #msg.rot_2_press = r2
			# #self._soft_arm_pub.publish(msg)
			# ## Move theta_4
			# #print("theta 4")
			# #arm_config = [self._cur_arm_config[0], self._cur_arm_config[1], self._cur_arm_config[2], theta_4]
			# #self._position_arm(arm_config)
			# ## wait 
			# #rospy.sleep(_WAIT)

			# ## collect data from polhemus
			# #print("collect data from polhemus")
			# #rigid_arm_end = self._patriot.get_pose(_RIGID_ARM_TIP_STATION)
			# #soft_arm_end = self._patriot.get_pose(_SOFT_ARM_TIP_STATION)
			# ## data = config, length, bending, r1, r2, arm_config, rigid_arm, soft_arm

			# #data.append([config, length, bending, r1, r2, arm_config, rigid_arm_end, soft_arm_end])

		# #print("Saving data")
		# #fout = open('hello.txt', 'w')
		# #fout.write('Hello, world!\n') 
		# #fout.close()
				
		# #with open("a.txt", "w") as filehandle:
			# #json.dump(data, filehandle)
			# #filehandle.write("Hello \n") 
		# #filehandle.close()
		
	# #def _train(self):
		# #for plan in range(_NUMB_TRAJECTORY):
			# #print("Plan " + str(plan)) 
			# #self._training_loop_trajectory(plan)

if __name__ == '__main__':
	rospy.init_node('soft_arm_trainer_node', anonymous=True)
	Test()
	# #rospy.spin()
