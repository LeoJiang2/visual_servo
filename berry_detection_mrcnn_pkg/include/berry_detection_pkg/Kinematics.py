import numpy as np

class Kinematics:
	def __init__(self):
		self.cur_pan_value_deg = 0.0
		self.cur_tilt_value_deg = 0.0


	def get_ee_in_world(self, theta1, theta2, theta3, theta4):
		x_ee_w = 0.2030*np.cos(theta1)*np.sin(theta2) - 0.0213*np.cos(theta4)*np.sin(theta1) + 0.0032*np.sin(theta1)*np.sin(theta4) - 0.0032*np.cos(theta4)*(np.cos(theta1)*np.sin(theta2)*np.sin(theta3) + np.cos(theta1)*np.cos(theta2)*np.cos(theta3)) + np.sin(theta1)*(0.0213*np.cos(theta4) - 0.0213) + np.cos(theta1)*np.sin(theta2)*(0.4700*np.cos(theta3) - 0.4700) + 0.3736*np.cos(theta1)*np.cos(theta2)*np.sin(theta3) - 0.8437*np.cos(theta1)*np.cos(theta3)*np.sin(theta2)
		y_ee_w = 0.0213*np.cos(theta1)*np.cos(theta4) - 0.0032*np.cos(theta1)*np.sin(theta4) + 0.2030*np.sin(theta1)*np.sin(theta2) - np.cos(theta1)*(0.0213*np.cos(theta4) - 0.0213) - 0.0032*np.cos(theta4)*(np.sin(theta1)*np.sin(theta2)*np.sin(theta3) + np.cos(theta2)*np.cos(theta3)*np.sin(theta1)) + 0.3736*np.cos(theta2)*np.sin(theta1)*np.sin(theta3) - 0.8437*np.cos(theta3)*np.sin(theta1)*np.sin(theta2) + np.sin(theta1)*np.sin(theta2)*(0.4700*np.cos(theta3) - 0.4700)
		z_ee_w = 0.8437*np.cos(theta2)*np.cos(theta3) - 0.2030*np.cos(theta2) + 0.3736*np.sin(theta2)*np.sin(theta3) - np.cos(theta2)*(0.4700*np.cos(theta3) - 0.4700) + 0.0032*np.cos(theta4)*(np.cos(theta2)*np.sin(theta3) - np.cos(theta3)*np.sin(theta2)) + 0.2030

		return [x_ee_w, y_ee_w, z_ee_w]

	def get_camera2world(self, x_c, y_c, z_c):
		theta_pan = np.deg2rad(self.cur_pan_value_deg)
		theta_tilt = np.deg2rad(self.cur_tilt_value_deg)
		print(theta_pan)
		X = np.array([x_c, y_c, z_c, 1])
		T = [[ -np.sin(theta_pan), -np.cos(theta_pan)*np.sin(theta_tilt),  np.cos(theta_pan)*np.cos(theta_tilt), 0.0070*np.sin(theta_pan) - 0.0750*np.cos(theta_pan) + 0.1381*np.cos(theta_pan)*np.cos(theta_tilt) + 0.1330*np.cos(theta_pan)*np.sin(theta_tilt) - np.cos(theta_pan)*(0.0550*np.cos(theta_tilt) + 0.1330*np.sin(theta_tilt) - 0.0550) + 0.0750], \
			 [ -np.cos(theta_pan),  np.sin(theta_pan)*np.sin(theta_tilt), -np.cos(theta_tilt)*np.sin(theta_pan), 0.0070*np.cos(theta_pan) + 0.0750*np.sin(theta_pan) - 0.1381*np.cos(theta_tilt)*np.sin(theta_pan) - 0.1330*np.sin(theta_pan)*np.sin(theta_tilt) + np.sin(theta_pan)*(0.0550*np.cos(theta_tilt) + 0.1330*np.sin(theta_tilt) - 0.0550) + 0.1330], \
		 	 [               0,                -np.cos(theta_tilt),                -np.sin(theta_tilt),                                                                                                                                                                                    0.1330 - 0.0831*np.sin(theta_tilt)], \
             [               0,                               0,                               0,                                                                                                                                                                                                                  1]]
               
		
		X_w = np.matmul(T,X)
		print(X_w)
		return [X_w[0], X_w[1], X_w[2]]


	# calibration version 
	def _camera2arm_base(self, point_camera, pan_d, tilt_d):
		point_camera_h = np.array([point_camera[0], point_camera[1], point_camera[2], 1])
		# #print(point_camera_h)
		theta_pan = np.deg2rad(pan_d)
		theta_tilt = np.deg2rad(tilt_d)
		# transform = np.array([[ -np.sin(theta_pan),  np.cos(theta_pan)*np.sin(theta_tilt),  np.cos(theta_pan)*np.cos(theta_tilt), 0.0530*np.sin(theta_pan) - 0.0360*np.cos(theta_pan) + 0.1343*np.cos(theta_pan)*np.cos(theta_tilt) - 0.0670*np.cos(theta_pan)*np.sin(theta_tilt) + np.cos(theta_pan)*(0.0670*np.sin(theta_tilt) - 0.0560*np.cos(theta_tilt) + 0.0560) + 0.0360], 
		# 					[ -np.cos(theta_pan), -np.sin(theta_pan)*np.sin(theta_tilt), -np.cos(theta_tilt)*np.sin(theta_pan), 0.0530*np.cos(theta_pan) + 0.0360*np.sin(theta_pan) - 0.1343*np.cos(theta_tilt)*np.sin(theta_pan) + 0.0670*np.sin(theta_pan)*np.sin(theta_tilt) - np.sin(theta_pan)*(0.0670*np.sin(theta_tilt) - 0.0560*np.cos(theta_tilt) + 0.0560) + 0.1435], 
		# 					[0, -np.cos(theta_tilt), np.sin(theta_tilt), 0.0784*np.sin(theta_tilt) + 0.0670], 
		# 					[0, 0, 0, 1]])
		# transform = np.array([[1.0*np.sin(theta_tilt)*np.cos(theta_pan), -1.0*np.cos(theta_pan)*np.cos(theta_tilt), -1.0*np.sin(theta_pan), 0.0356*np.sin(theta_pan) + 0.07405*np.cos(theta_pan)*np.cos(theta_tilt) - 0.00995*np.cos(theta_pan) - 0.00995], 
 	# 		[-1.0*np.sin(theta_pan)*np.sin(theta_tilt), 1.0*np.sin(theta_pan)*np.cos(theta_tilt), -1.0*np.cos(theta_pan), -0.07405*np.sin(theta_pan)*np.cos(theta_tilt) + 0.00995*np.sin(theta_pan) + 0.0356*np.cos(theta_pan)], 
  # 			[1.0*np.cos(theta_tilt), 1.0*np.sin(theta_tilt), 0, 0.0619 - 0.07405*np.sin(theta_tilt)], [0., 0., 0., 1.]])

		transform = np.array([[-1.0*np.sin(theta_pan), -1.0*np.sin(theta_tilt)*np.cos(theta_pan), 1.0*np.cos(theta_pan)*np.cos(theta_tilt), 0.0356*np.sin(theta_pan) + 0.07405*np.cos(theta_pan)*np.cos(theta_tilt) - 0.00995*np.cos(theta_pan) - 0.00995], [-1.0*np.cos(theta_pan), 1.0*np.sin(theta_pan)*np.sin(theta_tilt), -1.0*np.sin(theta_pan)*np.cos(theta_tilt), -0.07405*np.sin(theta_pan)*np.cos(theta_tilt) + 0.00995*np.sin(theta_pan) + 0.0356*np.cos(theta_pan)], [0, -1.0*np.cos(theta_tilt), -1.0*np.sin(theta_tilt), 0.0619 - 0.07405*np.sin(theta_tilt)], [0, 0, 0, 1.00000000000000]])


		point_base_h = np.matmul(transform, point_camera_h)
		# print(point_base_h)
		return [point_base_h[0] + 0.052 , point_base_h[1] + 0.152, point_base_h[2]]
