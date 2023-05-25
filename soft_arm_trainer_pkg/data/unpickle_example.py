#!/usr/bin/env python

import pickle


class ArmData():
    def __init__(self, config, length, bending, r1, r2, arm_config, rigid_arm_end, soft_arm_end,
                    dbending, drotation, dtheta_4, dlength):
		self.config = config
		self.length = length
		self.bending = bending
		self.r1 = r1
		self.r2 = r2
		self.arm_config = arm_config
		self.rigid_arm_end = rigid_arm_end
		self.soft_arm_end = soft_arm_end
		self.dbending = dbending
		self.drotation = drotation
		self.dtheta_4 = dtheta_4
		self.dlength = dlength

class PatriotData():
    def __init__(self, x, y, z, qx, qy, qz, qw):
		self.x = x
		self.y = y
		self.z = z
		self.qx = qx
		self.qy = qy
		self.qz = qz
		self.qw = qw


arm_angle = 0.0
plan_num = 0
filename = "soft_arm_data_" + str(arm_angle) + "_" + str(plan_num) + ".pkl"

with open(filename, "rb") as filehandle:
	data = pickle.load(filehandle)

print(data[0])
print(data[1].config)
print(data[1].rigid_arm_end.x)
