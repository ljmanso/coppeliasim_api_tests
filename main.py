#!/usr/bin/env python3

import os
import time
import b0RemoteApi
import sys
import random
import signal
import numpy as np
from math import cos, sin, atan2

coppelia_path = '/home/ljmanso/software/coppeliasim/'

exit_flag = False
def please_exit(sig, frame):
	global exit_flag
	print('exiting...')
	exit_flag = True
signal.signal(signal.SIGINT, please_exit)

#
# Wall
#  
class Wall(object):
	def __init__(self, constructor, handle):
		super(Wall, self).__init__()
		self.c = constructor
		self.handle = handle

#
# Human
#  
class Human(object):
	def __init__(self, constructor, handle, dummy_handle):
		super(Human, self).__init__()
		self.c = constructor
		self.handle = handle
		self.dummy_handle = dummy_handle
	def move(self, x, y, angle):
		self.c.set_object_position(self.dummy_handle, [x, y, -1.])


class TheConstructor(object):
	def __init__(self):
		super(TheConstructor, self).__init__()
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		self.client.simxStartSimulation('dddd')
		print('-------------------------------')
		print('Client', self.client)
		print('-------------------------------')

	def create_human(self, x, y, angle):
		model = 'models/people/path planning Bill.ttm'
		human_handle = self.create_model(model, x, y, 0, 0, 0, angle)
		print('Got human handle {}.'.format(human_handle))
		ret = self.client.simxGetObjectsInTree(human_handle, 'sim.object_dummy_type', 1+2, self.client.simxServiceCall())
		dummy_handle = None
		while dummy_handle is None:
			print('Got children query {}.'.format(ret))
			if ret[0] is not True:
				print('Error getting human\'s children {}.'.format(human_handle))
				sys.exit(-1)
			print(ret)
			if len(ret[1]) > 0:
				dummy_handle = ret[1][0]
		return Human(self, human_handle, dummy_handle)

	def create_wall(self, p1, p2):
		def get_transform_matrix(x, y, angle, scalex, scaley, scalez):
			scale_matrix = np.matrix([[1.0, 0.,     0.,   0.],
			                          [0., 100.,     0.,   0.],
								      [0.,         0., 0.1,   0.],
									  [0.,         0.,     0.,   10.]])
			rotat_matrix = np.matrix([[cos(angle), -sin(angle), 0., 0.],
									  [sin(angle),  cos(angle), 0., 0.],
									  [        0.,          0., 1., 0.],
									  [        0.,          0., 0., 1.]])
			trans_matrix = np.matrix([[ 1., 0., 0., x ],
									  [ 0., 1., 0., y ],
									  [ 0., 0., 1., 0.4],
									  [ 0., 0., 0., 1.]])
			# matrix = trans_matrix * scale_matrix * rotat_matrix
			# matrix = rotat_matrix
			matrix = scale_matrix
			# matrix = trans_matrix # WORKS
			return matrix.flatten().tolist()[0]

		model = 'models/infrastructure/walls/80cm high walls/wall section 100cm.ttm'
		x = 0.5*(p1[0] + p2[0])
		y = 0.5*(p1[1] + p2[1])
		angle = atan2(p1[0]-p2[0], p1[0]-p2[0])
		sx = sy = sz = 1.
		wall_handle = self.create_model(model, 0, 0, 0, 0, 0, 0)
		print('Got wall handle {}.'.format(wall_handle))
		ret = self.client.simxGetObjectMatrix(wall_handle, -1, self.client.simxServiceCall())
		print('GET MATRIX {}'.format(ret))
		M = get_transform_matrix(x, y, angle, sx, sy, sz)
		print('M', M)
		ret = self.client.simxSetObjectMatrix(wall_handle, -1, M, self.client.simxServiceCall())
		print('SET MATRIX {}'.format(ret))
		print(ret)
		return Wall(self, wall_handle)

	def create_model(self, model, x, y, z, rx, ry, rz):
		full_path = coppelia_path + '/' + model
		model_exists = os.path.exists(full_path)
		print ('File "{}" exists: {}'.format(full_path, model_exists))
		if not model_exists:
			sys.exit(-1)
		ret = self.client.simxLoadModelFromFile(full_path, self.client.simxServiceCall())
		print('Result of creating the model {}'.format(ret))
		if ret[0] is not True:
			print('Error creating model {}.'.format(full_path))
			sys.exit(-1)
		print('Created model with handle {}.'.format(ret[1]))
		return ret[1]


	def set_object_position(self, obj, lst):
		self.client.simxSetObjectPosition(obj, -1, lst, self.client.simxServiceCall())


	def set_object_orientation(self, obj, lst):
		self.client.simxSetObjectOrientation(obj, -1, lst, self.client.simxServiceCall())



if __name__ == '__main__':
	constructor = TheConstructor()
	# a = constructor.create_human(5.*(random.random()-0.5),
	#                          5.*(random.random()-0.5),
	# 					     2.*3.1415926535*random.random())
	# b = constructor.create_human(5.*(random.random()-0.5),
	#                          5.*(random.random()-0.5),
	# 						 2.*3.1415926535*random.random())

	wall1 = constructor.create_wall([5., 5.], [5.,-5.])
	wall2 = constructor.create_wall([5., -5.], [-5.,-5.])
	wall3 = constructor.create_wall([-5., -5.], [-5.,5.])
	wall4 = constructor.create_wall([-5., 5.], [5.,5.])


	while not exit_flag:
		# a.move(5.*(random.random()-0.5), 5.*(random.random()-0.5), None)
		# b.move(5.*(random.random()-0.5), 5.*(random.random()-0.5), None)

		for i in range(20):
			time.sleep(0.5)
			# constructor.client.simxSpinOnce()
			if exit_flag:
				break







