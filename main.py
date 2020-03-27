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


def get_transform_matrix(x, y, z, angle, scalex=1., scaley=1., scalez=1.):
	scale_matrix = np.matrix([[1.*scalex,    0.,        0.,   0.],
						      [0.,    1.*scaley,        0.,   0.],
							  [0.,           0., 1.*scalez,   0.],
							  [0.,           0.,        0.,   1.]])
	rotate_matrix = np.matrix([[cos(angle), -sin(angle), 0., 0.],
							   [sin(angle),  cos(angle), 0., 0.],
							   [        0.,          0., 1., 0.],
							   [        0.,          0., 0., 1.]])
	translate_matrix = np.matrix([[ 1., 0., 0., x ],
			 				      [ 0., 1., 0., y ],
			                      [ 0., 0., 1., z],
							      [ 0., 0., 0., 1.]])
	matrix = translate_matrix * rotate_matrix * scale_matrix
	return matrix.flatten().tolist()[0]

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

# TheConstructor
class TheConstructor(object):
	def __init__(self):
		super(TheConstructor, self).__init__()
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		print('-------------------------------')
		print('Client', self.client)
		print('-------------------------------')

	def get_objects_children(self, parent, children_type):
		ret = self.client.simxGetObjectsInTree(parent, children_type, 1+2, self.client.simxServiceCall())
		print('Got children query {}.'.format(ret))
		if ret[0] is not True:
			print('Error getting human\'s children {}.'.format(parent))
			sys.exit(-1)
		return ret[1]

	def create_human(self, x, y, angle):
		model = 'models/people/path planning Bill.ttm'
		human_handle = self.create_model(model, x, y, 0, 0, 0, angle)
		print('Got human handle {}.'.format(human_handle))
		dummy_handle = self.get_objects_children(human_handle, 'sim.object_dummy_type')[0]
		return Human(self, human_handle, dummy_handle)


	def create_wall(self, p1, p2):
		model = 'models/infrastructure/walls/80cm high walls/wall section 100cm.ttm'
		x, y = 0.5*(p1[0] + p2[0]), 0.5*(p1[1] + p2[1])
		angle = atan2(p2[1]-p1[1], p2[0]-p1[0])
		length = np.linalg.norm(np.array(p2)-np.array(p1))
		wall_handle = self.create_model(model, 0, 0, 0, 0, 0, 0)
		print('Got wall handle {}.'.format(wall_handle))
		M = get_transform_matrix(x, y, 0.4, angle, 1., 1., 1.)
		print('M', M)
		ret = self.client.simxSetObjectMatrix(wall_handle, -1, M, self.client.simxServiceCall())
		print('SET MATRIX {}'.format(ret))

		# self.client.simxExecuteScriptString('sim.scaleObject({},{},{},{})'.format(wall_handle, 10.*length,0.1,1.),
										# self.client.simxServiceCall())

		child = self.get_objects_children(wall_handle, 'sim.object_shape_type')[0]
		print('Got child wall handle {}.'.format(child))
		ret = self.client.simxExecuteScriptString('sim.scaleObject({},{},{},{},0)'.format(child, 10.*length,0.1,1.),
										self.client.simxServiceCall())
		# M = get_transform_matrix(0., 0., 0., 0., 1000., 1., 1.)
		# print('M', M)
		# ret = self.client.simxSetObjectMatrix(child, wall_handle, M, self.client.simxServiceCall())
		# print('SET MATRIX {}'.format(ret))
		# print(ret)
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
	a = constructor.create_human(15.*(random.random()-0.5),
	                         15.*(random.random()-0.5),
						     2.*3.1415926535*random.random())
	b = constructor.create_human(15.*(random.random()-0.5),
	                         15.*(random.random()-0.5),
							 2.*3.1415926535*random.random())

	constructor.create_wall([5., 5.], [5.,-5.])
	constructor.create_wall([5., -5.], [-5.,-5.])
	constructor.create_wall([-5., -5.], [-5.,5.])
	constructor.create_wall([-5., 5.], [5.,5.])

	constructor.client.simxStartSimulation(constructor.client.simxServiceCall())

	while not exit_flag:
		a.move(15.*(random.random()-0.5), 15.*(random.random()-0.5), None)
		b.move(15.*(random.random()-0.5), 15.*(random.random()-0.5), None)

		for i in range(20):
			time.sleep(0.5)
			constructor.client.simxSpinOnce()
			if exit_flag:
				break







