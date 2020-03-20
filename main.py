#!/usr/bin/env python3

import time
import b0RemoteApi
import sys
import random


import signal


exit_flag = False

print("Initialize")

client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
print('-------------------------------')
print('Client', client)

def please_exit(sig, frame):
	global exit_flag
	print('exiting...')
	exit_flag = True
signal.signal(signal.SIGINT, please_exit)

# goal = client.simxGetObjectHandle('Bill_goalDummy', client.simxServiceCall())
# if not goal[0]:
# 	print('-------------------------------')
# 	print('Can\'t get goal.')
# 	sys.exit(1)
# goal = goal[1]


# client.simxStartSimulation('dddd')
# simxAddDrawingObject_cubes
r = client.simxAddDrawingObject_cubes(3.1,[255,0,127], [1.,2.,2.], client.simxServiceCall())
print(r)
print('-------------------------------')


r = client.simxLoadModelFromFile('coppelia/models/people/path\ planning\ Bill.ttm', client.simxServiceCall())

print(r)
# print('-------------------------------')
# print('Goal', goal)

# while not exit_flag:
# 	x = 5.*(random.random()-0.5)
# 	z = 5.*(random.random()-0.5)
# 	client.simxSetObjectPosition(goal, -1, [x,z,-1.], client.simxServiceCall())

# 	for i in range(20):
# 		time.sleep(0.5)
# 		# client.simxSpinOnce()
# 		if exit_flag:
# 			break



