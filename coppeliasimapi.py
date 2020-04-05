#!/usr/bin/env python3
import os
import b0RemoteApi
import sys
import signal
import numpy as np
from math import cos, sin, atan2
from os import path

def please_exit(sig, frame):
    sys.exit(0)

def get_transform_matrix(x, y, z, angle):
	rotate_matrix = np.matrix([[cos(angle), -sin(angle), 0., 0.],
							   [sin(angle),  cos(angle), 0., 0.],
							   [        0.,          0., 1., 0.],
							   [        0.,          0., 0., 1.]])
	translate_matrix = np.matrix([[ 1., 0., 0., x ],
			 				      [ 0., 1., 0., y ],
			                      [ 0., 0., 1., z ],
							      [ 0., 0., 0., 1.]])
	
	return (translate_matrix * rotate_matrix).flatten().tolist()[0]

    # print(wheelJoints[1], -forwBackVel-leftRightVel-rotVel)
    # print(wheelJoints[2], -forwBackVel+leftRightVel-rotVel)
    # print(wheelJoints[3], -forwBackVel-leftRightVel+rotVel)
    # print(wheelJoints[4], -forwBackVel+leftRightVel+rotVel)
#
# Generic Handle
#
class CoppeliaHandle(object):
	def __init__(self, coppelia, handle):
		super(CoppeliaHandle, self).__init__()
		self.c = coppelia
		self.handle = handle

#
# Wall
#  
class Wall(object):
	def __init__(self, coppelia, handle):
		super(Wall, self).__init__()
		self.c = coppelia
		self.handle = handle

#
# Human
#  
class Human(object):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle : int):
        super(Human, self).__init__()
        self.c = coppelia
        self.handle = handle
        self.dummy_handle = coppelia.get_objects_children(handle, 'sim.object_dummy_type')[0]

    def move(self, x, y, z):
        self.c.set_object_position(self.dummy_handle, x, y, z)

#
# YouBot
#
class YouBot(object):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
        super(YouBot, self).__init__()
        self.c = coppelia
        self.handle = handle
        # children = coppelia.get_objects_children(handle, children_type='sim.handle_all', filter_children=0)
        children = coppelia.get_objects_children(handle, children_type='sim_object_joint_type', filter_children=0)
        print (children)
        for h in children:
            name = self.c.get_object_name(h)
            if 'rollingJoint_fl' in name:
                print('WHEEL 1:', h, name)
                self.wheel1 = h
            elif 'rollingJoint_rl' in name:
                print('WHEEL 2:', h, name)
                self.wheel2 = h
            elif 'rollingJoint_rr' in name:
                print('WHEEL 3:', h, name)
                self.wheel3 = h
            elif 'rollingJoint_fr' in name:
                print('WHEEL 4:', h, name)
                self.wheel4 = h

    @staticmethod
    def get_position_offsets():
        return 0., 0., 0.095

    @staticmethod
    def get_orientation_offsets():
        return 0., -1.57, 0.

    def set_velocity(self, forwBackVel, leftRightVel, rotVel):

        print('w1', self.c.set_joint_target_velocity(self.wheel1, -2))
        print('w2', self.c.set_joint_target_velocity(self.wheel2, -2))
        print('w3', self.c.set_joint_target_velocity(self.wheel3, -2))
        print('w4', self.c.set_joint_target_velocity(self.wheel4, -2))

        # print('w1', self.c.set_joint_target_velocity(self.wheel1, -forwBackVel-leftRightVel-rotVel))
        # print('w2', self.c.set_joint_target_velocity(self.wheel2, -forwBackVel+leftRightVel-rotVel))
        # print('w3', self.c.set_joint_target_velocity(self.wheel3, -forwBackVel-leftRightVel+rotVel))
        # print('w4', self.c.set_joint_target_velocity(self.wheel4, -forwBackVel+leftRightVel+rotVel))

        # print('VELS:', forwBackVel, leftRightVel, rotVel)
        # code = """
        # sim.setJointTargetVelocity({}, {}) and
        # sim.setJointTargetVelocity({}, {}) and
        # sim.setJointTargetVelocity({}, {}) and
        # sim.setJointTargetVelocity({}, {})
        # """.format(
        #     self.wheel1, -forwBackVel-leftRightVel-rotVel,
        #     self.wheel2, -forwBackVel+leftRightVel-rotVel,
        #     self.wheel3, -forwBackVel-leftRightVel+rotVel,
        #     self.wheel4, -forwBackVel+leftRightVel+rotVel)
        # print('CODE:', code)
        # print(self.c.run_script(code))

# CoppeliaSimAPI
class CoppeliaSimAPI(object):
    def __init__(self, paths=[]):
        super(CoppeliaSimAPI, self).__init__()
        self.coppelia_paths = paths + [os.environ['COPPELIASIM_ROOT']+'/']
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApiAddOn')
        signal.signal(signal.SIGINT, please_exit)


    def start(self):
        self.client.simxStartSimulation(self.client.simxServiceCall())

    def stop(self):
        self.client.simxStopSimulation(self.client.simxServiceCall())

    def get_objects_children(self, parent, children_type=None, filter_children=1+2):
        if children_type is None:
            children_type = 'sim.handle_all'
        ret = self.client.simxGetObjectsInTree(parent, children_type, filter_children, self.client.simxServiceCall())
        print('Got children query {}.'.format(ret))
        if ret[0] is True:
            return ret[1]
        raise Exception('Error getting human\'s children {}.'.format(parent))

    def get_object_name(self, object_handle, alternative_name=''):
        ret = self.client.simxGetObjectName(object_handle, alternative_name, self.client.simxServiceCall())
        if ret[0] is True:
            return ret[1]
        raise Exception('Can\'t get name for object handle {}.'.format(object_handle))

    def create_human(self, x, y, z, angle):
            model = 'models/people/path planning Bill.ttm'
            human_handle = self.create_model(model, x, y, z, angle)
            print('Got human handle {}.'.format(human_handle))
            return Human(self, human_handle)


    def create_wall(self, p1, p2):
        # pre
        model = 'models/infrastructure/walls/80cm high walls/wall section 100cm.ttm'
        x, y = 0.5*(p1[0] + p2[0]), 0.5*(p1[1] + p2[1])
        angle = atan2(p2[1]-p1[1], p2[0]-p1[0])
        print(angle)
        length = np.linalg.norm(np.array(p2)-np.array(p1))
        # create
        wall_handle = self.create_model(model, x, y, p1[2], angle)
        # resize
        child = self.get_objects_children(wall_handle, 'sim.object_shape_type')[0]
        print('Got child wall handle {}.'.format(child))
        self.scale_object(child, 6.749*length, 0.12, 1.5)
        return Wall(self, wall_handle)


    def create_model(self, model, x=None, y=None, z=None, rz=None):
        for source in self.coppelia_paths:
            full_path = source + '/' + model
            if path.exists(full_path):
                # print ('File "{}" exists: {}'.format(model, full_path))
                ret = self.client.simxLoadModelFromFile(full_path, self.client.simxServiceCall())
                # print('Result of creating the model {}'.format(ret))
                if ret[0] is not True:
                    raise Exception('Error creating model {}.'.format(full_path))
                # print('Created model with handle {}.'.format(ret[1]))
                if x is not None and y is not None and z is not None:
                    self.set_object_transform(ret[1], x, y, z, rz)
                return ret[1]
        raise Exception('Couldn\'t find model {} in any of the paths {}'.format(model, self.coppelia_paths))


    def get_object_handle(self, object_name):
        obj = self.client.simxGetObjectHandle(object_name, self.client.simxServiceCall())
        if obj[0] is True:
            return obj[1]
        raise Exception('Can\'t get object handle for object {}'.format(object_name))

    def set_object_transform(self, obj, x, y, z, angle):
        if type(obj) is str:
            obj = self.client.simxGetObjectHandle(obj, self.client.simxServiceCall())[1]
        M = get_transform_matrix(x, y, z, angle)
        ret = self.client.simxSetObjectMatrix(obj, -1, M, self.client.simxServiceCall())
        return ret

    def set_object_position(self, obj, x, y, z):
        if type(obj) is str:
            obj = self.client.simxGetObjectHandle(obj, self.client.simxServiceCall())[1]
        ret = self.client.simxSetObjectPosition(obj, -1, [x, y, z], self.client.simxServiceCall())
        return ret


    def set_object_orientation(self, obj, x, y, z):
        return self.client.simxSetObjectOrientation(obj, -1, [x, y, z], self.client.simxServiceCall())

    def remove_object(self, obj):
        if type(obj) is str:
            sobj = self.client.simxGetObjectHandle(obj, self.client.simxServiceCall())[1]
            print('Converted {} to {}.'.format(obj, sobj))
            obj = sobj
        return self.client.simxRemoveObjects([obj], 1+2, self.client.simxServiceCall())


    def run_script(self, script):
        return self.client.simxExecuteScriptString(script, self.client.simxServiceCall())

    def scale_object(self, handle, sx, sy, sz):
        return self.run_script('sim.scaleObject({},{},{},{},0)'.format(handle, sx, sy, sz))

    def close(self):
        return self.client.simxCloseScene(self.client.simxServiceCall())


    # NOT INCLUDED IN THE DOCUMENTATION YET

    def create_youbot(self, x, y, z) -> YouBot:
        ix, iy, iz = YouBot.get_position_offsets()
        ret = self.create_model('models/robots/mobile/KUKA YouBot.ttm', x+ix, y+iy, z+iz, 0.)
        self.set_object_orientation(ret, *YouBot.get_orientation_offsets())
        return YouBot(self, ret)

    def set_joint_target_velocity(self, handle, target):
        return self.client.simxSetJointTargetVelocity(handle, target, self.client.simxServiceCall())

    def pause(self):
        self.client.simxPauseSimulation(self.client.simxServiceCall())
    


# r2d2
# BallRobot
# Pioneer_p3dx
# youBot
# OmniPlatform




