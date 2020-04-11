#!/usr/bin/env python3
import os
import b0RemoteApi
import sys
import signal
import numpy as np
from math import cos, sin, atan2
from os import path
import time

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
class Wall(CoppeliaHandle):
    def __init__(self, coppelia, handle):
        super(Wall, self).__init__(coppelia, handle)

#
# Human
#  
class Human(CoppeliaHandle):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle : int):
        super(Human, self).__init__(coppelia, handle)
        self.dummy_handle = coppelia.get_objects_children(handle, 'sim.object_dummy_type')[0]

    def move(self, x, y, z):
        self.c.set_object_position(self.dummy_handle, x, y, z)

#
# YouBot
#
class YouBot(CoppeliaHandle):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
        super(YouBot, self).__init__(coppelia, handle)
        children = coppelia.get_objects_children(handle, children_type='sim_object_joint_type', filter_children=0)
        for h in children:
            name = self.c.get_object_name(h)
            if 'rollingJoint_fl' in name:
                # print('WHEEL 1:', h, name)
                self.wheel1 = h
            elif 'rollingJoint_rl' in name:
                # print('WHEEL 2:', h, name)
                self.wheel2 = h
            elif 'rollingJoint_rr' in name:
                # print('WHEEL 3:', h, name)
                self.wheel3 = h
            elif 'rollingJoint_fr' in name:
                # print('WHEEL 4:', h, name)
                self.wheel4 = h

    @staticmethod
    def get_position_offsets():
        return 0., 0., 0.095

    @staticmethod
    def get_orientation_offsets():
        return 0., -1.57, 0.

    def set_velocity(self, forwBackVel, leftRightVel, rotVel):
        self.c.set_joint_target_velocity(self.wheel1, -forwBackVel-leftRightVel-rotVel)
        self.c.set_joint_target_velocity(self.wheel2, -forwBackVel+leftRightVel-rotVel)
        self.c.set_joint_target_velocity(self.wheel3, -forwBackVel-leftRightVel+rotVel)
        self.c.set_joint_target_velocity(self.wheel4, -forwBackVel+leftRightVel+rotVel)
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


#
# Pioneer_p3dx
#
class Pioneer_p3dx(CoppeliaHandle):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
        super(Pioneer_p3dx, self).__init__(coppelia, handle)
        children = coppelia.get_objects_children(handle, children_type='sim_object_joint_type', filter_children=0)
        for h in children:
            name = self.c.get_object_name(h)
            if 'Pioneer_p3dx_leftMotor' in name:
                # print('left:', h, name)
                self.left_motor = h
            elif 'Pioneer_p3dx_rightMotor' in name:
                # print('right:', h, name)
                self.right_motor = h

    @staticmethod
    def get_position_offsets():
        return 0., 0., 0.095

    @staticmethod
    def get_orientation_offsets():
        return 0., 0., 0.

    def set_velocity(self, adv, rot):
        # print('@PIONEER', adv, rot)
        axisLength = 0.381
        wheelRadius = 0.0975
        left  = ( adv- ( rot*axisLength ) /2. ) /wheelRadius
        right = ( adv+ ( rot*axisLength ) /2. ) /wheelRadius
        self.c.set_joint_target_velocity(self.left_motor,  left)
        self.c.set_joint_target_velocity(self.right_motor, right)
        # self.c.set_joint_target_velocity(self.left_motor,  forw_back_vel)
        # self.c.set_joint_target_velocity(self.right_motor, forw_back_vel)


# CoppeliaSimAPI
class CoppeliaSimAPI(object):
    def __init__(self, paths=[]):
        super(CoppeliaSimAPI, self).__init__()
        self.coppelia_paths = paths + ['./', os.environ['COPPELIASIM_ROOT']+'/']
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApiAddOn')
        signal.signal(signal.SIGINT, please_exit)

    def load_scene(self, scene_path):
        for source in self.coppelia_paths:
            full_path = source + '/' + scene_path
            if path.exists(full_path):
                return self.client.simxLoadScene(os.path.abspath(full_path), self.client.simxServiceCall())

    def start(self):
        self.client.simxStartSimulation(self.client.simxServiceCall())

    def stop(self):
        self.client.simxStopSimulation(self.client.simxServiceCall())

    def get_objects_children(self, parent='sim.handle_scene', children_type=None, filter_children=1+2):
        if children_type is None:
            children_type = 'sim.handle_all'
        ret = self.client.simxGetObjectsInTree(parent, children_type, filter_children, self.client.simxServiceCall())
        # print('Got children query {}.'.format(ret))
        if ret[0] is True:
            return ret[1]
        raise Exception('Error getting human\'s children {}.'.format(parent))

    def set_object_parent(self, obj, parent, keep_in_place=True):
        obj = self.convert_to_valid_handle(obj)
        parent = self.convert_to_valid_handle(parent)
        code = "sim.setObjectParent({}, {}, {})".format(obj, parent, keep_in_place)
        print(code)
        print(code)
        print(code)
        ret = self.run_script(code)
        print(ret)
        return ret

    def get_object_name(self, handle, alternative_name=''):
        obj = self.convert_to_valid_handle(handle)
        ret = self.client.simxGetObjectName(handle, alternative_name, self.client.simxServiceCall())
        if ret[0] is True:
            return ret[1]
        raise Exception('Can\'t get name for object handle {}.'.format(handle))

    def create_human(self, x, y, z, angle):
        model = 'models/people/path planning Bill.ttm'
        human_handle = self.create_model(model, x, y, z, angle)
        # print('Got human handle {}.'.format(human_handle))
        return Human(self, human_handle)


    def create_wall(self, p1, p2):
        # pre
        model = 'models/infrastructure/walls/80cm high walls/wall section 100cm.ttm'
        x, y = 0.5*(p1[0] + p2[0]), 0.5*(p1[1] + p2[1])
        angle = atan2(p2[1]-p1[1], p2[0]-p1[0])
        # print(angle)
        length = np.linalg.norm(np.array(p2)-np.array(p1))
        # create
        wall_handle = self.create_model(model, x, y, p1[2], angle)
        # resize
        child = self.get_objects_children(wall_handle, 'sim.object_shape_type')[0]
        # print('Got child wall handle {}.'.format(child))
        self.scale_object(child, 6.749*length, 0.12, 1.5)
        return Wall(self, wall_handle)


    def create_model(self, model, x=None, y=None, z=None, rz=None):
        for source in self.coppelia_paths:
            full_path = source + '/' + model
            if path.exists(full_path):
                # print ('File "{}" exists: {}'.format(model, full_path))
                ret = self.client.simxLoadModelFromFile(os.path.abspath(full_path), self.client.simxServiceCall())
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
        obj = self.convert_to_valid_handle(obj)
        M = get_transform_matrix(x, y, z, angle)
        ret = self.client.simxSetObjectMatrix(obj, -1, M, self.client.simxServiceCall())
        return ret

    def convert_to_valid_handle(self, obj):
        if isinstance(obj, CoppeliaHandle):
            obj = obj.handle
        elif type(obj) is str:
            if obj not in ['sim.handle_parent']:
               obj = self.client.simxGetObjectHandle(obj, self.client.simxServiceCall())[1]
        return obj

    def set_object_position(self, obj, x, y, z, reference='sim.handle_parent'):
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        ret = self.client.simxSetObjectPosition(obj, reference, [x, y, z], self.client.simxServiceCall())
        return ret

    def get_object_position(self, obj, reference=-1):
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        ret = self.client.simxGetObjectPosition(obj, reference, self.client.simxServiceCall())
        if ret[0] is not True:
            raise Exception('CoppeliaSimAPI: get_object_position: Can\'t find object {}.'.format(obj))
        return ret[1]


    def set_object_orientation(self, obj, x, y, z, reference='sim.handle_parent'):
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        return self.client.simxSetObjectOrientation(obj, reference, [x, y, z], self.client.simxServiceCall())

    def get_object_orientation(self, obj, reference=-1):
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        ret = self.client.simxGetObjectOrientation(obj, reference, self.client.simxServiceCall())
        if ret[0] is not True:
            raise Exception('CoppeliaSimAPI: get_object_orientation: Can\'t find object {}.'.format(obj))
        return ret[1]

    def remove_object(self, obj):
        if type(obj) is str:
            sobj = self.client.simxGetObjectHandle(obj, self.client.simxServiceCall())[1]
            # print('Converted {} to {}.'.format(obj, sobj))
            obj = sobj
        return self.client.simxRemoveObjects([obj], 1+2, self.client.simxServiceCall())


    def run_script(self, script):
        return self.client.simxExecuteScriptString(script, self.client.simxServiceCall())

    def scale_object(self, handle, sx, sy, sz):
        return self.run_script('sim.scaleObject({},{},{},{},0)'.format(handle, sx, sy, sz))

    def close(self):
        return self.client.simxCloseScene(self.client.simxServiceCall())


    # NOT INCLUDED IN THE DOCUMENTATION YET
    def get_youbot(self) -> YouBot:
        children = self.get_objects_children('sim.handle_scene', children_type='sim.object_shape_type', filter_children=1+2)
        for h in children:
            name = self.get_object_name(h)
            if name == 'youBot':
                return YouBot(self, h)


    def create_youbot(self, x, y, z) -> YouBot:
        ix, iy, iz = YouBot.get_position_offsets()
        ret = self.create_model('models/robots/mobile/KUKA YouBot.ttm', x+ix, y+iy, z+iz, 0.)
        self.set_object_orientation(ret, *YouBot.get_orientation_offsets())
        return YouBot(self, ret)

    def create_pioneer_p3dx(self, x, y, z) -> Pioneer_p3dx:
        ix, iy, iz = Pioneer_p3dx.get_position_offsets()
        ret = self.create_model('models/robots/mobile/pioneer p3dx.ttm', x+ix, y+iy, z+iz, 0.)
        self.set_object_orientation(ret, *Pioneer_p3dx.get_orientation_offsets())
        return Pioneer_p3dx(self, ret)

    def get_pioneer_p3dx(self) -> Pioneer_p3dx:
        children = self.get_objects_children('sim.handle_scene', children_type='sim.object_shape_type', filter_children=1+2)
        for h in children:
            name = self.get_object_name(h)
            if name == 'Pioneer_p3dx':
                return Pioneer_p3dx(self, h)

    def set_joint_target_velocity(self, handle, target):
        return self.client.simxSetJointTargetVelocity(handle, target, self.client.simxServiceCall())

    def pause(self):
        self.client.simxPauseSimulation(self.client.simxServiceCall())
    


# r2d2
# BallRobot
# OmniPlatform




