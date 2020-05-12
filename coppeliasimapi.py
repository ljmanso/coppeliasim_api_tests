#!/usr/bin/env python3
import os
import b0RemoteApi
import sys
import threading
import numpy as np
from math import cos, sin, atan2
from os import path
import time


class CallType(object):
    def __init__(self, call):
        self.call = call

    def get(self):
        return self.call


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
    def __init__(self, coppelia, p1, p2):
        super(Wall, self).__init__(coppelia, -1)
        self.p1, self.p2 = p1, p2
        # pre
        model = 'models/infrastructure/walls/80cm high walls/wall section 100cm.ttm'
        x, y = 0.5 * (p1[0] + p2[0]), 0.5 * (p1[1] + p2[1])
        angle = atan2(p2[1] - p1[1], p2[0] - p1[0])
        length = np.linalg.norm(np.array(p2) - np.array(p1))
        # create
        wall_handle = coppelia.create_model(model, x, y, p1[2], angle, asynch=False)
        # resize
        child = coppelia.get_objects_children(wall_handle, 'sim.object_shape_type', asynch=False)[0]
        # print (f'WALL child {child}')
        coppelia.scale_object(child, 6.749 * length, 0.12, 1.5, asynch=False)
        coppelia.scale_object(wall_handle, 6.749 * length, 0.12, 1.5, asynch=False)

        self.handle = wall_handle


#
# Human
#  
class StandingHuman(CoppeliaHandle):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
        super(StandingHuman, self).__init__(coppelia, handle)


class Human(StandingHuman):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
        super(Human, self).__init__(coppelia, handle)
        self.dummy_handle = coppelia.get_objects_children(handle, 'sim.object_dummy_type')[0]
        # self.move(*coppelia.get_object_position(handle))

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
                self.wheel1 = h
            elif 'rollingJoint_rl' in name:
                self.wheel2 = h
            elif 'rollingJoint_rr' in name:
                self.wheel3 = h
            elif 'rollingJoint_fr' in name:
                self.wheel4 = h

    @staticmethod
    def get_position_offsets():
        return 0., 0., 0.095

    @staticmethod
    def get_orientation_offsets():
        return 0., -1.57, 0.

    def set_velocity(self, forwBackVel, leftRightVel, rotVel, asynch=False):
        # self.c.set_joint_target_velocity(self.wheel1, -forwBackVel-leftRightVel-rotVel, asynch=asynch)
        # self.c.set_joint_target_velocity(self.wheel2, -forwBackVel+leftRightVel-rotVel, asynch=asynch)
        # self.c.set_joint_target_velocity(self.wheel3, -forwBackVel-leftRightVel+rotVel, asynch=asynch)
        # self.c.set_joint_target_velocity(self.wheel4, -forwBackVel+leftRightVel+rotVel, asynch=asynch)
        # print('VELS:', forwBackVel, leftRightVel, rotVel)
        code = f"""sim.setJointTargetVelocity({self.wheel1}, {-forwBackVel - leftRightVel - rotVel}) and
                   sim.setJointTargetVelocity({self.wheel2}, {-forwBackVel + leftRightVel - rotVel}) and
                   sim.setJointTargetVelocity({self.wheel3}, {-forwBackVel - leftRightVel + rotVel}) and
                   sim.setJointTargetVelocity({self.wheel4}, {-forwBackVel + leftRightVel + rotVel})"""
        # print('CODE:', code)
        print(self.c.run_script(code))


class OmniPlatform(CoppeliaHandle):
    def __init__(self, coppelia: 'CoppeliaSimAPI', handle: int):
        super(OmniPlatform, self).__init__(coppelia, handle)

    @staticmethod
    def get_position_offsets():
        return 0., 0., 0.095

    @staticmethod
    def get_orientation_offsets():
        return 0., -1.57, 0.


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
                self.left_motor = h
            elif 'Pioneer_p3dx_rightMotor' in name:
                self.right_motor = h

    @staticmethod
    def get_position_offsets():
        return 0., 0., 0.095

    @staticmethod
    def get_orientation_offsets():
        return 0., 0., 0.

    def set_velocity(self, adv, rot, asynch=False):
        axisLength = 0.381
        wheelRadius = 0.0975
        left = (adv - (rot * axisLength) / 2.) / wheelRadius
        right = (adv + (rot * axisLength) / 2.) / wheelRadius
        self.c.set_joint_target_velocity(self.left_motor, left, asynch=asynch)
        self.c.set_joint_target_velocity(self.right_motor, right, asynch=asynch)
        # self.c.set_joint_target_velocity(self.left_motor,  forw_back_vel)
        # self.c.set_joint_target_velocity(self.right_motor, forw_back_vel)


# CoppeliaSimAPI
class CoppeliaSimAPI(object):
    def __init__(self, paths=[]):
        super(CoppeliaSimAPI, self).__init__()
        self.coppelia_paths = paths + ['./', os.environ['COPPELIASIM_ROOT'] + '/']
        self.client = self.create_client()  # 'b0RemoteApi_pythonClient', 'b0RemoteApiAddOn')

    @staticmethod
    def create_client(a=None, b=None):
        # print(f'Creating self.client {threading.get_ident()}')
        c = b0RemoteApi.RemoteApiClient(a, b)
        # print('XX created')
        return c

    def load_scene(self, scene_path, asynch=False):
        call = self.get_call_object(asynch)
        for source in self.coppelia_paths:
            full_path = source + '/' + scene_path
            if path.exists(full_path):
                return self.client.simxLoadScene(os.path.abspath(full_path), call.get())

    def spinOnce(self):
        self.client.simxSpinOnce()

    def get_call_object(self, asynch):
        if type(asynch) is CallType:
            return asynch

        if asynch == False:
            call = CallType(self.client.simxServiceCall())
        else:
            if asynch == True:
                call = CallType(self.client.simxDefaultPublisher())
            else:
                call = CallType(self.client.simxDefaultSubscriber(asynch))
        return call

    def start(self, asynch=False):
        call = self.get_call_object(asynch)
        self.client.simxStartSimulation(call.get())

    def stop(self, asynch=False):
        call = self.get_call_object(asynch)
        self.client.simxStopSimulation(call.get())

    def get_objects_children(self, parent='sim.handle_scene', children_type=None, filter_children=1 + 2, asynch=False):
        call = self.get_call_object(asynch)
        if children_type is None:
            children_type = 'sim.handle_all'
        ret = self.client.simxGetObjectsInTree(parent, children_type, filter_children, call.get())
        if asynch is False:
            if ret[0] is True:
                return ret[1]
            raise Exception(f'Error getting human\'s children {parent}.')

    def set_object_parent(self, obj, parent, keep_in_place=True):
        obj = self.convert_to_valid_handle(obj)
        parent = self.convert_to_valid_handle(parent)
        code = f'sim.setObjectParent({obj}, {parent}, {keep_in_place})'
        ret = self.run_script(code)
        return ret

    def get_object_name(self, handle, alternative_name=''):
        call = self.get_call_object(False)
        obj = self.convert_to_valid_handle(handle)
        ret = self.client.simxGetObjectName(handle, alternative_name, call.get())
        if ret[0] is True:
            return ret[1]
        raise Exception(f'Can\'t get name for object handle {handle}.')

    def create_human(self, x, y, z, angle, asynch=False):
        model = 'models/people/path planning Bill.ttm'
        human_handle = self.create_model(model, x, y, z, angle, asynch=asynch)
        return Human(self, human_handle)

    def create_standing_human(self, x, y, z, angle):
        model = 'models/people/Standing Bill.ttm'
        human_handle = self.create_model(model, x, y, z, angle)
        return StandingHuman(self, human_handle)

    def create_wall(self, p1, p2):
        return Wall(self, p1, p2)

    def create_model(self, model, x=None, y=None, z=None, rz=None, asynch=False):
        do_transform = False
        if x is not None and y is not None and z is not None:
            do_transform = True
        call = self.get_call_object(asynch)
        for source in self.coppelia_paths:
            full_path = source + '/' + model
            if path.exists(full_path):
                ret = self.client.simxLoadModelFromFile(os.path.abspath(full_path), call.get())
                if not asynch:
                    if ret[0] is not True:
                        raise Exception(f'Error creating model {full_path}.')
                if x is not None and y is not None and z is not None:
                    self.set_object_transform(ret[1], x, y, z, rz, asynch=asynch)
                return ret[1]
        raise Exception(f'Couldn\'t find model {model} in any of the paths {self.coppelia_paths}')

    def get_object_handle(self, object_name, asynch=False):
        call = self.get_call_object(asynch)
        obj = self.client.simxGetObjectHandle(object_name, call.get())
        if obj[0] is True:
            return obj[1]
        raise Exception(f'Can\'t get object handle for object {object_name}')

    def set_object_transform(self, obj, x, y, z, angle, asynch=False):
        call = self.get_call_object(asynch)
        obj = self.convert_to_valid_handle(obj)
        M = CoppeliaSimAPI.get_transform_matrix(x, y, z, angle)
        ret = self.client.simxSetObjectMatrix(obj, -1, M, call.get())
        return ret

    def convert_to_valid_handle(self, obj, asynch=False):
        call = self.get_call_object(asynch)
        if isinstance(obj, CoppeliaHandle):
            obj = obj.handle
        elif type(obj) is str:
            if obj not in ['sim.handle_parent']:
                obj = self.client.simxGetObjectHandle(obj, call.get())[1]
        return obj

    def set_object_position(self, obj, x, y, z, reference='sim.handle_parent', asynch=False):
        call = self.get_call_object(asynch)
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        ret = self.client.simxSetObjectPosition(obj, reference, [x, y, z], call.get())
        return ret

    def get_object_position(self, obj, reference=-1, asynch=False):
        call = self.get_call_object(asynch)
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        ret = self.client.simxGetObjectPosition(obj, reference, call.get())
        if asynch == False:
            if ret[0] is not True:
                raise Exception(f'CoppeliaSimAPI: get_object_position: Can\'t find object {obj}.')
            return ret[1]

    def set_object_orientation(self, obj, alpha, betta, gamma, reference='sim.handle_parent', asynch=False):
        call = self.get_call_object(asynch)
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        return self.client.simxSetObjectOrientation(obj, reference, [alpha, betta, gamma], call.get())

    def get_object_orientation(self, obj, reference=-1, asynch=False):
        call = self.get_call_object(asynch)
        obj = self.convert_to_valid_handle(obj)
        reference = self.convert_to_valid_handle(reference)
        ret = self.client.simxGetObjectOrientation(obj, reference, call.get())
        if asynch == False:
            if ret[0] is not True:
                raise Exception(f'CoppeliaSimAPI: get_object_orientation: Can\'t find object {obj}.')
            return ret[1]

    def remove_object(self, obj, asynch=False):
        call = self.get_call_object(asynch)
        if type(obj) is str:
            sobj = self.client.simxGetObjectHandle(obj, call.get())[1]
            obj = sobj
        return self.client.simxRemoveObjects([obj], 1, call.get())

    def run_script(self, script, asynch=False):
        call = self.get_call_object(asynch)
        return self.client.simxExecuteScriptString(script, call.get())

    def scale_object(self, handle, sx, sy, sz, asynch=False):
        return self.run_script(f'sim.scaleObject({handle},{sx},{sy},{sz},0)', asynch)

    def close(self, asynch=False):
        call = self.get_call_object(asynch)
        return self.client.simxCloseScene(call.get())

    # NOT INCLUDED IN THE DOCUMENTATION YET
    def get_youbot(self) -> YouBot:
        children = self.get_objects_children('sim.handle_scene', children_type='sim.object_shape_type',
                                             filter_children=1 + 2)
        for h in children:
            name = self.get_object_name(h)
            if name == 'youBot':
                return YouBot(self, h)

    def get_omniplatform(self) -> OmniPlatform:
        children = self.get_objects_children('sim.handle_scene', children_type='sim.object_shape_type',
                                             filter_children=1 + 2)
        for h in children:
            name = self.get_object_name(h)
            if name == 'OmniPlatform':
                return OmniPlatform(self, h)

    def create_youbot(self, x, y, z) -> YouBot:
        ix, iy, iz = YouBot.get_position_offsets()
        ret = self.create_model('models/robots/mobile/KUKA YouBot.ttm', x + ix, y + iy, z + iz, 0.)
        self.set_object_orientation(ret, *YouBot.get_orientation_offsets())
        return YouBot(self, ret)

    def create_pioneer_p3dx(self, x, y, z) -> Pioneer_p3dx:
        ix, iy, iz = Pioneer_p3dx.get_position_offsets()
        ret = self.create_model('models/robots/mobile/pioneer p3dx.ttm', x + ix, y + iy, z + iz, 0.)
        self.set_object_orientation(ret, *Pioneer_p3dx.get_orientation_offsets())
        return Pioneer_p3dx(self, ret)

    def get_pioneer_p3dx(self) -> Pioneer_p3dx:
        children = self.get_objects_children('sim.handle_scene', children_type='sim.object_shape_type',
                                             filter_children=1 + 2)
        for h in children:
            name = self.get_object_name(h)
            if name == 'Pioneer_p3dx':
                return Pioneer_p3dx(self, h)

    def set_joint_target_velocity(self, handle, target, asynch=False):
        call = self.get_call_object(asynch)
        return self.client.simxSetJointTargetVelocity(handle, target, call.get())

    def pause(self):
        call = self.get_call_object(asynch)
        self.client.simxPauseSimulation(call.get())

    def check_collision(self, obj1, obj2, asynch=False):
        call = self.get_call_object(asynch)
        obj1 = self.convert_to_valid_handle(obj1)
        obj2 = self.convert_to_valid_handle(obj2)

        success, collision_state = self.client.simxCheckCollision(obj1, obj2, call.get())
        return collision_state and success

    def set_collidable(self, obj, asynch=False):
        handle = self.convert_to_valid_handle(obj)
        return self.run_script(f'sim.setObjectSpecialProperty({handle},sim.objectspecialproperty_collidable+'
                               f'sim.objectspecialproperty_measurable+sim.objectspecialproperty_detectable_all'
                               f'+sim.objectspecialproperty_renderable)', asynch)

    @staticmethod
    def get_transform_matrix(x, y, z, angle):
        rotate_matrix = np.matrix([[cos(angle), -sin(angle), 0., 0.],
                                   [sin(angle), cos(angle), 0., 0.],
                                   [0., 0., 1., 0.],
                                   [0., 0., 0., 1.]])
        translate_matrix = np.matrix([[1., 0., 0., x],
                                      [0., 1., 0., y],
                                      [0., 0., 1., z],
                                      [0., 0., 0., 1.]])
        return (translate_matrix @ rotate_matrix).flatten().tolist()[0]

    @staticmethod
    def get_transformation_matrix(x, z, angle):
        M = np.zeros((3, 3))
        M[0][0], M[0][1], M[0][2] = +cos(-angle), -sin(-angle), x
        M[1][0], M[1][1], M[1][2] = +sin(-angle), +cos(-angle), z
        M[2][0], M[2][1], M[2][2] = 0., 0., 1.
        return M
