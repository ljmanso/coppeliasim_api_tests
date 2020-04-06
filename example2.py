#!/usr/bin/env python3

import time
import random
import pygame
from coppeliasimapi import CoppeliaSimAPI, YouBot, Pioneer_p3dx

import setproctitle
setproctitle.setproctitle('coppeliaExample1')

from math import pi

pygame.display.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()


# The CoppeliaSimAPI constructor receives a list of paths that the object will use to find models.
# Don't worry about the models in the CoppeliaSim path, the object will automatically append 
# $COPPELIASIM_ROOT to the list, so there is no need to provide any parameter if you are only using
# the models provided by the simulator.
coppelia = CoppeliaSimAPI(['./scenes/'])

# Stop the simulator and close the scene, just in case.
coppelia.stop()
coppelia.close()
coppelia.stop()
coppelia.close()

# Load a basic scene
coppelia.load_scene('dataset.ttt')


children = coppelia.get_objects_children('sim.handle_scene', children_type='sim.all_type', filter_children=1+2)
for child in children:
    name = coppelia.get_object_name(child)
    # Get the youbot that is already available in the scene
    if name == 'youBot':
        youbot = YouBot(coppelia, child)
        print('YouBot\'s handle:', youbot)
    # Get the Pioneer P3DX that is already available in the scene
    elif name == 'Pioneer_p3dx':
        pioneer = Pioneer_p3dx(coppelia, child)
        print('Pioneer p3dx\'s handle:', pioneer)
    elif name == 'Camera':
        perspective = child
        print('Perspective camera has handle:', perspective)


coppelia.set_object_parent(perspective, pioneer, True)
coppelia.set_object_position(perspective, 5, 0, 3.5, pioneer) # back, sideways, up
coppelia.set_object_orientation(perspective, 0., 0., 0., pioneer)
coppelia.set_object_orientation(perspective, 0, -pi/2., 0, perspective)
coppelia.set_object_orientation(perspective, 0.4, 0, 0, perspective)

# Add two humans
a = coppelia.create_human(10.*(random.random()-0.5),               # x
                            10.*(random.random()-0.5),             # y
                            0., 2.*3.1415926535*random.random())  # z, angle
b = coppelia.create_human(10.*(random.random()-0.5),               # x
                            10.*(random.random()-0.5),             # y
                            0., 2.*3.1415926535*random.random())  # z, angle

# Add four walls
coppelia.create_wall([ 5.,  5., 0.4], [ 5., -5., 0.4])
coppelia.create_wall([ 5., -5., 0.4], [-5., -5., 0.4])
coppelia.create_wall([-5., -5., 0.4], [-5.,  5., 0.4])
coppelia.create_wall([-5.,  5., 0.4], [ 5.,  5., 0.4])


# Start the simulation
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')
coppelia.start()
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')


# Loop
adv = rot = 0.
last_ten = time.time()-10
last_point_one = time.time()-10
while True:
    # EVERY 10 seconds
    if time.time() - last_ten > 10:
        last_ten = time.time()
        youbot.set_velocity(0.12, -0.12, 0)
        # Move the humans every 12.0 * 0.5 seconds
        a.move(10.*(random.random()-0.5), 10.*(random.random()-0.5), -1.)
        b.move(10.*(random.random()-0.5), 10.*(random.random()-0.5), -1.)
    # EVERY 0.1 seconds
    if time.time() - last_point_one > 0.5:
        last_point_one = time.time()
        pygame.event.pump()
        rot = -0.5.*pygame.joystick.Joystick(0).get_axis(0)
        adv = +1.*pygame.joystick.Joystick(0).get_axis(1)
        pioneer.set_velocity(adv, rot)
    time.sleep(0.033)


