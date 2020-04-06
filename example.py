#!/usr/bin/env python3

import time
import random
import pygame
from coppeliasimapi import CoppeliaSimAPI, YouBot

import setproctitle
setproctitle.setproctitle('coppeliaExample1')

pygame.display.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()


# The CoppeliaSimAPI constructor receives a list of paths that the object will use to find models.
# Don't worry about the models in the CoppeliaSim path, the object will automatically append 
# $COPPELIASIM_ROOT to the list, so there is no need to provide any parameter if you are only using
# the models provided by the simulator.
coppelia = CoppeliaSimAPI(['./models/'])



# Stop the simulator and close the scene, just in case.
coppelia.stop()
coppelia.close()
coppelia.stop()
coppelia.close()

# Move the floor downwards, as we want to use a prettier floor.
print('Getting floor name')
floor = coppelia.get_object_handle('ResizableFloor_5_25')
print('ret:', floor)
coppelia.set_object_transform('ResizableFloor_5_25', 0.0,  0.0, -1.0, 0)

# We are going to put our floor 10 metres up, just because we can, so we better move the camera too.
coppelia.set_object_position('DefaultCamera',       5.0, -7.0,  14.)

# Create and scale a floor
r = coppelia.create_model('models/infrastructure/floors/5mX5m wooden floor.ttm', 0, 0, 10., 1.57)
print('Floor:', r)
print(coppelia.scale_object(r, 2, 2, 1))
for handle in coppelia.get_objects_children(r):
    print(coppelia.scale_object(handle, 2, 2, 1))

# Add two humans
a = coppelia.create_human(10.*(random.random()-0.5),               # x
                            10.*(random.random()-0.5),             # y
                            10., 2.*3.1415926535*random.random())  # z, angle
b = coppelia.create_human(10.*(random.random()-0.5),               # x
                            10.*(random.random()-0.5),             # y
                            10., 2.*3.1415926535*random.random())  # z, angle

# Add four walls
coppelia.create_wall([ 5.,  5., 10.4], [ 5., -5., 10.4])
coppelia.create_wall([ 5., -5., 10.4], [-5., -5., 10.4])
coppelia.create_wall([-5., -5., 10.4], [-5.,  5., 10.4])
coppelia.create_wall([-5.,  5., 10.4], [ 5.,  5., 10.4])

# Create a YouBot
youbot = coppelia.create_youbot(2, 2, 10.095)
print('YouBot\'s handle:', youbot)

# Create a Pioneer_p3dx
pioneer = coppelia.create_pioneer_p3dx(0, 0, 10.195)
print('Pioneer p3dx\'s handle:', pioneer)

# Start the simulation
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')

time.sleep(10)


coppelia.start()
coppelia.run_script('sim.setBoolParameter(sim.boolparam_realtime_simulation,true)')


# Loop
adv = rot = 0.
last_ten = time.time()-10
last_point_one = time.time()-10
while True:
    # pygame.event.pump()
    # print('v', adv, rot)

    # EVERY 10 seconds
    if time.time() - last_ten > 10:
        last_ten = time.time()
        youbot.set_velocity(2, 2, 0)
        # Move the humans every 12.0 * 0.5 seconds
        a.move(10.*(random.random()-0.5), 10.*(random.random()-0.5), 1.)
        b.move(10.*(random.random()-0.5), 10.*(random.random()-0.5), 1.)
    # EVERY 0.1 seconds
    if time.time() - last_point_one > 0.5:
        last_point_one = time.time()
        pygame.event.pump()
        rot = 3.*pygame.joystick.Joystick(0).get_axis(0)
        adv = 2.*pygame.joystick.Joystick(0).get_axis(1)
        print('send', adv, rot)
        pioneer.set_velocity(adv, rot)
    time.sleep(0.1)
    # print('aaaaaaaaaaaaa')
    # coppelia.client.simxSpinOnce()
    # coppelia.client.simxSleep(50)
 

