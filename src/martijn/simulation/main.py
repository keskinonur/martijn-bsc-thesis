# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

import sys, time, math
from Simulation import *
from ARDrone import *

def discrete(width, height, course, spawn):
  # open window
  sim = Simulation(width, height, coarse, spawn)
  center = ( int(width / 2), int(height / 2) )
  # make border
  sim.add_object('rect',   4, (255, 50, 0), (  3,   3), (width-9, height-9))
  # make pylons
  sim.add_object('circle', 0, (255, 50, 0), (1*int(center[0]/2), center[1]), int(center[1]/40))
  sim.add_object('circle', 0, (255, 50, 0), (3*int(center[0]/2), center[1]), int(center[1]/40))
  # add forcefields for objects
  sim.show()
  raw_input()
  sim.add_object_forcefields(150, 20)
  sim.show()
  raw_input()
  sim.add_rounded_forcefield((600,300), int(center[1]/40), 300, 30, 90)
  sim.add_rounded_forcefield((200,300), int(center[1]/40), 300, 30, -90)
  
  # draw
  timestep  = 0.5
  sleepstep = 0.001
  while True:
    sim.show()
    #sim.ardrone.theta += math.radians(5)
    sim.ardrone.move(timestep)
    sim.ardrone.explore(timestep, 0.3)
    time.sleep(sleepstep)


if __name__ == "__main__":
  # settings
  width  = 800
  height = 600
  coarse = 20  # pixels per coarse (for discrete simulations)
  spawn = (int(width/2), int(height/4))
  # play simulation
  discrete(width, height, coarse, spawn)



