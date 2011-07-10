# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

from header import *

class ForceField:

  def __init__(self, name):
    # for force field arrows (one per course, or particles)
    # particles: (x, y, (length_x, length_y), val)
    self.particles = list()
    self.name = name


