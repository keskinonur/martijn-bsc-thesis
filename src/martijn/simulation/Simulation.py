# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

from header import *
import sys
import time
import random
import math
import pygame
from LA import *             # linear algebra functions
from Simulation import *     # simulation object (window, ardrone, field)
from ForceField import *     # force field object
from ARDrone import *        # ARDrone object

class Simulation:

  def __init__(self, width, height, spawn):
    pygame.init()
    self.width  = width
    self.height = height
    self.scr = pygame.display.set_mode((self.width, self.height))
    self.objects = list() # for border and pylons
    self.stages = dict()  # for stages with particles each
    self.stage = None     # current stage (string)
    self.transitions = list() # transition edges
    val = 1 # could be used for speed
    self.ardrone = ARDrone(spawn, self)



  ## state functions
  
  def check_for_state_transition(self, loc_new, loc_old):
    print "TODO: check for state transition"
    for stage_from, stage_to, edge in self.transitions:
      if stage_from == self.stage:
        edge_type, coord1, coord2 = edge
        # check for crossing
        if edge[0] == "line":
          pass


  ## 'setters'

  # shape: 'rect', 'circle'
  # width: 0 for filled
  # color: (r, g, b)
  # coord: (x, y) or r for circles
  def add_object(self, name, shape, width, color, coord1, coord2):
    self.objects.append( (name, shape, width, color, coord1, coord2) )

  # force distance function (scalar as weight)
  def fdv(self, distance, maximum):
    return 0.5 * math.pow(2, 1 - 5.0*distance/maximum)

  def add_particles_uniform(self, stage_name, coarse):
    val = 0
    stages = list()
    if stage_name == "all":
      stages = self.stages.keys()
    else:
      stages = [stage_name]
    for stage in stages:
      x = int(coarse/2)
      while (x < self.width):
        y = int(coarse/2)
        while (y < self.height):
          self.stages[stage].particles.append( (x, y, (0, 0), val) )
          #self.stages[stage].particles.append( (x, y, (random.randint(-20,20), random.randint(-20,20)), val) )
          y += coarse
        x += coarse

  def add_object_forcefields(self, stage_name, radius, strength):
    stages = list()
    if stage_name == "all":
      stages = self.stages.keys()
    else:
      stages = [stage_name]
    for stage in stages:
      # for each object, search for particles within radius
      for obj in self.objects:
        name, shape, width, color, coord1, coord2 = obj
        for i in range(len(self.stages[stage].particles)):
          x, y, vect, val = self.stages[stage].particles[i]
          if shape == 'rect':
            distance = 0
            vect_new = vect
            if abs(x - coord1[0]) < radius:   # left
              distance = abs(x - coord1[0])
              vect_force = (1, 0)
              scal_force = strength * self.fdv(distance, radius)
              vect_force = (vect_force[0]*scal_force, \
                            vect_force[1]*scal_force)
              vect_new = add(vect_new, vect_force)
            if abs(x - coord2[0]) < radius: # right
              distance = abs(x - coord2[0])
              vect_force = (-1, 0)
              scal_force = strength * self.fdv(distance, radius)
              vect_force = (vect_force[0]*scal_force, \
                            vect_force[1]*scal_force)
              vect_new = add(vect_new, vect_force)
            if abs(y - coord1[1]) < radius: # top
              distance = abs(y - coord1[1])
              vect_force = (0, 1)
              scal_force = strength * self.fdv(distance, radius)
              vect_force = (vect_force[0]*scal_force, \
                            vect_force[1]*scal_force)
              vect_new = add(vect_new, vect_force)
            if abs(y - coord2[1]) < radius: # bottom
              distance = abs(y - coord2[1])
              vect_force = (0, -1)
              scal_force = strength * self.fdv(distance, radius)
              vect_force = (vect_force[0]*scal_force, \
                            vect_force[1]*scal_force)
              vect_new = add(vect_new, vect_force)
            if distance != 0: # finish one of above
              self.stages[stage].particles[i] = (x, y, vect_new, val)
  
          elif shape == 'circle':
            if dist((x, y), coord1) - coord2 < radius:
              # add some of the forcefield_vector
              scal_force = strength * self.fdv(dist((x, y), \
                           coord1), (radius - coord2))
              vect_force = unit(diff((x,y), coord1))
              vect_force = (vect_force[0]*scal_force, \
                            vect_force[1]*scal_force)
              vect_new = add(vect, vect_force)
              self.stages[stage].particles[i] = (x, y, vect_new, val)
          
  def add_rounded_forcefield(self, stage_name, name, radius, strength, angle):
    name, shape, width, color, coord1, coord2 = self.get_object_by_name(name)
    stages = list()
    if stage_name == "all":
      stages = self.stages.keys()
    else:
      stages = [stage_name]
    for stage in stages:
      # search for particles within radius
      for i in range(len(self.stages[stage].particles)):
        x, y, vect, val = self.stages[stage].particles[i]
        if dist((x, y), coord1) - coord2 < radius:
          scal_force = strength * self.fdv(dist((x, y), \
                       coord1), (radius - coord2))
          vect_force = unit(rot(diff((x,y), coord1), angle))
          vect_force = (vect_force[0]*scal_force, \
                        vect_force[1]*scal_force)
          vect_new = add(vect, vect_force)
          self.stages[stage].particles[i] = (x, y, vect_new, val)
        
  def add_stage(self, name):
    self.stages[name] = ForceField(name)

  def add_stage_transition(self, stage_from, stage_to, edge):
    stages = list()
    if stage_from == "all":
      stages = self.stages.keys()
    else:
      stages = [stage_from]
    for stage in stages:
      self.transitions.append( (stage, stage_to, edge) )


  ## 'getters'

  def get_nearest_particle(self, loc, notlist=list()):
    if len(self.stages[self.stage].particles) == 0:
      return None
    nearest  = None
    distance = 1000000
    for x, y, vect, val in self.stages[self.stage].particles:
      temp = dist(loc, (x,y))
      if temp < distance and (x, y) not in notlist:
        nearest  = (x, y, vect, val)
        distance = temp
    return nearest

  def get_force(self, loc):
    # get force: now nearest neighbour
    # TODO: some interpolation
    x, y, vect, val = self.get_nearest_particle(loc)
    return vect
    
  def get_object_by_name(self, name):
    for obj in self.objects:
      if obj[0] == name:
        return obj    # found object
    return None       # object not found

    
  ## drawing

  def show(self):
    self.scr.fill( (10, 10, 10) )
    # show objects
    for obj in self.objects:
      name, shape, width, color, coord1, coord2 = obj
      if shape == 'rect':
        pygame.draw.rect(self.scr, color, (coord1[0], coord1[1], coord2[0], coord2[1]), width)
      elif shape == 'circle':
        pygame.draw.circle(self.scr, color, coord1, coord2, width)
      else:
        print "Unknown shape: " + shape
    # show particles
    if len(self.stages) == 0:
      print "No stages defined!"
    else:
      if self.stage == None:
        self.stage = self.stages.keys()[0]
      for x, y, vect, val in self.stages[self.stage].particles:
        self.draw_vector(x, y, vect, (50, 50, 255))
    # show transitions
    for stage_from, stage_to, edge in self.transitions:
      if stage_from == self.stage:
        edge_type, coord1, coord2 = edge
        if edge[0] == "line":
          pygame.draw.line(self.scr, (150, 0, 200), coord1, coord2, 2)
        #else: other edge types?
      #else: show inactive transitions in other color?

    # show ardrone
    self.ardrone.draw(self.scr)


    # display
    pygame.display.flip()


  def draw_vector(self, x, y, vect, color, size=8):
    x2, y2 = x+vect[0], y+vect[1]
    pygame.draw.line(self.scr, color, (x, y), (x2, y2), 1)
    # calculate arrow lines
    vect_norm = norm(vect)
    if vect_norm > 0:
      size = min(vect_norm, size)
      r = 1.5
      p = 0.5
      u_vect = unit(vect)
      reverse = (-1*u_vect[0], -1*u_vect[1])
      perp1   = (-1*u_vect[1],  1*u_vect[0])
      perp2   = ( 1*u_vect[1], -1*u_vect[0])
      line1 = (x2 + int((size / math.sqrt(2)) * (r*reverse[0] + p*perp1[0])), \
               y2 + int((size / math.sqrt(2)) * (r*reverse[1] + p*perp1[1])))
      line2 = (x2 + int((size / math.sqrt(2)) * (r*reverse[0] + p*perp2[0])), \
               y2 + int((size / math.sqrt(2)) * (r*reverse[1] + p*perp2[1])))
      pygame.draw.line(self.scr, color, (x2, y2), line1, 1)
      pygame.draw.line(self.scr, color, (x2, y2), line2, 1)











