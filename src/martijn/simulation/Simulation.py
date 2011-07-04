# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

import pygame, sys, math, random
from ARDrone import *

class Simulation:

  def __init__(self, width, height, course, spawn):
    pygame.init()
    self.width  = width
    self.height = height
    self.course = course
    self.scr = pygame.display.set_mode((self.width, self.height))
    self.objects = list() # for border and pylons
    # for force field arrows (one per course, or particles)
    # particles: (x, y, (length_x, length_y), val)
    self.particles = list()
    val = 1 # could be used for speed
    x = int(course/2)
    while (x < width):
      y = int(course/2)
      while (y < height):
        self.particles.append( (x, y, (0, 0), val) )
        #self.particles.append( (x, y, (random.randint(-20,20), random.randint(-20,20)), val) )
        y += course
      x += course
    self.ardrone = ARDrone(spawn, self)



  ## 'setters'

  # shape: 'rect', 'circle'
  # width: 0 for filled
  # color: (r, g, b)
  # coord: (x, y) or r for circles
  def add_object(self, shape, width, color, coord1, coord2):
    self.objects.append( (shape, width, color, coord1, coord2) )

  # force distance function (scalar as weight)
  def fdv(self, distance, maximum):
    return 0.5 * math.pow(2, 1 - 5.0*distance/maximum)

  def add_object_forcefields(self, radius, strength):
    # for each object, search for particles within radius
    for obj in self.objects:
      shape, width, color, coord1, coord2 = obj
      for i in range(len(self.particles)):
        x, y, vect, val = self.particles[i]
        if shape == 'rect':
          dist = 0
          vect_new = vect
          if abs(x - coord1[0]) < radius:   # left
            dist = abs(x - coord1[0])
            vect_force = (1, 0)
            scal_force = strength * self.fdv(dist, radius)
            vect_force = (vect_force[0]*scal_force, \
                          vect_force[1]*scal_force)
            vect_new = self.add(vect_new, vect_force)
          if abs(x - coord2[0]) < radius: # right
            dist = abs(x - coord2[0])
            vect_force = (-1, 0)
            scal_force = strength * self.fdv(dist, radius)
            vect_force = (vect_force[0]*scal_force, \
                          vect_force[1]*scal_force)
            vect_new = self.add(vect_new, vect_force)
          if abs(y - coord1[1]) < radius: # top
            dist = abs(y - coord1[1])
            vect_force = (0, 1)
            scal_force = strength * self.fdv(dist, radius)
            vect_force = (vect_force[0]*scal_force, \
                          vect_force[1]*scal_force)
            vect_new = self.add(vect_new, vect_force)
          if abs(y - coord2[1]) < radius: # bottom
            dist = abs(y - coord2[1])
            vect_force = (0, -1)
            scal_force = strength * self.fdv(dist, radius)
            vect_force = (vect_force[0]*scal_force, \
                          vect_force[1]*scal_force)
            vect_new = self.add(vect_new, vect_force)
          if dist != 0: # finish one of above
            self.particles[i] = (x, y, vect_new, val)

        elif shape == 'circle':
          if self.dist((x, y), coord1) - coord2 < radius:
            # add some of the forcefield_vector
            scal_force = strength * self.fdv(self.dist((x, y), \
                         coord1), (radius - coord2))
            vect_force = self.unit(self.diff((x,y), coord1))
            vect_force = (vect_force[0]*scal_force, \
                          vect_force[1]*scal_force)
            vect_new = self.add(vect, vect_force)
            self.particles[i] = (x, y, vect_new, val)
          
  def add_rounded_forcefield(self, loc, r, radius, strength, angle):
    coord1 = loc
    coord2 = r
    # search for particles within radius
    for i in range(len(self.particles)):
      x, y, vect, val = self.particles[i]
      if self.dist((x, y), coord1) - coord2 < radius:
        scal_force = strength * self.fdv(self.dist((x, y), \
                     coord1), (radius - coord2))
        vect_force = self.unit(self.rot(self.diff((x,y), coord1), angle))
        vect_force = (vect_force[0]*scal_force, \
                      vect_force[1]*scal_force)
        vect_new = self.add(vect, vect_force)
        self.particles[i] = (x, y, vect_new, val)
        


  ## 'getters'

  def get_nearest_particle(self, loc, notlist=list()):
    if len(self.particles) == 0:
      return None
    nearest  = None
    distance = 1000000
    for x, y, vect, val in self.particles:
      temp = self.dist(loc, (x,y))
      if temp < distance and (x, y) not in notlist:
        nearest  = (x, y, vect, val)
        distance = temp
    return nearest

  def get_force(self, loc):
    # get force: now nearest neighbour
    # TODO: some interpolation
    x, y, vect, val = self.get_nearest_particle(loc)
    return vect
    

    
  ## drawing

  def show(self):
    self.scr.fill( (10, 10, 10) )
    # show objects
    for obj in self.objects:
      shape, width, color, coord1, coord2 = obj
      if shape == 'rect':
        pygame.draw.rect(self.scr, color, (coord1[0], coord1[1], coord2[0], coord2[1]), width)
      elif shape == 'circle':
        pygame.draw.circle(self.scr, color, coord1, coord2, width)
      else:
        print "Unknown shape: " + shape
    # show particles
    for x, y, vect, val in self.particles:
      self.draw_vector(x, y, vect, (50, 50, 255))
    # show ardrone
    self.ardrone.draw(self.scr)


    # display
    pygame.display.flip()


  def draw_vector(self, x, y, vect, color, size=8):
    x2, y2 = x+vect[0], y+vect[1]
    pygame.draw.line(self.scr, color, (x, y), (x2, y2), 1)
    # calculate arrow lines
    norm = self.norm(vect)
    if norm > 0:
      size = min(norm, size)
      r = 1.5
      p = 0.5
      unit = self.unit(vect)
      reverse = (-1*unit[0], -1*unit[1])
      perp1   = (-1*unit[1],  1*unit[0])
      perp2   = ( 1*unit[1], -1*unit[0])
      line1 = (x2 + int((size / math.sqrt(2)) * (r*reverse[0] + p*perp1[0])), \
               y2 + int((size / math.sqrt(2)) * (r*reverse[1] + p*perp1[1])))
      line2 = (x2 + int((size / math.sqrt(2)) * (r*reverse[0] + p*perp2[0])), \
               y2 + int((size / math.sqrt(2)) * (r*reverse[1] + p*perp2[1])))
      pygame.draw.line(self.scr, color, (x2, y2), line1, 1)
      pygame.draw.line(self.scr, color, (x2, y2), line2, 1)


  ## math && linear algebra

  def norm(self, vect):
    return math.sqrt(vect[0]*vect[0] + vect[1]*vect[1])

  def unit(self, vect):
    norm = self.norm(vect)
    if norm == 0:
      return None # is scalar in GA
    else:
      return ( float(vect[0]) / norm, float(vect[1]) / norm )

  def dot(self, vect1, vect2):
    return vect1[0] * vect2[0] + vect1[1] * vect2[1]

  def add(self, vect1, vect2):
    return (vect1[0] + vect2[0], vect1[1] + vect2[1])

  def diff(self, vect1, vect2):
    return (vect1[0] - vect2[0], vect1[1] - vect2[1])

  def dist(self, vect1, vect2):
    return math.sqrt(math.pow(vect1[0] - vect2[0], 2) + \
                     math.pow(vect1[1] - vect2[1], 2))

  def rot(self, vect, angle):
    angle = math.radians(angle)
    return (math.cos(angle)*vect[0] - math.sin(angle)*vect[1], \
            math.sin(angle)*vect[0] + math.cos(angle)*vect[1])









