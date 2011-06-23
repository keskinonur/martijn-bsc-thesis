# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

import math, random, pygame

class ARDrone:

  def __init__(self, location, simulation, color=(50, 255, 50)):
    self.color = color
    self.theta = 0
    self.speed = (0, 0)
    self.location = location
    self.simulation = simulation


  def draw(self, scr):
    # ardrone: 4 circles and a line
    r = 8
    rc = math.sqrt(2*r*r)
    x, y = self.location
    xdiff = int(rc * math.cos(self.theta + math.radians(45)))
    ydiff = int(rc * math.sin(self.theta + math.radians(45)))
    c = [(x + xdiff, y + ydiff),
         (x - ydiff, y + xdiff),
         (x + ydiff, y - xdiff),
         (x - xdiff, y - ydiff)]
    l_start = (x + int(r * math.cos(-1*self.theta)), \
               y - int(r * math.sin(-1*self.theta)))
    l_stop  = (x - 2*int(r * math.cos(-1*self.theta)), \
               y + 2*int(r * math.sin(-1*self.theta)))

    for circle in c:
      pygame.draw.circle(scr, self.color, circle, r, 2)
    pygame.draw.line(scr, self.color, l_start, l_stop, 2)


  def move(self, timestep, decay=0.0005):
    x,  y  = self.location
    vx, vy = self.speed
    fx, fy = self.simulation.get_force(self.location)
    self.location = (x + timestep * vx, y + timestep * vy)
    # TODO: make more realistici decay
    self.speed = (vx + timestep*fx - timestep*decay * vx*vx*vx, \
                  vy + timestep*fy - timestep*decay * vy*vy*vy)
    # for fun: set angle
    self.theta = math.acos(self.simulation.dot(self.simulation.unit(self.speed), (1, 0)))
    print self.speed
    print "with force " + str((fx, fy))
    
  def explore(self, timestep, maximum):
    x,  y  = self.location
    vx, vy = self.speed
    fx, fy = self.simulation.get_force(self.location)
    vect_e = self.simulation.unit((0.5 - random.random(), 0.5 - random.random()))
    scal_e = random.random() * maximum
    ex, ey = (scal_e * vect_e[0], scal_e * vect_e[1])
    self.location = (x + timestep * vx, y + timestep * vy)
    self.speed = (vx + timestep*(fx + ex), vy + timestep*(fy + ey))
    


