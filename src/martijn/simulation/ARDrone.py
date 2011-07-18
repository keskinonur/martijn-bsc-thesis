# simulation of RL techniques for ARDrone # spring 2011 # author: martijn van der veen # licence: GPLv3

from header import *

class ARDrone:

  def __init__(self, location, simulation, color=(50, 255, 50)):
    self.color = color
    self.theta = 0
    self.speed = (0, 0)
    self.location = location
    self.constant_speed = False
    self.maximum_speed = 30       # none or integer
    self.old_location = location
    self.simulation = simulation


  def draw(self, scr):
    # ardrone: 4 circles and a line
    r = 8
    rc = math.sqrt(2*r*r)
    x, y = self.location
    xdiff = int(rc * math.cos(self.theta + math.radians(45)))
    ydiff = int(rc * math.sin(self.theta + math.radians(45)))
    c = [(int(x + xdiff), int(y + ydiff)),
         (int(x - ydiff), int(y + xdiff)),
         (int(x + ydiff), int(y - xdiff)),
         (int(x - xdiff), int(y - ydiff))]
    l_start = (x + int(r * math.cos(-1*self.theta)), \
               y - int(r * math.sin(-1*self.theta)))
    l_stop  = (x - 2*int(r * math.cos(-1*self.theta)), \
               y + 2*int(r * math.sin(-1*self.theta)))

    for circle in c:
      pygame.draw.circle(scr, self.color, circle, r, 2)
    pygame.draw.line(scr, self.color, l_start, l_stop, 2)


  def move(self, timestep, decay=0.005):
    self.old_location = self.location
    x,  y  = self.location
    vx, vy = self.speed
    fx, fy = self.simulation.get_force(self.location)
    # update location
    self.location = (x + timestep * vx, y + timestep * vy)
    # update speed
    # TODO: make more realistic decay
    self.speed = (vx + timestep*fx - timestep*decay * vx*vx*vx, \
                  vy + timestep*fy - timestep*decay * vy*vy*vy)
    if self.constant_speed:
      if self.speed == (0, 0):
        self.speed = unit( (random.random(), random.random()) )
      else:
        self.speed = unit(self.speed)
    # maximum speed:
    if self.maximum_speed != None and norm(self.speed) > self.maximum_speed:
       print "! lowered speed from " + str(norm(self.speed)) + " to " + str(self.maximum_speed)
       speed_factor = self.maximum_speed / norm(self.speed)
       self.speed = (self.speed[0] / speed_factor, self.speed[1] / speed_factor)
    # for fun: set angle
    self.theta = math.acos(dot(unit(self.speed), (1, 0)))

    # do reinforcement learning steps
    self.simulation.reinf_learn( self.location, self.old_location, (vx,vy) )
    
  def explore(self, timestep, maximum, decay=0.005):
    self.old_location = self.location
    x,  y  = self.location
    vx, vy = self.speed
    #fx, fy = self.simulation.get_force(self.location)
    vect_e = unit((0.5 - random.random(), 0.5 - random.random()))
    scal_e = random.random() * maximum
    ex, ey = (scal_e * vect_e[0], scal_e * vect_e[1])
    # update location
    self.location = (x + timestep * vx, y + timestep * vy)
    # update speed
    self.speed = (vx + timestep*(ex) - timestep*decay * vx*vx*vx, \
                  vy + timestep*(ey) - timestep*decay * vy*vy*vy,)
    if self.constant_speed:
      if self.speed == (0, 0):
        self.speed = unit( (random.random(), random.random()) )
      else:
        self.speed = unit(self.speed)

    # do reinforcement learning steps
    self.simulation.reinf_learn( self.location, self.old_location, (vx,vy), explore=True )
    


