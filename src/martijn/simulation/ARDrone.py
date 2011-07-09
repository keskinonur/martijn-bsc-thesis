# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

from header import *

class ARDrone:

  def __init__(self, location, simulation, color=(50, 255, 50)):
    self.color = color
    self.theta = 0
    self.speed = (0, 0)
    self.location = location
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


  def move(self, timestep, decay=0.0005):
    self.old_location = self.location
    x,  y  = self.location
    vx, vy = self.speed
    fx, fy = self.simulation.get_force(self.location)
    # update location
    self.location = (x + timestep * vx, y + timestep * vy)
    # TODO: make more realistic decay
    self.speed = (vx + timestep*fx - timestep*decay * vx*vx*vx, \
                  vy + timestep*fy - timestep*decay * vy*vy*vy)
    # for fun: set angle
    self.theta = math.acos(dot(unit(self.speed), (1, 0)))
    #print self.speed
    #print "with force " + str((fx, fy))

    # do reinforcement learning steps
    self.simulation.reinf_learn( (x,y), (vx,vy) )
    # check for state transition
    self.simulation.check_for_state_transition(self.location, self.old_location)
    
  def explore(self, timestep, maximum):
    self.old_location = self.location
    x,  y  = self.location
    vx, vy = self.speed
    fx, fy = self.simulation.get_force(self.location)
    vect_e = unit((0.5 - random.random(), 0.5 - random.random()))
    scal_e = random.random() * maximum
    ex, ey = (scal_e * vect_e[0], scal_e * vect_e[1])
    # update location
    self.location = (x + timestep * vx, y + timestep * vy)
    self.speed = (vx + timestep*(fx + ex), vy + timestep*(fy + ey))

    # do reinforcement learning steps
    self.simulation.reinf_learn( (x,y), (vx,vy) )
    # check for state transition
    self.simulation.check_for_state_transition(self.location, self.old_location)
    


