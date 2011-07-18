# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

from header import *

# width, height: window size
# coarse: size of one 'block' (size between vectors)
# spawn: spawn location
# speed: FPS, stepsize: nr of coarses per round
def fig8_discrete(width, height, coarse, spawn, history, exploration, \
                  constant_speed, speed=1000, stepsize=0.5):
  # open window
  sim = Simulation(width, height, spawn, coarse=coarse, history=history) 
  sim.constant_speed = constant_speed
  center = ( int(width / 2), int(height / 2) )
  # make border
  sim.add_object('wall', 'rect',   4, (255, 50, 0), (  3,   3), (width-9, height-9))
  # make pylons
  sim.add_object('pylon_left',  'circle', 0, (255, 50, 0), (1*int(center[0]/2), center[1]), int(center[1]/40))
  sim.add_object('pylon_right', 'circle', 0, (255, 50, 0), (3*int(center[0]/2), center[1]), int(center[1]/40))
  # add stages
  sim.add_stage('right_to_left')
  sim.add_stage('left_to_right')
  sim.add_particles_uniform('all', coarse)
  # add stage transitions
  trans1 = ('line', (5, center[1]), (1*int(center[0]/2), center[1]))
  trans2 = ('line', (width-5, center[1]), (3*int(center[0]/2), center[1]))
  sim.add_stage_transition('right_to_left', 'left_to_right', trans1)
  sim.add_stage_transition('left_to_right', 'right_to_left', trans2)
  
  # add forcefields for objects
  #                         (  obj,   r, pwr)
  sim.add_object_forcefields('all', 150, 20 )
  
  #                         ( stage,   object name,   r,pwr, ang)
  sim.add_rounded_forcefield('all' , 'pylon_left' , 300, 30,  90)
  sim.add_rounded_forcefield('all' , 'pylon_right', 300, 30, -90)
  
  # draw
  while True:
    try:
      sim.show()
      #sim.ardrone.theta += math.radians(5)
      if random.random() < exploration:
        sim.ardrone.explore(stepsize, 2)
      else:
        sim.ardrone.move(stepsize)
      time.sleep(1/speed)
    except KeyboardInterrupt:
      print "\n\n    Stopped simulation program!"
      exit()
    except:
      print "Unexpected error:", sys.exc_info()
      pygame.display.flip()
      while True:
        pass

def usage(name):
  print "Usage: " + name + " [arguments]"
  print " arguments:"
  print " -w <int> <int> window size"
  print " -s <int>: speed in FPS"
  print " -t <float>: size of timestep per round"
  print " -p <int> <int>: spawn location"
  print " -c <int>: coarse (# pixels per coarse)"
  print " -o <int>: use contant speed (1 or 0)"
  exit()

if __name__ == "__main__":
  print unit( (1, 2) )
  # default settings
  width  = 800
  height = 600
  speed = 10000
  stepsize = 1
  spawn = (int(width/2), int(height/4))
  coarse = 20         # pixels per coarse (for discrete simulations)
  history = 2        # reinf learn: number of vectors to update
  exploration = 0.2   # reinf learn: chance for exploration move
  constant_speed = True

  # parse command line args
  i = 1
  while i < len(sys.argv):
    arg = sys.argv[i]
    # window size
    if arg == "-w":
      i += 2
      width, height = int(sys.argv[i-1]), int(sys.argv[i])
    # speed in FPS
    elif arg == "-s":
      i += 1
      speed = float(sys.argv[i])
    # stepsize / timestep size per round
    elif arg == "-t":
      i += 1
      stepsize = float(sys.argv[i])
    # spawn location
    elif arg == "-p":
      i += 2
      spawn = (float(sys.argv[i-1]), float(sys.argv[i]))
    # coarse size
    elif arg == "-c":
      i += 1
      coarse = int(sys.argv[i])
    elif arg == "-o":
      i += 1
      constant_speed = bool(sys.argv[i])
    else:
      print arg
      usage(sys.argv[0])
    i += 1

  # play discrete simulation for figure-8's
  fig8_discrete(width, height, coarse, spawn, \
           history, exploration, constant_speed, \
           speed, stepsize)


