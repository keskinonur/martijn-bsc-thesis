# simulation of RL techniques for ARDrone
# spring 2011
# author: martijn van der veen
# licence: GPLv3

from header import *

## math && linear algebra

def norm(vect):
  return math.sqrt(vect[0]*vect[0] + vect[1]*vect[1])

def unit(vect):
  vect_norm = norm(vect)
  if vect_norm == 0:
    return (0,0) # is scalar in GA
  else:
    return ( float(vect[0]) / vect_norm, float(vect[1]) / vect_norm )

def dot(vect1, vect2):
  return vect1[0] * vect2[0] + vect1[1] * vect2[1]

def add(vect1, vect2):
  return (vect1[0] + vect2[0], vect1[1] + vect2[1])

def diff(vect1, vect2):
  return (vect1[0] - vect2[0], vect1[1] - vect2[1])

def dist(vect1, vect2): # distance
  return math.sqrt(math.pow(vect1[0] - vect2[0], 2) + \
                   math.pow(vect1[1] - vect2[1], 2))

def rot(vect, angle):
  angle = math.radians(angle)
  return (math.cos(angle)*vect[0] - math.sin(angle)*vect[1], \
          math.sin(angle)*vect[0] + math.cos(angle)*vect[1])

# helper function to check for sign
# (returns true if path from a to b to c takes bocht to right)
def sign(a, b, c):
  return (c[0]*a[1] - c[0]*b[1] + c[1]*b[0] - c[1]*a[0] + b[1]*a[0] - b[0]*a[1]) > 0

