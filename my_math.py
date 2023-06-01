import math

def angvel(rpm):
  return rpm * 2 * math.pi / 60

def rpm(angvel):
  return angvel * 60 / (2 * math.pi)

def clamp(x, a, b):
  return max(min(x, b), a)

def lerp(x, min, max):
  t = (x - min[0]) / (max[0] - min[0])
  return min[1] + t * (max[1] - min[1])

class PointwiseLerp:
  points: list

  def __init__(self, points):
    self.points = points

  def get(self, x):
    n = len(self.points)
    if n == 0:
      return 0
    
    if n == 1 or x < self.points[0][0]:
      return self.points[0][1]
    
    for i in range(len(self.points)):
      pt = self.points[i]
      if pt[0] > x: # too late
        return lerp(x, self.points[i - 1], pt)
    
    return self.points[-1][1]
    
