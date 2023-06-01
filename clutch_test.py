from dataclasses import dataclass
from my_math import *
import numpy as np
import matplotlib.pyplot as plt
import phys
import time


SOUND = False
if SOUND:
  import pysinewave


CAR_INERTIA = 0.07 # out of my ass
ENGINE_INERTIA = 0.02 # out of my ass
ENGINE_FRICTION_TORQUE = 0 # out of my ass
ENGINE_PEAK_TORQUE = 68
ENGINE_LC_ANGVEL = angvel(5000)

CLUTCH_STATIC_MAX_TORQUE = ENGINE_PEAK_TORQUE * 1.8 # out of my ass
CLUTCH_KINETIC_MAX_TORQUE = CLUTCH_STATIC_MAX_TORQUE * 0.8 # out of my ass

# Proportion of torque transmitted by the clutch in [0, 1] 
# as a function of clutch position in [0, 1]
CLUTCH_CURVE = PointwiseLerp([
  (0.2, 0), # Clutch engagement
  (1, 1)
])

# Torque in function of angular velocity
ENGINE_CURVE = PointwiseLerp([
  (angvel(2000), 0), 
  (angvel(2500), 9.35),
  (angvel(12000), ENGINE_PEAK_TORQUE),
  (angvel(12000.1), 0)
])

def engine_torque(throttle, angvel, LC):
  if LC and angvel > ENGINE_LC_ANGVEL:
    return 0
  return ENGINE_CURVE.get(angvel) * throttle - ENGINE_FRICTION_TORQUE

PRE_A_DUR = 0.1
HOLD_DUR = 1
POST_B_DUR = 0.1
CLT_A = 0.35
CLT_B = 0.5

CLT_POS_CURVE = PointwiseLerp([
  (0, 0),
  (PRE_A_DUR, CLT_A),
  (PRE_A_DUR + HOLD_DUR, CLT_B),
  (PRE_A_DUR + HOLD_DUR + POST_B_DUR, 1)
])

# INPUTS
def control_clutch_position(t):
  return CLT_POS_CURVE.get(t)

def control_throttle(t):
  return 1

clutch = phys.Clutch()

clutch.p_slip_tol = 0.05
clutch.c_input_shaft = phys.RotationalInertia(ENGINE_INERTIA, ENGINE_LC_ANGVEL)
clutch.c_output_shaft = phys.RotationalInertia(CAR_INERTIA, angvel(0))

TIME = 2
INTERVAL = 1 / 1000

x = np.arange(0, TIME, INTERVAL)
y_in = []
y_out = []
y_clt = []

if SOUND:
  sw = pysinewave.SineWave(pitch_per_second=50000)
  sw.set_frequency(0)
  sw.play()

for t in x:
  clutch.i_input_torque = engine_torque(control_throttle(t), clutch.c_input_shaft.angular_velocity, True)

  clt = CLUTCH_CURVE.get(control_clutch_position(t))
  clutch.i_kinetic_maxtorque = CLUTCH_KINETIC_MAX_TORQUE * clt
  clutch.i_static_maxtorque = CLUTCH_STATIC_MAX_TORQUE * clt

  y_in.append(rpm(clutch.c_input_shaft.angular_velocity))
  y_out.append(rpm(clutch.c_output_shaft.angular_velocity))
  y_clt.append(clt * 10000)
  clutch.tick(INTERVAL)

  if SOUND:
    sw.set_frequency(4 * rpm(clutch.c_input_shaft.angular_velocity) / 60)
    time.sleep(INTERVAL)

if SOUND:
  sw.stop()

plt.plot(x, y_in)
plt.plot(x, y_out)
plt.plot(x, y_clt)
plt.show()