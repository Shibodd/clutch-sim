from dataclasses import dataclass
import math
import numpy as np
import matplotlib.pyplot as plt
import phys
import time
import pysinewave

def angvel(rpm):
  return rpm * 2 * math.pi / 60

def rpm(angvel):
  return angvel * 60 / (2 * math.pi)

# CAR_WHEEL_RADIUS = 0.125 # m
# CAR_MASS = 300 # kg
# GEAR_RATIO = 0.11

CAR_INERTIA = 0.07 # GEAR_RATIO * CAR_MASS * (CAR_WHEEL_RADIUS ** 2)

ENGINE_IDLE_ANGVEL = angvel(2500)
ENGINE_IDLE_TORQUE = 9.35
ENGINE_PEAK_TORQUE = 68 # Nm
ENGINE_PEAK_TORQUE_ANGVEL = angvel(11500)
ENGINE_INERTIA = 0.02 # out of my ass
ENGINE_FRICTION_TORQUE = 0

ENGINE_LC_ANGVEL = angvel(5000)

ENGINE_OFF_ANGVEL = angvel(2000)

ENGINE_LOWRANGE_TORQUE_M = (ENGINE_IDLE_TORQUE - 0) / (ENGINE_IDLE_ANGVEL - ENGINE_OFF_ANGVEL)
ENGINE_OKRANGE_TORQUE_M = (ENGINE_PEAK_TORQUE - ENGINE_IDLE_TORQUE) / (ENGINE_PEAK_TORQUE_ANGVEL - ENGINE_IDLE_ANGVEL)

CLUTCH_STATIC_MAX_TORQUE = ENGINE_PEAK_TORQUE * 2.5 # 1.8 # out of my ass
CLUTCH_KINETIC_MAX_TORQUE = CLUTCH_STATIC_MAX_TORQUE * 0.8 # out of my ass

CLUTCH_ENGAGE_POSITION = 0.2
CLUTCH_POSITION_M = 1 / (1 - CLUTCH_ENGAGE_POSITION)

def engine_curve(angvel):
  if angvel < ENGINE_OFF_ANGVEL:
    return 0
  
  elif angvel < ENGINE_IDLE_ANGVEL:
    return (ENGINE_LOWRANGE_TORQUE_M * (angvel - ENGINE_OFF_ANGVEL))
  
  elif angvel < ENGINE_PEAK_TORQUE_ANGVEL:
    return ENGINE_OKRANGE_TORQUE_M * (angvel - ENGINE_IDLE_ANGVEL) + ENGINE_IDLE_TORQUE
  
  else:
    return 0

def engine_torque(throttle, angvel, LC):
  if LC and angvel > ENGINE_LC_ANGVEL:
    return 0
  return engine_curve(angvel) * throttle - ENGINE_FRICTION_TORQUE

def clamp(x, a, b):
  return max(min(x, b), a)

def clutch_pressure(pos):
  return clamp((pos - CLUTCH_ENGAGE_POSITION) * CLUTCH_POSITION_M, 0, 1)



# INPUTS
def control_clutch_position(t):
  PRE_A_DURATION = 0.1
  HOLD_DURATION = 0.5
  POST_B_DURATION = 0.1

  HOLD_A = 0.2
  HOLD_B = 0.3

  return clamp(ans, 0, 1)

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

sw = pysinewave.SineWave(pitch_per_second=50000)

sw.set_frequency(0)
sw.play()

for t in x:
  clutch.i_input_torque = engine_torque(control_throttle(t), clutch.c_input_shaft.angular_velocity, True)

  clt = clutch_pressure(control_clutch_position(t))
  clutch.i_kinetic_maxtorque = CLUTCH_KINETIC_MAX_TORQUE * clt
  clutch.i_static_maxtorque = CLUTCH_STATIC_MAX_TORQUE * clt

  y_in.append(rpm(clutch.c_input_shaft.angular_velocity))
  y_out.append(rpm(clutch.c_output_shaft.angular_velocity))
  y_clt.append(clt * 10000)
  clutch.tick(INTERVAL)

  sw.set_frequency(4 * rpm(clutch.c_input_shaft.angular_velocity) / 60)
  time.sleep(INTERVAL)

sw.stop()

plt.plot(x, y_in)
plt.plot(x, y_out)
plt.plot(x, y_clt)
plt.show()