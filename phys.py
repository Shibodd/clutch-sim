from dataclasses import dataclass
import math

@dataclass
class RotationalInertia:
  moment: float
  angular_velocity: float

class Clutch:
  i_input_torque: float
  i_kinetic_maxtorque: float
  i_static_maxtorque: float

  p_slip_tol: float

  c_input_shaft: RotationalInertia
  c_output_shaft: RotationalInertia
  
  __state: int
  STATE_LOCKED = 0
  STATE_UNLOCKED = 1

  def __init__(self):
    self.__state = Clutch.STATE_UNLOCKED

  def __state_transition(self):
    should_unlock = abs(self.i_input_torque) > self.i_static_maxtorque

    if self.__state == Clutch.STATE_LOCKED:
      if should_unlock:
        self.__state = Clutch.STATE_UNLOCKED

    else: # unlocked
      if not should_unlock: # if the input torque is in the rated range
        slip = self.c_input_shaft.angular_velocity - self.c_output_shaft.angular_velocity
        if slip <= self.p_slip_tol:
          self.__state = Clutch.STATE_LOCKED

  def tick(self, dt):
    self.__state_transition()

    if self.__state == Clutch.STATE_LOCKED:
      angaccel = self.i_input_torque / (self.c_input_shaft.moment + self.c_output_shaft.moment)
      angvel = self.c_input_shaft.angular_velocity + dt * angaccel

      self.c_input_shaft.angular_velocity = angvel
      self.c_output_shaft.angular_velocity = angvel
    
    else:
      slip = self.c_input_shaft.angular_velocity - self.c_output_shaft.angular_velocity
      torque_out = math.copysign(self.i_kinetic_maxtorque, slip)

      self.c_input_shaft.angular_velocity += dt * ((self.i_input_torque - torque_out) / self.c_input_shaft.moment)
      self.c_output_shaft.angular_velocity += dt * (torque_out / self.c_output_shaft.moment)