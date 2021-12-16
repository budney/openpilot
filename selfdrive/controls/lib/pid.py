import numpy as np
from numbers import Number

from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class PIDController():
  def __init__(self, k_p, k_i, k_d, k_f=0., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.4, convert=None):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_d = k_d  # derivative gain
    self.k_f = k_f  # feedforward gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit
    self.sat_limit = sat_limit / DT_CTRL

    self.convert = convert

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])

  def _check_saturation(self, control, check_saturation, saturation):
    thresh = 0.05
    contr = control

    if (abs(saturation) > thresh) and check_saturation:
      self.sat_count += 1
    else:
      self.sat_count -= 1

    self.sat_count = clip(self.sat_count, 0.0, self.sat_limit * 1.25)

    contr *= 1.0 + thresh

    self.saturated = self.saturated or (contr <= self.neg_limit) or (contr >= self.pos_limit)
    self.saturated = self.saturated and (self.sat_count > self.sat_limit)

  def reset(self):
    self.e0, self.e1, self.e2 = 0.0, 0.0, 0.0
    self.bf1, self.bf2 = 0.0, 0.0

    self.p, self.p1, self.p2 = 0.0, 0.0, 0.0
    self.i, self.i1, self.i2 = 0.0, 0.0, 0.0
    self.d, self.d1, self.d2 = 0.0, 0.0, 0.0
    self.f = 0.0

    self.sat_count = 0.0
    self.saturated = False

  def update(self, setpoint, measurement, last_output, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    k_bf = 1.0
    _N = 6
    _Ts = DT_CTRL
    
    Kp, Ki, Kd = self.k_p, self.k_i, self.k_d
    a0, a1, a2 = (1 + _N*_Ts), -(2 + _N*_Ts), 1
    
    self.ku1, self.ku2 = a1/a0, a2/a0

    self.e2, self.e1 = self.e1, self.e0    
    self.e0 = float(apply_deadzone(setpoint - measurement, deadzone))

    #back-feed / back-calculate integrator anti-windup
    self.bf2 = self.bf1
    self.bf1 = self.p + self.i + self.d + self.f - last_output

    self.p2, self.p1 = self.p1, self.p
    self.i2, self.i1 = self.i1, self.i
    self.d2, self.d1 = self.d1, self.d

    self.p = (Kp*(    a0*self.e0 + a1*self.e1 + self.e2) / a0) -self.ku1*self.p1 - self.ku2*self.p2
    self.i = (Ki*_Ts*(a0*self.e0 -    self.e1)           / a0) -self.ku1*self.i1 - self.ku2*self.i2 - (k_bf*_Ts*(a0*self.bf1 - self.bf2)/ a0)
    self.d = (Kd*_N*(    self.e0 -  2*self.e1 + self.e2) / a0) -self.ku1*self.d1 - self.ku2*self.d2
    self.f = self.k_f*feedforward

    saturation = (self.p + self.i + self.d + self.f)
    saturation -= clip(saturation, self.neg_limit, self.pos_limit)
    self._check_saturation(last_output, check_saturation, saturation)

    self.f = clip(self.f, self.neg_limit, self.pos_limit)
    control = self.p + self.i + self.d + self.f
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    return clip(control, self.neg_limit, self.pos_limit)

class PIController():
  def __init__(self, k_p, k_i, k_f=0., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = sat_limit

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control <= self.neg_limit) or (control >= self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p
    self.f = feedforward * self.k_f

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i
    self.saturated = self._check_saturation(control, check_saturation, error)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
