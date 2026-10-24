"""Driver-owned quiet release and recovery-only steering trajectory.

No CAN, parameters, sockets or I/O. Physical/safety limits remain downstream.
"""
import math

QUIET_SECONDS = .5


def clip(value, low, high):
  return min(high, max(low, value))


class WheelMotion:
  """Derive motion from EPAS angle; this car's STW rate is an invalid sentinel."""
  def __init__(self, dt=.01):
    self.dt = dt
    self.reset()

  def reset(self):
    self.angle = None
    self.age = self.rate = 0.
    self.samples = 0
    self.ready = False

  def update(self, angle):
    if not math.isfinite(angle):
      self.reset()
      return
    if self.angle is None:
      self.angle = angle
      return
    self.age += self.dt
    self.samples += 1
    if abs(angle-self.angle) >= .05:
      # Stationary repeats are fresh EPAS samples, not time available
      # for a later jump. Cap observation span at two 25 Hz frames.
      observed = (angle-self.angle)/min(self.age, .08)
      if abs(observed) > 720.:
        self.reset()
        self.angle = angle
        return
      self.rate += self.age/(.10+self.age)*(observed-self.rate)
      self.angle = angle
      self.age = 0.
    elif self.age >= .08:
      self.rate += self.dt/(.10+self.dt)*(0.-self.rate)
    self.ready = self.samples >= 10


class QuietHandover:
  def __init__(self, dt=.01):
    self.dt = dt
    self.pending = False
    self.quiet_ticks = 0

  def reset(self):
    self.pending = False
    self.quiet_ticks = 0

  def update(self, requested, healthy, pulling, quiet):
    if not requested:
      self.reset()
      return False
    if pulling or not healthy:
      self.pending = True
      self.quiet_ticks = 0
      return True
    if self.pending:
      self.quiet_ticks = self.quiet_ticks + 1 if quiet else 0
      if self.quiet_ticks * self.dt >= QUIET_SECONDS:
        self.pending = False
        self.quiet_ticks = 0
    return self.pending


class SteeringRecovery:
  """Remove the initial handback offset; do not filter subsequent path motion.

  A finite quintic blend removes the mismatch captured at takeover release.
  The normal downstream physical angle/rate/jerk limits remain authoritative.
  Those limits, rather than restarting a tracking filter, handle any clipping.
  """
  def __init__(self, dt=.02):
    self.dt = dt
    self.reset()

  def reset(self):
    self.active = False
    self.seeded = False
    self.elapsed = 0.
    self.x = self.v = self.a = 0.

  def seed(self, angle, rate, max_rate):
    self.active = self.seeded = True
    self.x = angle
    self.v = clip(rate, -max_rate, max_rate)
    self.a = 0.
    self.elapsed = 0.

  def update(self, target, max_rate):
    if not self.active:
      return target
    if self.seeded:
      self.seeded = False
      self.offset = self.x - target
      self.target_first = target
      self.initial_rate = self.v
      # Small bend corrections end quickly; large mismatches get a longer
      # blend instead of a large initial kick. This is not an inactive wait.
      self.duration = max(.35 + abs(self.offset)/150.,
                          1.875*abs(self.offset)/max_rate,
                          abs(self.initial_rate)/500.)
      return self.x
    if self.elapsed == 0.:
      target_rate = clip((target-self.target_first)/self.dt, -max_rate, max_rate)
      self.initial_rate -= target_rate
      self.duration = max(self.duration, abs(self.initial_rate)/500.)
    self.elapsed += self.dt
    q = min(1., self.elapsed/self.duration)
    h0 = 1. - 10.*q**3 + 15.*q**4 - 6.*q**5
    h1 = q - 6.*q**3 + 8.*q**4 - 3.*q**5
    self.x = target + self.offset*h0 + self.initial_rate*self.duration*h1
    if q >= 1.:
      self.active = False
      self.x = target
    return self.x

  def commit(self, applied):
    # The physical limiter owns the transmitted trajectory. It is not
    # bypassed at blend completion and no unsent state is accumulated here.
    if not math.isfinite(applied):
      self.reset()
