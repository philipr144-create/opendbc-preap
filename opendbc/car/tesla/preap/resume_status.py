"""Read-only planner bridge for the authoritative carstate brake/resume FSM."""
import json
import os
import time
import uuid


class ResumeStatus:
  def __init__(self, path='/dev/shm/nap_brake_resume.json'):
    self.path = path
    self.session = uuid.uuid4().hex
    self.previous = None
    self.last_write = -1.0

  def publish(self, engagement, brake, healthy, can_engage, enabled, now=None):
    now = time.monotonic() if now is None else now
    state = dict(session=self.session, sequence=engagement.resume_sequence,
                 cruise=bool(engagement.cruiseEnabled and can_engage),
                 paused=bool(engagement.brake_paused_long), brake=bool(brake),
                 healthy=bool(healthy), enabled=bool(enabled))
    if state == self.previous and now-self.last_write < .1:
      return
    temp = self.path + '.tmp'
    try:
      with open(temp, 'w') as stream:
        json.dump(dict(state, received_mono=now, expires_mono=now+.5), stream, allow_nan=False)
      os.replace(temp, self.path)
      self.previous, self.last_write = state, now
    except (OSError, ValueError):
      # Missing/stale status cannot grant map authority or authorize resume.
      try:
        os.unlink(temp)
      except OSError:
        pass
