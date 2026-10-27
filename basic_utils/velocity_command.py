"""Timestamped planar velocity command, independent of ROS and Habitat."""
import math
import time


class VelocityCommand:
    def __init__(self, timeout=.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Velocity command timeout must be positive and finite')
        self.timeout = timeout
        self.stop()

    def update(self, linear, angular, now=None):
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.stop()
            return
        self.linear, self.angular = float(linear), float(angular)
        self.received_at = time.monotonic() if now is None else now

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        if self.received_at is None or now - self.received_at > self.timeout:
            return 0., 0.
        return self.linear, self.angular

    def stop(self):
        self.linear, self.angular = 0., 0.
        self.received_at = None
