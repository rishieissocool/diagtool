"""Thread-safe per-robot store for the latest onboard observation."""

import threading
import time


class OnboardObservationStore:
    """Keyed by (is_yellow, robot_id). Writers call `put(obs)`; readers
    call `get(is_yellow, robot_id)` to retrieve the most recent reading.

    Freshness is the receiver's responsibility: `OnboardObservation.recv_ts`
    is a wall-clock timestamp that callers can compare against `time.time()`
    to decide whether a reading is still usable.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._by_key = {}   # (is_yellow, robot_id) -> OnboardObservation

    def put(self, obs):
        if obs is None or obs.robot_id < 0:
            return
        key = (bool(obs.is_yellow), int(obs.robot_id))
        with self._lock:
            self._by_key[key] = obs

    def get(self, is_yellow, robot_id, max_age=None):
        """Return the latest observation for one robot or None.

        If `max_age` is given (seconds), return None when the reading is
        older than that.
        """
        key = (bool(is_yellow), int(robot_id))
        with self._lock:
            obs = self._by_key.get(key)
        if obs is None:
            return None
        if max_age is not None and (time.time() - obs.recv_ts) > max_age:
            return None
        return obs

    def snapshot(self):
        """Copy of the whole map, for debugging / UI."""
        with self._lock:
            return dict(self._by_key)

    def clear(self):
        with self._lock:
            self._by_key.clear()
