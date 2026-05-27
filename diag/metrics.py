"""
metrics.py — small, dependency-free statistics helpers.

Everything DiagTool measures ends up as a stream of samples (latencies,
speeds, frame intervals). These helpers turn streams into summary stats and
keep running rate/jitter estimates without storing unbounded history.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Iterable


def summarize(samples: Iterable[float], unit: str = "") -> dict:
    """Return a stats dict for a list of numeric samples.

    Keys: n, mean, median, std, min, max, p05, p95, unit. Empty input yields
    an all-None dict (so callers/JSON stay uniform).
    """
    xs = sorted(float(x) for x in samples if x is not None and not _is_nan(x))
    n = len(xs)
    if n == 0:
        return {"n": 0, "mean": None, "median": None, "std": None,
                "min": None, "max": None, "p05": None, "p95": None,
                "unit": unit}
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / n
    return {
        "n": n,
        "mean": mean,
        "median": _percentile(xs, 50.0),
        "std": math.sqrt(var),
        "min": xs[0],
        "max": xs[-1],
        "p05": _percentile(xs, 5.0),
        "p95": _percentile(xs, 95.0),
        "unit": unit,
    }


def _is_nan(x) -> bool:
    try:
        return math.isnan(float(x))
    except (TypeError, ValueError):
        return True


def _percentile(sorted_xs: list[float], pct: float) -> float:
    if not sorted_xs:
        return float("nan")
    if len(sorted_xs) == 1:
        return sorted_xs[0]
    rank = (pct / 100.0) * (len(sorted_xs) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return sorted_xs[lo]
    frac = rank - lo
    return sorted_xs[lo] * (1 - frac) + sorted_xs[hi] * frac


def fmt_stat(stats: dict, scale: float = 1.0, digits: int = 2) -> str:
    """One-line human summary of a summarize() dict, e.g.
    'mean=12.3 med=11.8 std=2.1 min=9.0 max=21.4 (n=40) ms'."""
    if not stats or stats.get("n", 0) == 0:
        return "no samples"
    u = stats.get("unit", "")

    def s(key):
        v = stats.get(key)
        return "—" if v is None else f"{v * scale:.{digits}f}"

    return (f"mean={s('mean')} med={s('median')} std={s('std')} "
            f"min={s('min')} max={s('max')} (n={stats['n']})"
            + (f" {u}" if u else ""))


class RateMeter:
    """Tracks event rate (Hz) and inter-arrival interval stats over a window.

    Call `tick()` on every event. `rate_hz` and `interval_stats()` read the
    sliding window of recent timestamps.
    """

    def __init__(self, window: int = 240):
        self._ts: deque[float] = deque(maxlen=window)
        self._intervals: deque[float] = deque(maxlen=window)
        self.total = 0

    def tick(self, t: float | None = None) -> None:
        t = time.perf_counter() if t is None else t
        if self._ts:
            self._intervals.append(t - self._ts[-1])
        self._ts.append(t)
        self.total += 1

    @property
    def rate_hz(self) -> float:
        if len(self._ts) < 2:
            return 0.0
        span = self._ts[-1] - self._ts[0]
        return (len(self._ts) - 1) / span if span > 0 else 0.0

    @property
    def last_ts(self) -> float | None:
        return self._ts[-1] if self._ts else None

    def age(self, now: float | None = None) -> float | None:
        """Seconds since the most recent event, or None if never."""
        if not self._ts:
            return None
        now = time.perf_counter() if now is None else now
        return now - self._ts[-1]

    def interval_stats(self, unit: str = "ms") -> dict:
        scale = 1000.0 if unit == "ms" else 1.0
        return summarize([dt * scale for dt in self._intervals], unit=unit)


class Welford:
    """Online mean/variance (Welford's algorithm) — O(1) memory."""

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self._m2 = 0.0

    def add(self, x: float) -> None:
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self._m2 += d * (x - self.mean)

    @property
    def variance(self) -> float:
        return self._m2 / self.n if self.n > 1 else 0.0

    @property
    def std(self) -> float:
        return math.sqrt(self.variance)
