#!/usr/bin/env python3
"""
Checks for VelocityEstimator, runnable on a desktop Python.

    python3 tools/test_velocity.py

The estimator drives the whole latency compensation, and a fault in it would
show up on stage as a background that swims against the screen rather than as
an obvious error. It is also pure arithmetic with no hardware dependency, so
it is worth checking directly rather than by eye through a video pipeline.

The class is extracted from code.py rather than copied, so these tests cannot
drift away from what actually runs on the board.

The wrap case matters most: supervisor.ticks_ms() rolls over every ~6.2 days,
so that path would otherwise only ever be exercised in production.
"""

import pathlib
import re
import sys
from array import array

HERE = pathlib.Path(__file__).resolve().parent
CODE = HERE.parent / "code.py"


def load_estimator():
    src = CODE.read_text()
    period = re.search(r"^_TICKS_PERIOD = .*$", src, re.M)
    cls = re.search(r"^class VelocityEstimator:.*?(?=^\S)", src, re.M | re.S)
    if not period or not cls:
        raise SystemExit("could not find VelocityEstimator in %s" % CODE)
    namespace = {"array": array}
    exec(period.group(0) + "\n" + cls.group(0), namespace)  # noqa: S102
    return namespace["VelocityEstimator"], namespace["_TICKS_PERIOD"]


def drive(Est, *, steps, step_counts, dt_ms, window_ms, start_ticks=0, period=None):
    """Feed evenly spaced samples and return the final velocity."""
    est = Est()
    ticks, pos, velocity = start_ticks, 0, 0.0
    for _ in range(steps):
        velocity = est.update(pos, ticks % period if period else ticks, window_ms)
        pos += step_counts
        ticks += dt_ms
    return velocity


def main():
    Est, period = load_estimator()
    failures = []

    def check(label, got, expect, tol):
        ok = abs(got - expect) <= tol
        print("%-22s %9.1f  (expect %s)  %s"
              % (label, got, expect, "ok" if ok else "FAIL"))
        if not ok:
            failures.append(label)

    # 10 counts every 5ms = 2000 counts/s.
    check("steady motion", drive(Est, steps=80, step_counts=10, dt_ms=5, window_ms=150),
          2000, 40)
    check("reverse motion", drive(Est, steps=80, step_counts=-10, dt_ms=5, window_ms=150),
          -2000, 40)
    check("stationary", drive(Est, steps=80, step_counts=0, dt_ms=5, window_ms=150),
          0, 0)
    # A short window must still measure the same speed, just over less history.
    check("short window", drive(Est, steps=60, step_counts=10, dt_ms=5, window_ms=50),
          2000, 60)
    # Straddle the ticks_ms rollover.
    check("across ticks wrap",
          drive(Est, steps=60, step_counts=10, dt_ms=5, window_ms=150,
                start_ticks=period - 40, period=period),
          2000, 40)

    # A single sample cannot imply a speed.
    est = Est()
    check("first sample", est.update(1234, 500, 150), 0, 0)

    print()
    if failures:
        print("FAILED: %s" % ", ".join(failures))
        return 1
    print("all velocity estimator checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
