#!/usr/bin/env python3
"""Tests for epics_control.PVWriter — tunnel put never hangs and raises
WriteError when no forward serves the PV. No IOC / no live network: the
tunnel test points at a refused port and asserts a fast WriteError."""

import time
from epics_control import PVWriter, WriteError


def test_tunnel_put_unreachable_raises_fast():
    # localhost:1 refuses immediately -> no forward serves the PV.
    w = PVWriter(forwards=[{"host": "localhost", "port": 1}])
    t0 = time.monotonic()
    raised = False
    try:
        w.put("X:Y:Setpoint", 1.0, timeout=0.5)
    except WriteError:
        raised = True
    elapsed = time.monotonic() - t0
    w.close()
    assert raised, "expected WriteError on unreachable forward"
    assert elapsed < 3.0, f"put hung for {elapsed:.2f}s"
    print("ok  test_tunnel_put_unreachable_raises_fast")


def test_native_mode_selected_without_forwards():
    # No forwards -> native mode object constructs; we don't connect (no IOC).
    w = PVWriter(forwards=None)
    assert w._forwards is None
    w.close()
    print("ok  test_native_mode_selected_without_forwards")


if __name__ == "__main__":
    test_tunnel_put_unreachable_raises_fast()
    test_native_mode_selected_without_forwards()
    print("\nall passed")
