#!/usr/bin/env python3
"""Tests for pv_monitor config parsing and the tunnel-mode no-hang guarantee.

Run:  .venv/bin/python test_pv_monitor.py
These need caproto but no IOC / no live network — the tunnel test points at a
dead host and asserts start() returns instantly and nothing raises.
"""

import json
import tempfile
import time
from pathlib import Path

from pv_monitor import load_pv_config, PVMonitor


def _write(tmp, obj):
    p = Path(tmp) / "cfg.json"
    p.write_text(json.dumps(obj) if not isinstance(obj, str) else obj)
    return p


def test_config_parsing():
    with tempfile.TemporaryDirectory() as tmp:
        # object form, no _epics -> native
        p = _write(tmp, {"a": "PV:A", "b": "PV:B"})
        pv_map, tun = load_pv_config(p)
        assert pv_map == {"a": "PV:A", "b": "PV:B"}, pv_map
        assert tun is None

        # _epics with host -> tunnel; _-keys excluded from pv_map
        p = _write(tmp, {"_epics": {"host": "localhost", "port": 15064}, "a": "PV:A"})
        pv_map, tun = load_pv_config(p)
        assert pv_map == {"a": "PV:A"}, pv_map
        assert tun == {"host": "localhost", "port": 15064}, tun

        # empty host -> native (so the example config is safe to copy)
        p = _write(tmp, {"_epics": {"host": "", "port": 15064}, "a": "PV:A"})
        pv_map, tun = load_pv_config(p)
        assert tun is None, tun

        # default port when omitted
        p = _write(tmp, {"_epics": {"host": "gw"}, "a": "PV:A"})
        _, tun = load_pv_config(p)
        assert tun == {"host": "gw", "port": 5064}, tun

        # list form -> names double as labels, native
        p = _write(tmp, ["PV:X", "PV:Y"])
        pv_map, tun = load_pv_config(p)
        assert pv_map == {"PV:X": "PV:X", "PV:Y": "PV:Y"}, pv_map
        assert tun is None

        # missing / empty / bad json
        assert load_pv_config(Path(tmp) / "nope.json") == ({}, None)
        assert load_pv_config(_write(tmp, "")) == ({}, None)
        assert load_pv_config(_write(tmp, "{not json")) == ({}, None)
    print("ok  test_config_parsing")


def test_tunnel_start_never_blocks_or_raises():
    # Port 1 on a non-routable TEST-NET address: connect will fail/backoff.
    pv_map = {"a": "PV:A", "b": "PV:B"}
    mon = PVMonitor(pv_map, tunnel_cfg={"host": "192.0.2.1", "port": 1})

    t0 = time.monotonic()
    mon.start()                 # must return immediately, never raise
    elapsed = time.monotonic() - t0
    assert elapsed < 0.5, f"start() blocked for {elapsed:.2f}s"

    snap = mon.snapshot()
    assert set(snap) == {"a", "b"}
    assert all(not v["connected"] for v in snap.values()), snap
    assert mon.connected_count() == 0
    assert mon.total_count() == 2

    t0 = time.monotonic()
    mon.stop()                  # must not hang
    assert time.monotonic() - t0 < 5.0
    print("ok  test_tunnel_start_never_blocks_or_raises")


def test_no_pvs_is_noop():
    mon = PVMonitor({}, tunnel_cfg={"host": "192.0.2.1", "port": 1})
    mon.start()
    assert mon.snapshot() == {}
    assert mon.total_count() == 0
    mon.stop()
    print("ok  test_no_pvs_is_noop")


if __name__ == "__main__":
    test_config_parsing()
    test_tunnel_start_never_blocks_or_raises()
    test_no_pvs_is_noop()
    print("\nall passed")
