#!/usr/bin/env python3
"""
Timestamped EPICS Channel Access metadata capture (caproto, pure Python).

Reads a JSON map of {label: pv_name} at startup, opens persistent CA monitors,
and keeps a thread-safe cache of the latest value + EPICS source timestamp for
every PV. ``snapshot()`` returns that cache so it can be folded into the
per-shot sidecar at acquisition time.

Designed to be entirely optional and failure-tolerant — nothing here can
raise into the acquisition path:
  * No JSON / empty JSON          → no PVs, snapshot() returns {}.
  * caproto not installed         → PVMonitor is a silent no-op.
  * A PV that never connects      → recorded as {"connected": False}.
  * One bad PV among many         → the rest still subscribe normally.

Timestamps are the EPICS *source* timestamps (set by the IOC), captured via
DBR_TIME monitors. They are stored alongside each value so downstream analysis
can correlate a readback with a given shot. Note the IOC clock, the camera
clock, and the laptop clock are independent unless network-time-synced, so do
the final per-shot matching in post using these timestamps rather than assuming
they share an origin.
"""

import json
import threading
from pathlib import Path

try:
    from caproto.threading.client import Context
    _CAPROTO_OK = True
except Exception:                       # caproto missing or import error
    _CAPROTO_OK = False


def load_pv_config(path) -> dict:
    """Load a {label: pv_name} map from JSON. Missing/empty/bad file → {}.

    Accepts either an object ``{"q1_current": "PV:NAME", ...}`` or a bare list
    ``["PV:NAME", ...]`` (each PV name then doubles as its own label).
    """
    path = Path(path)
    if not path.exists():
        return {}
    try:
        raw = json.loads(path.read_text())
    except Exception:
        return {}
    if isinstance(raw, dict):
        return {str(k): str(v) for k, v in raw.items() if v}
    if isinstance(raw, list):
        return {str(v): str(v) for v in raw if v}
    return {}


def _to_native(data):
    """Coerce a caproto response payload into a JSON/text-friendly Python value."""
    v = data
    try:
        import numpy as np
        if isinstance(v, np.ndarray):
            v = v.reshape(-1)[0].item() if v.size == 1 else v.tolist()
    except Exception:
        pass
    if isinstance(v, bytes):
        return v.decode(errors="replace")
    if isinstance(v, (list, tuple)):
        return [x.decode(errors="replace") if isinstance(x, bytes) else x for x in v]
    return v


class PVMonitor:
    """Persistent CA monitor set with a thread-safe latest-value cache.

    Cache entry per label: ``{"value": <py>, "timestamp": <float|None>, "connected": <bool>}``.
    """

    def __init__(self, pv_map: dict):
        self._pv_map = dict(pv_map)                 # label -> pv name
        self._lock   = threading.Lock()
        self._cache  = {
            label: {"value": None, "timestamp": None, "connected": False}
            for label in self._pv_map
        }
        self._ctx  = None
        self._pvs  = []
        self._subs = []

    def start(self):
        """Open the CA context and subscribe (timestamped) to every PV.

        Connection and reconnection happen in the background; an unreachable PV
        simply never updates its cache entry. Never raises.
        """
        if not _CAPROTO_OK or not self._pv_map:
            return
        try:
            self._ctx = Context()
            labels = list(self._pv_map.keys())
            names  = [self._pv_map[label] for label in labels]
            self._pvs = self._ctx.get_pvs(*names)   # lazy; connects in background
            for label, pv in zip(labels, self._pvs):
                try:
                    sub = pv.subscribe(data_type="time")   # DBR_TIME → carries timestamp
                except Exception:
                    try:
                        sub = pv.subscribe()               # fall back to untimed
                    except Exception:
                        continue                           # one bad PV can't sink the rest
                sub.add_callback(self._make_cb(label))
                self._subs.append(sub)
        except Exception:
            self._ctx = None

    def _make_cb(self, label):
        def _cb(sub, response):
            try:
                value = _to_native(response.data)
                ts    = getattr(response.metadata, "timestamp", None)
            except Exception:
                return
            with self._lock:
                self._cache[label] = {"value": value, "timestamp": ts, "connected": True}
        return _cb

    def snapshot(self) -> dict:
        """Return a copy of the cache: label -> {value, timestamp, connected}."""
        with self._lock:
            return {k: dict(v) for k, v in self._cache.items()}

    def connected_count(self) -> int:
        with self._lock:
            return sum(1 for v in self._cache.values() if v["connected"])

    def total_count(self) -> int:
        return len(self._pv_map)

    def stop(self):
        """Best-effort teardown, watchdog-bounded so it can never hang the caller.

        caproto's context teardown can occasionally block on network sockets;
        since this is called from the GUI close path, we run it on a daemon
        thread and give up after a short timeout rather than freeze the app.
        """
        def _teardown():
            for sub in self._subs:
                try:
                    sub.clear()
                except Exception:
                    pass
            if self._ctx is not None:
                try:
                    self._ctx.disconnect()
                except Exception:
                    pass

        t = threading.Thread(target=_teardown, daemon=True)
        t.start()
        t.join(timeout=2.0)        # if it's still going, let the daemon thread die with us
        self._subs = []
        self._pvs  = []
        self._ctx  = None
