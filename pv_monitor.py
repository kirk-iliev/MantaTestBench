#!/usr/bin/env python3
"""
Timestamped EPICS Channel Access metadata capture (caproto, pure Python).

Reads a JSON map of {label: pv_name} at startup, opens persistent CA monitors,
and keeps a thread-safe cache of the latest value + EPICS source timestamp for
every PV. ``snapshot()`` returns that cache so it can be folded into the
per-shot sidecar at acquisition time.

Two connection backends sit behind one unchanged public interface, chosen by an
optional ``_epics`` block in the config:

  * Native mode (default, on the controls subnet): a caproto threading
    ``Context()`` with standard UDP discovery. This is the original behaviour
    and is byte-for-byte unchanged when no ``_epics.host`` is configured.
  * Tunnel mode (off-subnet / lab WiFi via ``ssh -L``): one raw-socket
    ``caproto.VirtualCircuit`` per configured forward, each to a fixed
    ``host:port``, bypassing the CA search phase entirely. Each PV is opened by
    name on every forward and auto-claims the one IOC that serves it (others
    fast-fail). Built for a per-IOC ``ssh -L`` forward (ideally to a CA gateway).
    See ``docs/EPICS_CONNECTIVITY.md``.

Designed to be entirely optional and failure-tolerant — nothing here can
raise into the acquisition path, and ``start()`` never blocks the caller:
  * No JSON / empty JSON          → no PVs, snapshot() returns {}.
  * caproto not installed         → PVMonitor is a silent no-op.
  * A PV that never connects      → recorded as {"connected": False}.
  * One bad PV among many         → the rest still subscribe normally.
  * A dead/missing SSH tunnel     → start() returns instantly; a background
                                    thread retries with backoff forever.

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
    import caproto as ca
    from caproto import field_types
    from caproto.threading.client import Context
    _CAPROTO_OK = True
except Exception:                       # caproto missing or import error
    _CAPROTO_OK = False


# Reconnect backoff (seconds) for tunnel mode, indexed by attempt then clamped.
_RETRY_DELAYS = (2.0, 5.0, 10.0, 30.0)


def load_pv_config(path):
    """Load PV config from JSON. Returns ``(pv_map, tunnel_cfg)``.

    ``pv_map`` is a ``{label: pv_name}`` dict. ``tunnel_cfg`` is a list of
    ``{"host": str, "port": int}`` forwards (one per ``ssh -L`` local port)
    when tunnel mode is requested, else ``None``.

    Accepts either an object ``{"q1_current": "PV:NAME", ...}`` or a bare list
    ``["PV:NAME", ...]`` (each PV name then doubles as its own label). In object
    form, any key beginning with ``_`` is reserved (not a PV); the optional
    ``_epics`` key configures tunnel mode::

        {
          "_epics": {"forwards": [{"host": "localhost", "port": 15064},
                                  {"host": "localhost", "port": 15065}]},
          "gun_phase_deg": "EGUN:PHASE:RBV"
        }

    Tunnel mode is requested by either ``_epics.forwards`` (a non-empty list of
    ``{"host","port"}`` entries) OR a non-empty ``_epics.host`` (the original
    single-forward back-compat form, normalized to a one-element list);
    ``forwards`` takes precedence when both are present. An empty/absent
    ``_epics`` block means native mode (the original behaviour). Missing,
    empty, or malformed files return ``({}, None)``.
    """
    path = Path(path)
    if not path.exists():
        return {}, None
    try:
        raw = json.loads(path.read_text())
    except Exception:
        return {}, None

    if isinstance(raw, list):
        return {str(v): str(v) for v in raw if v}, None

    if isinstance(raw, dict):
        tunnel_cfg = _parse_tunnel_cfg(raw.get("_epics"))
        pv_map = {
            str(k): str(v)
            for k, v in raw.items()
            if v and not str(k).startswith("_")
        }
        return pv_map, tunnel_cfg

    return {}, None


def _parse_tunnel_cfg(block):
    """Validate an ``_epics`` block into a list of forwards, or ``None``.

    Returns ``None`` (native mode) unless tunnel mode is requested. Tunnel mode
    is requested by either of, with ``forwards`` taking precedence:

      * ``forwards``: a non-empty list of ``{"host","port"}`` entries — one per
        ``ssh -L`` local port. Each ``host`` defaults to ``"localhost"`` and
        ``port`` to 5064. Non-dict entries are skipped.
      * ``host``: a non-empty string (the original single-forward form),
        normalized to a one-element list.

    Any block that yields no valid forward (incl. an empty/empty-host config)
    returns ``None``, so the example config stays safe to copy.
    """
    if not isinstance(block, dict):
        return None

    def _one(host, port):
        if not isinstance(host, str) or not host:
            return None
        try:
            port = int(port)
        except (TypeError, ValueError):
            port = 5064
        return {"host": host, "port": port}

    raw_forwards = block.get("forwards")
    if isinstance(raw_forwards, list):
        out = []
        for entry in raw_forwards:
            if not isinstance(entry, dict):
                continue
            fwd = _one(entry.get("host", "localhost"), entry.get("port", 5064))
            if fwd is not None:
                out.append(fwd)
        return out or None

    fwd = _one(block.get("host"), block.get("port", 5064))
    return [fwd] if fwd is not None else None


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


# ---------------------------------------------------------------------------
# Low-level socket helpers (tunnel mode only) — ported from PyBeamViewer's
# core/epics_layer.py (proven byte-level CA handshake), trimmed to the subset
# the monitor needs: connect, open channel, and drain incoming commands.
# ---------------------------------------------------------------------------

def _drain(sock, circuit, timeout=10.0, idle_timeout=0.05):
    """Read available bytes and feed them through the circuit, returning the
    parsed commands. Blocks at most ``timeout`` seconds. Raises ConnectionError
    if the peer closes the socket."""
    import select
    import time
    commands = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        wait = idle_timeout if commands else min(remaining, 0.5)
        ready = select.select([sock], [], [], wait)
        if not ready[0]:
            break
        data = sock.recv(65536)
        if not data:
            raise ConnectionError("Socket closed by peer")
        cmds, _ = circuit.recv(data)
        for c in cmds:
            circuit.process_command(c)
        commands.extend(cmds)
    return commands


def _connect_epics_socket(host, port, timeout=10.0):
    """Open a TCP socket to ``host:port`` and complete the CA handshake.
    Returns ``(sock, circuit)``. Blocking, with ``timeout``; intended to run on
    a background thread."""
    import select
    import socket
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.setblocking(False)
    circuit = ca.VirtualCircuit(our_role=ca.CLIENT, address=(host, port), priority=0)
    for msg in [
        ca.VersionRequest(version=13, priority=0),
        ca.HostNameRequest("localhost"),
        ca.ClientNameRequest("manta-test-bench"),
    ]:
        sock.sendall(b"".join(bytes(b) for b in circuit.send(msg)))

    if not select.select([sock], [], [], timeout)[0]:
        raise TimeoutError("No CA handshake response")
    data = sock.recv(4096)
    cmds, _ = circuit.recv(data)
    for c in cmds:
        circuit.process_command(c)
    return sock, circuit


def _open_channel_safe(sock, circuit, pv_name, timeout=5.0):
    """Create a channel by name on an existing circuit (no search phase).
    Returns the connected ``ClientChannel``, or ``None`` if it fails/times out
    — so one missing PV can't sink the rest."""
    import time
    try:
        cid = circuit.new_channel_id()
        chan = ca.ClientChannel(pv_name, circuit, cid=cid)
        for b in circuit.send(chan.create()):
            sock.sendall(bytes(b))
        deadline = time.monotonic() + timeout
        while chan.states[ca.CLIENT] is not ca.CONNECTED:
            if chan.states[ca.CLIENT] is ca.FAILED:
                return None
            if time.monotonic() > deadline:
                return None
            _drain(sock, circuit, timeout=0.1)
        return chan
    except Exception:
        return None


def _time_subscribe(chan):
    """Build a DBE_VALUE subscription request on ``chan`` using the TIME variant
    of its native type (so EventAddResponse metadata carries the IOC timestamp),
    falling back to an untimed subscription if promotion isn't possible."""
    try:
        time_type = field_types["time"][chan.native_data_type]
        return chan.subscribe(data_type=time_type, mask=ca.SubscriptionType.DBE_VALUE)
    except Exception:
        return chan.subscribe(mask=ca.SubscriptionType.DBE_VALUE)


class PVMonitor:
    """Persistent CA monitor set with a thread-safe latest-value cache.

    Cache entry per label: ``{"value": <py>, "timestamp": <float|None>, "connected": <bool>}``.

    ``tunnel_cfg`` selects the backend: ``None`` (native), a single
    ``{"host","port"}`` dict (back-compat), or a list of such dicts (one per
    ``ssh -L`` forward). Native mode runs when it is empty or caproto is absent.
    """

    def __init__(self, pv_map: dict, tunnel_cfg=None):
        self._pv_map = dict(pv_map)                 # label -> pv name
        self._forwards = self._normalize_forwards(tunnel_cfg)
        self._lock   = threading.Lock()
        self._cache  = {
            label: {"value": None, "timestamp": None, "connected": False}
            for label in self._pv_map
        }
        self._ctx  = None
        self._pvs  = []
        self._subs = []
        # tunnel-mode state
        self._stop_event = threading.Event()
        self._tunnel_threads = []
        self._tunnel_socks = set()      # live sockets, guarded by _lock
        self._claimed = set()           # labels owned by a forward, guarded by _lock

    @staticmethod
    def _normalize_forwards(tunnel_cfg):
        """Accept None (native), a single ``{host,port}`` dict (back-compat), or
        a list of such dicts, and return a list of forwards. Empty -> native."""
        if not tunnel_cfg:
            return []
        if isinstance(tunnel_cfg, dict):
            return [tunnel_cfg]
        return list(tunnel_cfg)

    def start(self):
        """Open the connection(s) and subscribe (timestamped) to every PV.

        Connection and reconnection happen in the background; an unreachable PV
        or forward simply never updates its cache entry. Never raises, and never
        blocks the caller — tunnel mode does zero network I/O here, it only spawns
        the background loops, so a dead SSH forward can't stall GUI startup.
        """
        if not _CAPROTO_OK or not self._pv_map:
            return
        if self._forwards:
            for fwd in self._forwards:
                t = threading.Thread(
                    target=self._run_one_forward, args=(fwd,),
                    name=f"pv-tunnel-{fwd['port']}", daemon=True)
                t.start()
                self._tunnel_threads.append(t)
        else:
            self._start_native()

    # ── native mode (original behaviour) ──────────────────────────────────

    def _start_native(self):
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

    # ── tunnel mode ───────────────────────────────────────────────────────

    def _run_one_forward(self, forward):
        """Background loop for ONE forward: connect a circuit to its host:port,
        open every not-yet-claimed PV by name (non-owning IOCs fast-fail), claim
        and subscribe the ones that connect, and feed EventAddResponse payloads
        into the cache. Reconnects with backoff on any failure. Honours stop."""
        host = forward["host"]
        port = forward["port"]
        attempt = 0

        while not self._stop_event.is_set():
            sock = None
            my_labels = []
            try:
                sock, circuit = _connect_epics_socket(host, port)
                with self._lock:
                    self._tunnel_socks.add(sock)
                attempt = 0

                # Open all FIRST (channel-open drains the socket; a live
                # subscription's EventAddResponse would be consumed and lost
                # there). Skip labels another forward already serves; fast-fail
                # for PVs this IOC doesn't own.
                channels = []
                for label, pv_name in self._pv_map.items():
                    if self._stop_event.is_set():
                        break
                    with self._lock:
                        if label in self._claimed:
                            continue
                    chan = _open_channel_safe(sock, circuit, pv_name, timeout=2.0)
                    if chan is None:
                        continue
                    with self._lock:
                        if label in self._claimed:   # lost a race; owner keeps it
                            continue
                        self._claimed.add(label)
                    channels.append((label, chan))
                    my_labels.append(label)

                subid_to_label = {}
                for label, chan in channels:
                    req = _time_subscribe(chan)
                    subid_to_label[req.subscriptionid] = label
                    for b in circuit.send(req):
                        sock.sendall(bytes(b))

                while not self._stop_event.is_set():
                    for cmd in _drain(sock, circuit, timeout=0.5):
                        if not isinstance(cmd, ca.EventAddResponse):
                            continue
                        label = subid_to_label.get(cmd.subscriptionid)
                        if label is None:
                            continue
                        try:
                            value = _to_native(cmd.data)
                            ts    = getattr(cmd.metadata, "timestamp", None)
                        except Exception:
                            continue
                        with self._lock:
                            self._cache[label] = {
                                "value": value, "timestamp": ts, "connected": True}

            except Exception:
                pass
            finally:
                with self._lock:
                    self._tunnel_socks.discard(sock)
                    for label in my_labels:          # release for re-claim
                        self._claimed.discard(label)
                        self._cache[label]["connected"] = False
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

            if self._stop_event.is_set():
                break
            delay = _RETRY_DELAYS[min(attempt, len(_RETRY_DELAYS) - 1)]
            attempt += 1
            self._stop_event.wait(delay)

    # ── public read API (unchanged) ───────────────────────────────────────

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
        self._stop_event.set()
        with self._lock:                    # unblock every drain loop's select()
            socks = list(self._tunnel_socks)
        for s in socks:
            try:
                s.close()
            except Exception:
                pass

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
        t.join(timeout=2.0)        # if still going, let the daemon die with us
        for tt in self._tunnel_threads:
            tt.join(timeout=2.0)
        self._subs = []
        self._pvs  = []
        self._ctx  = None
