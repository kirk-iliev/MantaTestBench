#!/usr/bin/env python3
"""Synchronous EPICS Channel Access writes (caproto), native or tunnel mode.

Mirror of pv_monitor.py's two backends, for the write direction:

  * Native mode (forwards is None): a caproto threading Context + PV.write.
  * Tunnel mode (forwards is a list of {"host","port"}): a raw-socket
    VirtualCircuit per forward, opening the target channel by name on
    whichever forward's IOC serves it (others fast-fail), then a
    write-with-notify awaited to its ack. Reuses pv_monitor's socket helpers.

put() is synchronous and bounded by ``timeout``; it raises WriteError rather
than hanging. The scan layer treats any WriteError as a fail-fast fault. The
post-write readback-settle check in the engine is the real confirmation the
setpoint took effect, so an acked-but-ineffective write still surfaces as a
settle timeout.
"""

import time

try:
    import caproto as ca
    from caproto.threading.client import Context
    _CAPROTO_OK = True
except Exception:
    _CAPROTO_OK = False

from pv_monitor import _connect_epics_socket, _open_channel_safe, _drain


class WriteError(Exception):
    pass


class PVWriter:
    """Synchronous EPICS put. ``forwards``: list of {"host","port"} (tunnel)
    or None (native)."""

    def __init__(self, forwards=None):
        self._forwards = list(forwards) if forwards else None
        self._ctx = None                 # native Context
        self._native_pvs = {}            # pv_name -> PV
        self._conns = []                 # tunnel: [(sock, circuit)]
        self._chan_cache = {}            # pv_name -> (sock, circuit, chan)

    def put(self, pv_name, value, timeout=5.0):
        if not _CAPROTO_OK:
            raise WriteError("caproto not available")
        if self._forwards is None:
            self._put_native(pv_name, value, timeout)
        else:
            self._put_tunnel(pv_name, value, timeout)

    # ── native ────────────────────────────────────────────────────────────
    def _put_native(self, pv_name, value, timeout):
        try:
            if self._ctx is None:
                self._ctx = Context()
            pv = self._native_pvs.get(pv_name)
            if pv is None:
                (pv,) = self._ctx.get_pvs(pv_name)
                self._native_pvs[pv_name] = pv
            pv.wait_for_connection(timeout=timeout)
            pv.write(value, wait=True, timeout=timeout)
        except Exception as e:
            raise WriteError(f"native put {pv_name}={value} failed: {e}") from e

    # ── tunnel ────────────────────────────────────────────────────────────
    def _ensure_channel(self, pv_name, timeout):
        cached = self._chan_cache.get(pv_name)
        if cached is not None:
            return cached
        last_err = None
        for fwd in self._forwards:
            sock = None
            try:
                sock, circuit = _connect_epics_socket(
                    fwd["host"], fwd["port"], timeout=timeout)
            except Exception as e:
                last_err = e
                continue
            chan = _open_channel_safe(sock, circuit, pv_name, timeout=timeout)
            if chan is not None:
                self._conns.append((sock, circuit))
                self._chan_cache[pv_name] = (sock, circuit, chan)
                return self._chan_cache[pv_name]
            try:
                sock.close()
            except Exception:
                pass
        raise WriteError(
            f"no forward serves {pv_name}"
            + (f" (last connect error: {last_err})" if last_err else ""))

    def _put_tunnel(self, pv_name, value, timeout):
        sock, circuit, chan = self._ensure_channel(pv_name, timeout)
        try:
            req = chan.write(value, notify=True)
            for b in circuit.send(req):
                sock.sendall(bytes(b))
        except Exception as e:
            self._chan_cache.pop(pv_name, None)
            raise WriteError(f"tunnel put {pv_name}={value} send failed: {e}") from e
        deadline = time.monotonic() + timeout
        ioid = getattr(req, "ioid", None)
        while time.monotonic() < deadline:
            try:
                cmds = _drain(sock, circuit, timeout=0.5)
            except Exception as e:
                self._chan_cache.pop(pv_name, None)
                raise WriteError(f"tunnel put {pv_name}={value} drain failed: {e}") from e
            for cmd in cmds:
                if isinstance(cmd, ca.WriteNotifyResponse) and (
                        ioid is None or cmd.ioid == ioid):
                    return
        self._chan_cache.pop(pv_name, None)
        raise WriteError(f"tunnel put {pv_name}={value} timed out waiting for ack")

    def close(self):
        for sock, _ in self._conns:
            try:
                sock.close()
            except Exception:
                pass
        self._conns = []
        self._chan_cache = {}
        if self._ctx is not None:
            try:
                self._ctx.disconnect()
            except Exception:
                pass
            self._ctx = None
