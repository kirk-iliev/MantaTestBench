# Multi-Forward EPICS Tunnel Mode — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let `pv_monitor.py` reach EPICS PVs spread across multiple IOCs by running one tunnel-mode circuit per `ssh -L` forward, with each PV auto-claiming the forward that serves it.

**Architecture:** Generalize the tunnel config from a single `{host,port}` to a *list* of forwards. `PVMonitor` spawns one background thread per forward; each opens every PV on its circuit (non-owning IOCs fast-fail via CreateChan-fail), and a lock-guarded `claimed` set ensures each PV is subscribed on exactly one forward. Native mode and the single-host tunnel config are preserved byte-for-byte.

**Tech Stack:** Python 3.12, `caproto` 1.3.0 (already in `.venv`), stdlib `threading`/`socket`/`select`. No new dependencies.

## Global Constraints

- **Back-compat is the hard gate:** native mode (no `_epics` / empty host) and the existing single-host tunnel form must behave exactly as before. The public API — `snapshot()`, `connected_count()`, `total_count()`, `start()`, `stop()` — and the per-shot sidecar contract stay unchanged. `test_bench_gui.py` is NOT modified.
- **Pure Python, no new deps.** caproto only.
- **Tests are script-style**, not pytest: plain `assert` + `print("ok  <name>")`, run via `.venv/bin/python test_pv_monitor.py`, added to the `__main__` block. They use NO live network (dead TEST-NET hosts) and NO IOC.
- **Failure-tolerant, always:** `start()` never blocks or raises; `stop()` is watchdog-bounded and never hangs. One bad PV/forward can't sink the rest.
- **Fast-test command:** `.venv/bin/python test_pv_monitor.py` — must print `all passed` (exit 0) at the end of every task.

---

## File Structure

- `pv_monitor.py` — MODIFY. `_parse_tunnel_cfg` (parsing → forwards list) and `PVMonitor` (per-forward threads, claimed set). `load_pv_config` body is unchanged (it just passes `_epics` through). The low-level socket helpers (`_drain`, `_connect_epics_socket`, `_open_channel_safe`, `_time_subscribe`, `_to_native`) are reused unchanged.
- `test_pv_monitor.py` — MODIFY. Update the single-host parse assertions to expect a list; add forwards-form parse tests and a multi-forward no-network start/stop test.
- `docs/EPICS_CONNECTIVITY.md` — MODIFY. Replace the "native-only / porting would take" status with the shipped multi-forward `ssh -L` recipe.

---

## Task 1: Config parsing returns a forwards list

**Files:**
- Modify: `pv_monitor.py` (`_parse_tunnel_cfg`, lines ~99-114)
- Test: `test_pv_monitor.py` (`test_config_parsing`, lines ~23-57)

**Interfaces:**
- Consumes: `load_pv_config(path) -> (pv_map, tunnel_cfg)` — body unchanged; it already calls `_parse_tunnel_cfg(raw.get("_epics"))`.
- Produces: `_parse_tunnel_cfg(block) -> list[{"host": str, "port": int}] | None`. Native → `None`. Single-host `{"host","port"}` → one-element list. `{"forwards": [...]}` → list (empty/all-invalid → `None`). `forwards` wins over `host` if both present.

- [ ] **Step 1: Update + add failing tests**

In `test_pv_monitor.py`, change the two single-host assertions in `test_config_parsing` to expect a one-element list:

```python
        # _epics with host -> tunnel (one-element forwards list); _-keys excluded
        p = _write(tmp, {"_epics": {"host": "localhost", "port": 15064}, "a": "PV:A"})
        pv_map, tun = load_pv_config(p)
        assert pv_map == {"a": "PV:A"}, pv_map
        assert tun == [{"host": "localhost", "port": 15064}], tun
```

```python
        # default port when omitted
        p = _write(tmp, {"_epics": {"host": "gw"}, "a": "PV:A"})
        _, tun = load_pv_config(p)
        assert tun == [{"host": "gw", "port": 5064}], tun
```

Then add these new blocks inside `test_config_parsing`, just before the `print("ok ...")` line:

```python
        # forwards list -> multi-forward tunnel, in order
        p = _write(tmp, {"_epics": {"forwards": [
            {"host": "localhost", "port": 15064},
            {"host": "localhost", "port": 15065}]}, "a": "PV:A"})
        pv_map, tun = load_pv_config(p)
        assert pv_map == {"a": "PV:A"}, pv_map
        assert tun == [{"host": "localhost", "port": 15064},
                       {"host": "localhost", "port": 15065}], tun

        # forwards: missing host -> localhost; bad port -> 5064; non-dict skipped
        p = _write(tmp, {"_epics": {"forwards": [
            {"port": 16000}, {"host": "h", "port": "x"}, "junk"]}})
        _, tun = load_pv_config(p)
        assert tun == [{"host": "localhost", "port": 16000},
                       {"host": "h", "port": 5064}], tun

        # empty forwards list -> native
        p = _write(tmp, {"_epics": {"forwards": []}, "a": "PV:A"})
        _, tun = load_pv_config(p)
        assert tun is None, tun

        # forwards wins over host when both present
        p = _write(tmp, {"_epics": {"host": "ignored",
                                    "forwards": [{"host": "localhost", "port": 1}]}})
        _, tun = load_pv_config(p)
        assert tun == [{"host": "localhost", "port": 1}], tun
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python test_pv_monitor.py`
Expected: FAIL — `AssertionError` on the updated single-host assertion (current code returns a dict, not a list).

- [ ] **Step 3: Rewrite `_parse_tunnel_cfg`**

Replace the entire current `_parse_tunnel_cfg` function in `pv_monitor.py` with:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python test_pv_monitor.py`
Expected: PASS — `ok  test_config_parsing` plus the other two existing tests; `all passed`.
(Note: `test_tunnel_start_never_blocks_or_raises` still passes a *dict* `tunnel_cfg` directly to `PVMonitor` and still passes — `PVMonitor` is untouched in this task and still accepts a dict.)

- [ ] **Step 5: Commit**

```bash
git add pv_monitor.py test_pv_monitor.py
git commit -m "feat(epics): parse _epics into a forwards list (multi-forward config)"
```

---

## Task 2: PVMonitor runs one circuit per forward (auto-claim)

**Files:**
- Modify: `pv_monitor.py` (module + class docstrings; `PVMonitor.__init__`; add `_normalize_forwards`; `start`; rename/parameterize `_run_tunnel_mode` → `_run_one_forward`; `stop`)
- Test: `test_pv_monitor.py` (add `test_multi_forward_start_never_blocks_or_raises`)

**Interfaces:**
- Consumes: `_parse_tunnel_cfg` → forwards list (Task 1); the unchanged helpers `_connect_epics_socket`, `_open_channel_safe`, `_time_subscribe`, `_drain`, `_to_native`, and `_RETRY_DELAYS`.
- Produces: `PVMonitor(pv_map, tunnel_cfg=None)` where `tunnel_cfg` accepts `None` (native), a single `{host,port}` dict (back-compat), or a list of such dicts. Same public methods, same cache entry shape `{"value","timestamp","connected"}`.

- [ ] **Step 1: Write the failing test**

Add to `test_pv_monitor.py` (after `test_tunnel_start_never_blocks_or_raises`):

```python
def test_multi_forward_start_never_blocks_or_raises():
    # Two dead forwards (TEST-NET, port 1): both threads back off forever;
    # start() must return instantly and stop() must not hang.
    pv_map = {"a": "PV:A", "b": "PV:B"}
    mon = PVMonitor(pv_map, tunnel_cfg=[
        {"host": "192.0.2.1", "port": 1},
        {"host": "192.0.2.2", "port": 1}])

    t0 = time.monotonic()
    mon.start()
    elapsed = time.monotonic() - t0
    assert elapsed < 0.5, f"start() blocked for {elapsed:.2f}s"

    snap = mon.snapshot()
    assert set(snap) == {"a", "b"}
    assert all(not v["connected"] for v in snap.values()), snap
    assert mon.connected_count() == 0
    assert mon.total_count() == 2

    t0 = time.monotonic()
    mon.stop()
    assert time.monotonic() - t0 < 5.0
    print("ok  test_multi_forward_start_never_blocks_or_raises")
```

And add it to the `__main__` block, after the existing tunnel test call:

```python
if __name__ == "__main__":
    test_config_parsing()
    test_tunnel_start_never_blocks_or_raises()
    test_multi_forward_start_never_blocks_or_raises()
    test_no_pvs_is_noop()
    print("\nall passed")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv/bin/python test_pv_monitor.py`
Expected: FAIL — `PVMonitor.__init__`/`start` don't yet accept a *list* `tunnel_cfg` (current code stores it as `self._tunnel_cfg` and `start()` treats the whole list as one `{host,port}`, raising/`KeyError` or never connecting). The exact failure may be a `TypeError`/`KeyError` in the tunnel thread or an assertion mismatch; any non-PASS is fine here.

- [ ] **Step 3: Rewrite `PVMonitor.__init__`, add `_normalize_forwards`, `start`**

Replace `PVMonitor.__init__` with (note new state: `_forwards`, `_tunnel_threads`, `_tunnel_socks`, `_claimed`):

```python
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
```

Replace `start` with (spawns one thread per forward):

```python
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
```

- [ ] **Step 4: Replace `_run_tunnel_mode` with `_run_one_forward`**

Replace the whole `_run_tunnel_mode` method with `_run_one_forward`, which takes a single forward and consults the shared `claimed` set:

```python
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
```

- [ ] **Step 5: Update `stop` to close all sockets and join all threads**

Replace the body of `stop` with:

```python
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
```

- [ ] **Step 6: Refresh the docstrings**

In the module docstring (top of file), change the tunnel-mode bullet to reflect multiple forwards. Replace the sentence beginning "Tunnel mode (off-subnet / lab WiFi via ``ssh -L``): a single raw-socket ``caproto.VirtualCircuit`` to a fixed ``host:port``…" with:

```
  * Tunnel mode (off-subnet / lab WiFi via ``ssh -L``): one raw-socket
    ``caproto.VirtualCircuit`` per configured forward, each to a fixed
    ``host:port``, bypassing the CA search phase entirely. Each PV is opened by
    name on every forward and auto-claims the one IOC that serves it (others
    fast-fail). Built for a per-IOC ``ssh -L`` forward (ideally to a CA gateway).
    See ``docs/EPICS_CONNECTIVITY.md``.
```

In the `PVMonitor` class docstring, replace the `tunnel_cfg` sentence with:

```
    ``tunnel_cfg`` selects the backend: ``None`` (native), a single
    ``{"host","port"}`` dict (back-compat), or a list of such dicts (one per
    ``ssh -L`` forward). Native mode runs when it is empty or caproto is absent.
```

- [ ] **Step 7: Run the full suite to verify it passes**

Run: `.venv/bin/python test_pv_monitor.py`
Expected: PASS — all four tests, ending in `all passed`. In particular `test_tunnel_start_never_blocks_or_raises` (dict back-compat) and `test_multi_forward_start_never_blocks_or_raises` (list) both pass.

- [ ] **Step 8: Commit**

```bash
git add pv_monitor.py test_pv_monitor.py
git commit -m "feat(epics): one tunnel circuit per forward with auto-claim"
```

---

## Task 3: Document the multi-forward recipe

**Files:**
- Modify: `docs/EPICS_CONNECTIVITY.md` (the "tunnel-mode option" and "Status of pv_monitor.py" sections, ~lines 91-128)

**Interfaces:** none (docs only).

- [ ] **Step 1: Update the tunnel-mode section**

In `docs/EPICS_CONNECTIVITY.md`, under "## The tunnel-mode option", replace the two "Caveats of tunnel mode" bullets' closing line about needing "a forward per IOC, or (better) a CA gateway" by appending this concrete recipe right after that section:

```markdown
### Multi-forward recipe (implemented)

`pv_monitor.py` now supports **one forward per IOC**. Confirmed on-site, the
PVs span multiple IOC hosts (all on the facility-pinned CA server port 35131):

```bash
# through the controls jump host appsdev2, one -L per IOC host:
ssh -L 15064:131.243.89.29:35131 \
    -L 15065:b04lx-dpsc.als.lbl.gov:35131 \
    kirkiliev@appsdev2
```

Then in `pv_config.json`:

```json
"_epics": {
  "forwards": [
    {"host": "localhost", "port": 15064},
    {"host": "localhost", "port": 15065}
  ]
}
```

Each PV is opened by name on every forward and auto-claims whichever IOC serves
it; PVs on neither IOC stay `disconnected`. (Port 35131 looks pinned
facility-wide — confirm it survives an IOC reboot; a dynamically-assigned server
port would change and break the forward.)
```

- [ ] **Step 2: Update the status section**

Replace the body of "## Status of pv_monitor.py + what porting would take" so it no longer says native-only. New content:

```markdown
## Status of pv_monitor.py

- Native mode (on-subnet) and tunnel mode (off-subnet) both ship.
- Tunnel mode takes a **list of forwards** under `_epics.forwards`; the older
  single `_epics.host`/`port` form still works (one forward). Each forward runs
  its own circuit on a background thread, reconnecting with backoff; a dead
  forward never blocks startup.
- The `snapshot()` / sidecar contract is unchanged — the GUI needs no changes.
```

- [ ] **Step 3: Verify the docs and that the shipped example still parses native**

Run: `.venv/bin/python -c "from pv_monitor import load_pv_config; print(load_pv_config('pv_config.example.json'))"`
Expected: the example (`_epics.host == ""`) still resolves to native — output ends with `, None)`. Confirms back-compat of the shipped example.

Read back the two edited sections to confirm the fenced code blocks render and the `ssh -L`/JSON are correct.

- [ ] **Step 4: Commit**

```bash
git add docs/EPICS_CONNECTIVITY.md
git commit -m "docs(epics): document the multi-forward ssh -L tunnel recipe"
```

---

## Self-Review

**Spec coverage:**
- Spec §1 Config schema (`forwards` list, back-compat table) → Task 1.
- Spec §2 Connection model (one thread per forward, start/stop) → Task 2 (Steps 3,5).
- Spec §3 Auto-claim (claimed set, fast-fail) → Task 2 (Step 4).
- Spec §4 Unchanged contracts → Global Constraints + `test_bench_gui.py` untouched; verified by Task 3 Step 3 (example parses native) and the dict back-compat test.
- Spec §Testing → Task 1 (parse: forwards, normalization, native fallthrough, malformed coercion) + Task 2 (multi-forward start/stop, no network).
- Spec §Docs → Task 3.

**Placeholder scan:** none — every code step shows complete code; every run step shows the exact command and expected output.

**Type consistency:** `_parse_tunnel_cfg` returns `list[{host,port}]|None` (Task 1) → `PVMonitor._normalize_forwards` accepts `None|dict|list` and yields `self._forwards: list` (Task 2) → `start()` iterates `self._forwards`, `_run_one_forward(forward)` reads `forward["host"]`/`forward["port"]`. Cache entry shape `{"value","timestamp","connected"}` is identical across native and tunnel paths. `_claimed`/`_tunnel_socks` are `set`s guarded by `self._lock`.
