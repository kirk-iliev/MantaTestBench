# Multi-Forward Tunnel Mode for `pv_monitor.py`

**Date:** 2026-06-18
**Status:** Approved design, pre-implementation

## Problem

The test bench captures per-shot EPICS PV metadata via `pv_monitor.py`. Two
backends exist behind one interface: native mode (on the controls subnet, UDP
discovery) and tunnel mode (off-subnet via `ssh -L`, a single raw-socket
`caproto.VirtualCircuit` to one fixed `host:port`, bypassing CA search).

Acquisition must run on the laptop (the camera is physically attached there), so
EPICS has to be reached off-subnet through an SSH jump host (`appsdev2`, already
confirmed on the controls net). On-site `cainfo` showed the PVs we need are
served by **multiple IOC hosts**:

```
LTB:Q1_1:Setpoint  ->  131.243.89.29:35131
LTB:Q1_2:Setpoint  ->  b04lx-dpsc.als.lbl.gov:35131
```

Two PVs in the *same* subsystem live on *different* IOCs, both on TCP port
`35131` (looks like a facility-pinned `EPICS_CA_SERVER_PORT`). Current tunnel
mode connects a single circuit to a single `host:port`, so it can only reach PVs
served by one IOC. Anything on another IOC stays `disconnected`.

No CA gateway is in this path (`cainfo` shows real IOC hosts, not a gateway), so
we cannot collapse the IOCs behind one port. We need tunnel mode to span
multiple SSH forwards, one per IOC host.

## Goals

- Reach PVs spread across multiple IOCs via multiple `ssh -L` forwards.
- A PV finds its forward automatically — no per-PV bookkeeping.
- Native mode and existing single-host tunnel config keep working unchanged.
- The public interface (`snapshot()`, sidecar contract, GUI wiring) is untouched.

## Non-Goals

- The tool does **not** spawn or manage `ssh`. The user opens the forwards
  manually (documented recipe); the config only references the local ports.
- No CA gateway support beyond what already works (a gateway would just be one
  forward in the list).
- No change to native mode behavior.

## Design

### 1. Config schema

A plural `forwards` list under `_epics`. Each entry is a local port produced by
an `ssh -L` forward; `host` defaults to `localhost`.

```json
{
  "_epics": {
    "forwards": [
      {"host": "localhost", "port": 15064},
      {"host": "localhost", "port": 15065}
    ]
  },
  "ltb_q1_1_setpoint": "LTB:Q1_1:Setpoint",
  "ltb_q1_2_setpoint": "LTB:Q1_2:Setpoint"
}
```

Backward compatibility (behavior unchanged):

| Config | Mode |
|---|---|
| no `_epics`, or `host: ""` | native |
| old single `{"host": "...", "port": ...}` | one-forward tunnel (normalized to a one-element list) |
| `forwards: []` or empty | native |
| `forwards: [...]` non-empty | multi-forward tunnel |

`_parse_tunnel_cfg()` generalizes to return a **list of forwards** (or `None`),
normalizing the old single-host form into a one-element list so there is one
downstream code path. A forward entry with a missing/empty host defaults to
`localhost`; a malformed port defaults to `5064` (same coercion as today).

### 2. Connection model — one thread per forward

Today: one background thread, one circuit. New: **one thread per forward**, each
running the existing connect -> open-all -> subscribe -> drain ->
reconnect-with-backoff loop, all feeding the same lock-guarded cache.

- `start()` spawns one thread per forward (native mode path unchanged).
- `stop()` sets the shared stop-event, closes each thread's socket to unblock its
  `select()`, and joins all threads under the same watchdog bound as today.
- `_run_tunnel_mode` is parameterized by the single forward it owns; the body is
  otherwise the proven existing loop (handshake, open-all-then-subscribe-all,
  sole-drainer main loop, mark-all-disconnected on drop, backoff).

*Alternative considered and rejected:* a single thread multiplexing N sockets in
one `select()`. It would rewrite the drain loop and entangle per-forward backoff
for no benefit at our scale (2-3 IOCs). One-thread-per-forward reuses the
existing loop almost verbatim.

### 3. Auto-claim (self-sorting)

Each forward's thread attempts to open **every** PV on its circuit. The owning
IOC connects the channel; every other IOC fast-fails with a CreateChan-fail
response (`_open_channel_safe` already returns `None` on the FAILED state — no
hang). A small shared `claimed` set, guarded by the existing lock, lets a thread
skip a PV another forward already owns so we never double-subscribe and don't
re-attempt churn on reconnect. On a circuit drop, that thread releases its own
claimed labels and marks them `disconnected`, so they can be re-claimed on
reconnect. A PV no IOC serves stays `{"connected": false}` — the same
graceful-degrade contract as today.

### 4. Unchanged contracts

`snapshot()`, `connected_count()`, `total_count()`, the per-shot sidecar lines,
and the GUI wiring (`_init_pv_monitor` in `test_bench_gui.py`) stay byte-for-byte.
Native mode is untouched. Only the tunnel internals gain the forward-list
dimension.

## Testing

Mirror the existing `test_pv_monitor.py` style:

- `load_pv_config` parses the new `forwards` form.
- Old single-host form normalizes to a one-element forward list.
- Native fallthrough: no `_epics`, empty host, and empty `forwards` all yield
  native mode (`tunnel_cfg is None`).
- Malformed entries: bad port coerces to default; missing host defaults to
  `localhost`; non-dict `_epics` -> native.
- No-network behavior: multi-forward `start()` spawns the expected number of
  threads and `stop()` joins cleanly with caproto mocked/absent (as the suite
  already handles).

Live socket behavior is validated on-site with a manual probe rather than in CI:

```bash
export EPICS_CA_ADDR_LIST="localhost:15064"
export EPICS_CA_AUTO_ADDR_LIST="NO"
caproto-get LTB:Q1_1:Setpoint
```

## Docs

Update `docs/EPICS_CONNECTIVITY.md`:

- Replace the "native-only / porting would take..." section with the shipped
  multi-forward recipe.
- Document the exact forwards through `appsdev2`:

  ```bash
  ssh -L 15064:131.243.89.29:35131 \
      -L 15065:b04lx-dpsc.als.lbl.gov:35131 \
      kirkiliev@appsdev2
  ```

- Note port `35131` appears pinned facility-wide; confirm it survives an IOC
  reboot (a pinned `EPICS_CA_SERVER_PORT` does; a dynamically-assigned one would
  change and break the forward).
- Keep the post-hoc archiver-timestamp fallback note.

## Rollout

1. Generalize `_parse_tunnel_cfg` + `load_pv_config` (forwards list).
2. Parameterize `_run_tunnel_mode` per forward; spawn/join per-forward threads in
   `start()`/`stop()`; add the shared `claimed` set.
3. Tests.
4. Docs + a real `pv_config.json` (gitignored) / refreshed `pv_config.example.json`.
