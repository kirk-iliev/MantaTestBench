# EPICS Connectivity Notes — MantaTestBench

Reference for getting the test bench's EPICS PV metadata capture (`pv_monitor.py`)
to actually connect to the accelerator IOCs. Written up before a lab visit to
sort out the network situation on-site.

## TL;DR

- **The current code is native-mode only.** It will connect **only if the
  machine is on the controls subnet.**
- **On general lab WiFi (even physically standing in the linac): it will NOT
  connect.** Physical location is irrelevant — CA connectivity is about IP
  routing and firewalls, not geography.
- The failure is **silent**: no crash, frames/TIFFs still save, but every
  sidecar records `epics.<label>: disconnected` and you get zero accelerator
  data.

## How EPICS Channel Access (CA) connects

CA is a **discovery** protocol, not a point-to-point one. For a PV like
`EGUN:PHASE:RBV`:

1. **Search (UDP broadcast, port 5064):** client shouts "who serves this PV?"
2. **Reply:** the owning IOC answers with its IP + a TCP port.
3. **Connect (TCP):** client opens TCP to that IOC and subscribes.

`caproto.threading.client.Context()` takes no host/port — it drives this whole
dance off the network and standard EPICS environment variables.

### Relevant environment variables

| Variable | Purpose | When needed |
|---|---|---|
| `EPICS_CA_ADDR_LIST` | Explicit IPs/hosts (optionally `:port`) to send searches to | Off-subnet — point at IOCs or a CA gateway |
| `EPICS_CA_AUTO_ADDR_LIST` | `YES`/`NO` — also auto-broadcast on local subnets | Set `NO` to use only ADDR_LIST |
| `EPICS_CA_SERVER_PORT` | CA port (default 5064) | Non-standard facility port |
| `EPICS_CA_MAX_ARRAY_BYTES` | Max array payload | Large waveform PVs |

Set these **before** launching the GUI; nothing in the code configures them.

## Why WiFi-in-the-linac doesn't work

The IOCs live on a **controls/accelerator network** — its own subnet/VLAN,
almost always **firewalled off** from general lab/office/WiFi networks
(intentional: machine safety + security). Therefore from WiFi:

1. **UDP broadcast discovery dies** — broadcasts don't leave your own subnet,
   so they never reach the controls subnet. `EPICS_CA_AUTO_ADDR_LIST=YES`
   finds nothing.
2. **Even unicast `EPICS_CA_ADDR_LIST` won't help** unless there's an actual IP
   route AND the firewall permits UDP 5064 + TCP 5064 both ways — which an
   isolated controls net blocks by design.
3. **NAT would break it anyway** — the IOC advertises its own IP for the TCP
   leg; if anything NATs between WiFi and controls, that address is
   unreachable. (Same reason a plain `ssh -L` tunnel fails for native CA.)

Note: an Ethernet jack in the linac may itself be on the *general* VLAN — being
wired in isn't automatically being on the controls net. Confirm the port's VLAN.

## What actually makes it work

Be **on, or properly bridged into, the controls network** (by IP, not by
location):

- **Wired into a controls-network port** (verify the VLAN).
- **CA gateway** — controls group runs a gateway at the network boundary that
  re-serves selected PVs to the WiFi side. Set
  `EPICS_CA_ADDR_LIST=<gateway-ip>`, `EPICS_CA_AUTO_ADDR_LIST=NO`. Cleanest for
  a multi-IOC PV set like ours (EGUN:*, GTL:*).
- **VPN** into the controls network (makes the laptop logically on-subnet).
- **Run the acquisition on a control-room/console machine** already on the
  controls net, and remote in (RDP/X/VNC).

### Quick connectivity test (no GUI needed)

```bash
export EPICS_CA_ADDR_LIST="<gateway-or-ioc-ip>"
export EPICS_CA_AUTO_ADDR_LIST="NO"
caproto-get EGUN:PHASE:RBV       # or caget, if EPICS base tools are present
```

Value back → the GUI will work. Timeout → it's network/firewall, no code change
fixes it.

### The one question to ask controls

> "From general lab WiFi, can I reach the IOCs — is there a CA gateway, and what
> do I put in `EPICS_CA_ADDR_LIST`? If not, controls net is isolated and I need
> a wired port / VPN / on-subnet machine."

## The tunnel-mode option (from PyBeamViewer)

The sibling project `../PyBeamViewer` (`core/epics_layer.py`) solves the
off-subnet case with a **hybrid** EPICS layer, switched by one config field
`epics.host`:

- **`host == ""` → Native Mode:** normal `Context()` + UDP discovery (on-subnet).
- **`host` set → Tunnel Mode:** raw sockets, **bypasses the CA search phase
  entirely** by hand-building a `caproto.VirtualCircuit` straight to a
  `host:port` and opening each channel by name. All traffic stays inside one TCP
  connection — exactly what an SSH `-L` forward provides.

Run pattern:

```bash
# local 15064 → IOC's CA server port (5064) via a jump host on the controls net
ssh -L 15064:<ioc-host-or-ip>:5064  you@controls-gateway
# then set config: "host": "localhost", "port": 15064
```

**Caveats of tunnel mode:**
- It assumes the IOC serves on a **fixed, known TCP port** (works for
  areaDetector camera IOCs with a fixed `EPICS_CA_SERVER_PORT`; not guaranteed
  for arbitrary PVs that use ephemeral ports negotiated during search).
- One forward = one IOC's port. Our PVs span subsystems (EGUN:*, GTL:*) that may
  be **different IOCs on different ports** → need a forward per IOC (implemented),
  or a **CA gateway** on the jump host multiplexing them behind one port.

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
{
  "_epics": {
    "forwards": [
      {"host": "localhost", "port": 15064},
      {"host": "localhost", "port": 15065}
    ]
  },
  "gun_phase_deg": "EGUN:PHASE:RBV"
}
```

Each PV is opened by name on every forward and auto-claims whichever IOC serves
it; PVs on neither IOC stay `disconnected`. (Port 35131 looks pinned
facility-wide — confirm it survives an IOC reboot; a dynamically-assigned server
port would change and break the forward.)

## Status of pv_monitor.py

- Native mode (on-subnet) and tunnel mode (off-subnet) both ship.
- Tunnel mode takes a **list of forwards** under `_epics.forwards`; the older
  single `_epics.host`/`port` form still works (one forward). Each forward runs
  its own circuit on a background thread, reconnecting with backoff; a dead
  forward never blocks startup.
- The `snapshot()` / sidecar contract is unchanged — the GUI needs no changes.

## Fallback regardless of connectivity

The design degrades gracefully: if EPICS can't be reached, you still get every
frame + TIFF + camera/wall-clock timestamps. Accelerator data can then be merged
in afterward from an archiver (if the facility runs one) using those timestamps.
Given that the IOC clock, camera clock, and laptop clock are independent unless
NTP-synced, **post-hoc timestamp matching is arguably the more robust workflow
anyway** for a WiFi laptop.
