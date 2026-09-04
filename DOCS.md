# v2x-bridge - extended documentation

A single ns-3 program that couples an external Python process (CARLA/OpenCDA, or the
provided `test_harness.py`) with a simulated 5G NR network:

```
 CARLA / OpenCDA                v2x-bridge (ns-3 + 5G-LENA)
┌───────────────┐   real UDP   ┌──────────────────────────────────────────────┐
│  Python app   │ ──────────►  │ poll OS socket ── JSON command               │
│               │  JSON cmds   │   │ 1) update UE positions                   │
│               │              │   │ 2) simulated UDP send over NR Uu:        │
│               │  ◄──────────                                                │
│               │  JSON reply  │   UE ──► gNB ──► SGW/PGW ──► "BS"/MEC host   │
│               │              │   (or reverse, or UE ──► PGW ──► UE)         │
└───────────────┘              │   3) log CSVs, 4) reply if requested         │
                               └──────────────────────────────────────────────┘
```

The real UDP datagrams are **control messages**, not tunnelled traffic: each one
describes the scene (vehicles/VRUs) and, optionally, one packet that the bridge then
generates **inside** the simulation between the corresponding NR nodes. The bridge stays
alive indefinitely (stop with Ctrl-C or a `shutdown` message).

Built and tested on **ns-3.46 + 5G-LENA (nr) v4.1.1**.

## Time synchronization (two modes)

**Default — external clock (OpenCDA/CARLA lockstep).** The external simulator is the
clock master and ns-3 never runs ahead of it. In practice this is one ordinary ns-3
event — `ExternalSyncLoop()` — whose callback does not return until a command arrives:
it sits in a `poll()` on the real UDP socket (waking every 100 ms only to check for
Ctrl-C). ns-3 is single-threaded and runs one event at a time, so the whole simulator
waits inside that call: wall-clock time passes, simulated time does not. This is the
"gate", and exactly one of them is always pending.

Once commands arrive, the gate does three things and returns: it maps every command's
`timestamp` onto the simulation clock and schedules the command at that simulated time;
it schedules the next gate `--drainMs` after the newest of them; and by returning it
lets ns-3 resume, which jumps the clock forward to the first of those events.

**The mapping.** External time and simulation time tick at the same rate but start from
different origins, so the bridge locks them together on the first command it ever
receives: that timestamp becomes the origin and is mapped to `--warmupMs` of simulation
time (default 500 ms, which the UE pool needs to complete its RRC attach before any
packet flies). Every later command lands at

```
t_sim = warmupMs + (t_ext − t_ext_first)
```

e.g. with `--warmupMs=500` and a first command at `t_ext = 6.735 s`, that command runs
at `t_sim = 0.500 s`, one at `t_ext = 6.785 s` runs at `t_sim = 0.550 s`, and one at
`t_ext = 10.735 s` runs at `t_sim = 4.500 s`. Nothing is scaled or dropped; a command
whose timestamp is already behind the simulation clock (out of order, or inside an
already-simulated drain window) simply runs at the current time.

**Why `--drainMs` (default 20 ms).** It leaves a slice of simulated time free to run
before the next gate freezes the clock again. Packets need that slice: sending a packet
does not deliver it, it only schedules its arrival a few milliseconds later, and a
frozen clock never reaches that arrival. Example, with CARLA ticking every 50 ms and
`--drainMs=20`: a command for `t_sim = 0.500 s` sends a packet whose arrival is ~7 ms
away, the next gate is placed at `0.520 s`, so the simulation runs that far, the packet
is delivered at ~`0.507 s` and its reply reaches OpenCDA inside the same CARLA tick.
With `--drainMs=0` the next gate would sit at `0.500 s`, no time would pass at all, and
that reply would only come out on the following tick — 50 ms late. Keep `--drainMs`
below the external tick period (CARLA's `fixed_delta_seconds`): if it is longer, the
clock runs past the time the next command belongs to, and that command then has to be
executed "now" instead of at its own timestamp.

Wall-clock speed is irrelevant in this mode: a paused, slower- or faster-than-real-time
external simulator only makes the gate block longer, which is exactly what OpenCDA
(CARLA 0.9.12, synchronous mode) needs.

**Optional — real time (`--realtime`).** The program runs under
`ns3::RealtimeSimulatorImpl` (best-effort), simulation time tracks the wall clock, and a
recurring 1 ms poll event drains the real socket, so incoming commands are executed as
they arrive (their `timestamp` is ignored). There is no gate and no drain window in this
mode. Use it when the external application has no usable time reference of its own.

In both modes CSV timestamps and latencies are simulation time; in external-clock mode
sim time maps back to the external clock as `t_ext = t_ext0 + (t_sim − warmupMs)`.

## Files

| File | Purpose |
|---|---|
| `v2x-bridge.cc` | the whole bridge (auto-built by the `scratch/` machinery) |
| `json.hpp` | vendored [nlohmann/json](https://github.com/nlohmann/json) 3.11 single header (MIT) |
| `test_harness.py` | standalone CARLA/OpenCDA stand-in for end-to-end testing |
| `DOCS.md` | this document (protocol, CSV formats, architecture notes) |

## Build & run

```sh
cd <ns-3-root>
./ns3 build v2x-bridge

# terminal 1 — the bridge (gNB position via CLI):
./ns3 run "v2x-bridge --gnbX=0 --gnbY=0 --gnbZ=10 --maxUes=10"

# terminal 2 — the test harness (sends 12 commands, then shutdown):
python3 scratch/v2x-bridge/test_harness.py --num 12 --shutdown
```

Expected harness output: one `-->` line per command and one `<--` JSON reply each,
ending with a summary such as `sent=12 delivered=12 timed_out=0 mean_latency=6.047 ms`.
The two CSVs (`v2x-send.csv`, `v2x-recv.csv` by default, written in the ns-3 root when
run via `./ns3 run`) fill up in parallel.

## CLI arguments

| Arg | Default | Meaning |
|---|---|---|
| `--gnbX/--gnbY/--gnbZ` | 0 / 0 / 10 | gNB (BS) position [m] |
| `--maxUes` | 10 | UE pool size = max distinct vehicle/VRU `origin_ID`s for the run |
| `--listenPort` | 5555 | real OS UDP port for JSON control messages |
| `--realtime` | false | wall-clock pacing (`RealtimeSimulatorImpl`) instead of following the external clock |
| `--warmupMs` | 500 | external-clock mode: sim time mapped to the first received timestamp |
| `--drainMs` | 20 | external-clock mode: extra sim time processed after each command batch (keep < external tick) |
| `--simAppPort` | 6000 | UDP port used inside the simulation (src *and* dst of app packets) |
| `--timeoutMs` | 500 | per-packet delivery timeout before a `timeout` row/reply |
| `--csvSend` | `v2x-send.csv` | send-log CSV path |
| `--csvRecv` | `v2x-recv.csv` | receive/failure-log CSV path |
| `--gnbTxPowerDbm` | 40 | gNB Tx power [dBm] |
| `--ueTxPowerDbm` | 23 | UE Tx power [dBm] |
| `--gnbNumRows/--gnbNumColumns` | 4 / 8 | gNB antenna panel (3GPP element) |
| `--ueNumRows/--ueNumColumns` | 1 / 2 | UE antenna panel (isotropic elements) |
| `--gnbDowntiltDeg` | 6 | gNB electrical downtilt [deg] |
| `--scenario` | `UMi` | 3GPP channel scenario (`UMi` or `UMa`) |
| `--channelUpdateMs` | 0 | fading update period [ms]; >0 lightens CPU load (0 = model default) |
| `--shadowing` | false | enable log-normal shadowing |
| `--injected-loss-perc` | 0 | % of the packets discarded on purpose at the destination, on top of the NR model's own losses (can be used to simulated unreliable networks even in presence of a few actual simulated UEs) |
| `--verbose` | false | per-packet progress on stdout |

Fixed (per project requirements): band **n78** (3.5 GHz), **60 MHz** channel,
**numerology 1**, **TDD** pattern `DL|S|UL|UL|DL|DL|S|UL|UL|DL|` (DL/UL-symmetric),
**IPv4 only**.

### Parameter rationale

- **40 dBm gNB**: 10 W over 60 MHz ≈ 22 dBm/MHz — a mid-range urban macro/small-macro
  figure (3GPP TR 38.802/38.913 urban BS classes span ~33–44 dBm).
- **23 dBm UE**: NR power class 3 (3GPP TS 38.101-1), the standard handset / vehicle OBU class.
- **4×8 gNB panel, 3GPP element, 6° downtilt**: a typical mid-band urban sector panel;
  the `ThreeGppAntennaModel` element gives the 3GPP directional pattern (~8 dBi).
- **1×2 isotropic UE array**: small rooftop/handheld antenna.
- **UMi scenario**: street-level urban micro cell matching the 10 m default gNB height
  (switch to `UMa` for above-rooftop macro deployments).
- **500 ms timeout**: far above any plausible NR round trip (delivered packets measure
  2–10 ms) yet short enough to matter for V2X decision loops.

## Injected packet loss (`--injected-loss-perc`)

`--injected-loss-perc=X` discards, on purpose, X % of the simulated packets, so that an
external application can be studied on an unreliable link without touching the radio
configuration (the NR model on its own loses almost nothing if there are few UEs.
`0` (the default) disables any injected packet loss.

```bash
./ns3 run "v2x-bridge --gnbX=-42 --gnbY=25 --maxUes=32 --injected-loss-perc=10"
```

This option works as follows:

- **The drop happens at the destination application socket**, not at the sender: the
  packet is really generated, scheduled and transmitted over the air interface, so the
  radio load, the latency distribution of the surviving packets and every other
  measurement are identical to a run without the option: only the delivery is
  suppressed. The result is an end-to-end loss of `1 − (1 − p_NR)(1 − X)`, i.e. the
  injected loss composes with whatever the NR model itself really loses instead of hiding it.
- **The injected loss is managed at the moment the packet would have been delivered** (a few
  ms), not at `--timeoutMs`: a `lost` row is written to the received packets CSV file and,
  if `request_reply` was set, a `lost` reply is sent immediately. Hence, any external application
  never stalls waiting on a delivery that will not come due to the injected loss.
- **Draws come from a dedicated random stream** created before any model object, so that the
  loss sequence is fixed by `--RngRun`/`--RngSeed` alone and does not shift when an
  unrelated parameter (`--maxUes`, `--scenario`, or others) changes the number of random
  variables the scenario instantiates. Use `--RngRun=N` for independent repetitions of
  the same loss rate.

On simulation termination the bridge prints the realized packet loss, e.g.
`injected loss dropped 30 of the 58 packet(s) that reached their destination (51.72 %,
target 50.00 %)`. The realized packet loss is over the packets that reached their
destination, so it should converge to X % and is unaffected by the packets the NR model loses
by itself.

## JSON protocol

One UDP datagram = one JSON object. Three message types: `control` (default),
`shutdown`, and the bridge's `reply`.

### Endpoint identity: `origin_ID`

An `origin_ID` is the **name of one endpoint** of the simulated network: a string chosen
by the external application, unique per actor and stable for the whole run (OpenCDA uses
`"Cav1"`, `"ped_3"`, …; `test_harness.py` uses `"veh_1"`). Every field that refers to an
endpoint — `entities[].origin_ID`, `packet.sender`, `packet.receiver`, and the
`sender`/`receiver` columns of the CSV logs — uses this string, never an IP address.

- **It is an identity, not an address.** The bridge creates and attaches a pool of
  `--maxUes` UEs at t=0. The first time an `origin_ID` appears in `entities` it
  permanently claims one free slot — an ns-3 node with its own IPv4 address — and that UE
  is teleported to the reported position; the bridge prints
  `origin_ID "ped_3" -> UE slot 4 (IP 7.0.0.6)`. Every later appearance simply moves the
  same UE. The mapping is never released.
- **Keep the string stable.** Same actor ⇒ same `origin_ID` on every tick; matching is
  exact and case-sensitive, so a renamed actor is a brand-new endpoint and burns a second
  slot.
- **`--maxUes` caps the number of *distinct* IDs over the whole run**, not how many are in
  view at once. Size it to every vehicle + VRU that will ever appear. Beyond that the
  bridge prints `UE pool exhausted, cannot map origin_ID "..." (raise --maxUes)` and the
  entity is ignored.
- **`"BS"` is reserved** for the edge/MEC application endpoint (the EPC remote host, see
  "Architecture notes"). It always resolves as a `sender`/`receiver` and consumes no UE
  slot. Listing it in `entities` is pointless but harmless: the bridge ignores the entry
  and warns on stderr, since the base-station position is fixed by
  `--gnbX`/`--gnbY`/`--gnbZ`.
- **An ID must be introduced before it can transmit.** `packet.sender` and
  `packet.receiver` must already be known: either from an earlier datagram, or from the
  `entities` array of the same datagram (what OpenCDA and `test_harness.py` do). An
  unknown ID produces an `error` reply (`unknown sender`/`unknown receiver`) and no packet
  is simulated.

### Command (`msg_type: "control"`)

```json
{
  "msg_type": "control",
  "timestamp": 12.35,
  "entities": [
    {
      "timestamp": 1721211055.123,
      "origin_ID": "veh_1",
      "origin_vehicle_type": "car",
      "Position": { "x_m": 120.5, "y_m": -14.2, "z_m": 1.5 },
      "Velocity": 13.9,
      "Heading": 1.570796
    }
  ],
  "packet": {
    "sender": "veh_1",
    "receiver": "BS",
    "size_bytes": 300,
    "packet_id": 42,
    "type": "detection",
    "request_reply": true
  }
}
```

- `timestamp` (top level, seconds): the external simulator's current time — **the time
  reference that drives the simulation clock in the default mode** (see "Time
  synchronization"). If absent, the bridge falls back to the newest entity `timestamp`;
  a command with no timestamp at all is executed at the current sim time without
  advancing the clock (fine in `--realtime` mode, warned about otherwise).
- `entities` (optional array): current state of vehicles/VRUs. Units: metres (`x_m`,
  `y_m`, `z_m`), m/s (`Velocity`), radians (`Heading`), external epoch seconds
  (`timestamp`, informational). `origin_vehicle_type` is free-form (`"car"`, `"truck"`,
  `"pedestrian"`, …; `"base_station"` is reserved for the BS).
  The first appearance of an `origin_ID` claims a UE from the pool and every later one
  updates its position (see "Endpoint identity: `origin_ID`" above).
  `Velocity`/`Heading` are stored but do not drive mobility — the external simulator
  refreshes positions, which is the co-simulation pattern.
- `packet` (optional object): one packet to transmit over the simulated NR network.
  - `sender` / `receiver`: `origin_ID`s, or the reserved ID **`"BS"`** for the base
    station side. Both must be known (introduced in this or an earlier datagram);
    sender ≠ receiver.
  - `size_bytes`: simulated UDP payload size (clamped to [12, 60000] and never smaller
    than 12 + the application payload length; the first 12 bytes carry the packet id and
    the payload length).
  - `packet_id`: unique integer; must not collide with an id still in flight.
  - `type`: `"detection"` or `"warning"`.
  - `payload` (optional): **application payload**, base64-encoded — the actual bytes of
    the message being simulated (a UPER-encoded ETSI CAM/VAM, a JSON detection list, a warning
    JSON, ...). The bytes are embedded in the simulated packet (after the 12-byte
    id+length header), traverse the simulated NR network, are extracted at the
    destination node and returned base64-encoded in the `payload` field of the
    delivery reply — so the actual information (not only the size metadata) reaches
    the simulated receiver. Without `payload` the simulated packet is zero-filled
    (metadata-only simulation, as before).
  - `request_reply`: **the return trigger** — the bridge sends a real UDP reply for this
    packet only if `true` (default `false`). The external application therefore decides
    when a return message is emitted.
- A datagram may carry only `entities` (scene update) or only `packet` (send between
  already-known endpoints).

### Shutdown

`{"msg_type": "shutdown"}` → the bridge answers `{"msg_type":"shutdown_ack", ...}` and
stops cleanly. Ctrl-C (SIGINT/SIGTERM) does the same without the ack.

### Reply (bridge → external app, real UDP)

Sent to the **source address:port of the triggering command datagram**, only when
`request_reply` was true:

```json
{
  "msg_type": "reply",
  "packet_id": 42,
  "type": "detection",
  "status": "delivered",
  "latency_ms": 3.214,
  "sim_time_ms": 15234.417
}
```

- `status`: `delivered` | `timeout` | `lost` | `error` | `error_duplicate`.
  `lost` is only produced by `--injected-loss-perc`: the packet crossed the simulated
  network but was discarded at the destination, and carries
  `"error_msg": "discarded by injected packet loss"`. Clients that do not know the
  status treat it, correctly, as a non-delivery.
- `latency_ms`: one-way application-layer delay in **simulation time**
  (receive − send); `null` unless delivered (`lost` packets report no latency).
- `payload` (only when the command carried one and the packet was delivered): the
  application payload as extracted from the simulated packet at the destination node,
  base64-encoded. The external application uses it to hand the received bytes to the
  destination-side logic (e.g. the edge server LDM).
- `error` replies add an `error_msg` string (e.g. `unknown sender "ghost"`,
  `ue_pool_exhausted` style messages appear on stderr too).

### Round-trip reference: requesting a transmission and getting the delivery confirmation

This is the exact exchange OpenCDA performs when a vehicle uploads a message to the MEC
server and must wait for confirmation before proceeding.

**1. Transmission request** (OpenCDA → bridge, one UDP datagram to `--listenPort`).
Every field needed for a vehicle→MEC transmission, in one self-contained message:

```json
{
  "msg_type": "control",
  "timestamp": 12.35,
  "entities": [
    {
      "timestamp": 12.35,
      "origin_ID": "veh_1",
      "origin_vehicle_type": "car",
      "Position": { "x_m": 120.5, "y_m": -14.2, "z_m": 1.5 },
      "Velocity": 13.9,
      "Heading": 1.570796
    }
  ],
  "packet": {
    "sender": "veh_1",
    "receiver": "BS",
    "size_bytes": 300,
    "packet_id": 42,
    "type": "detection",
    "request_reply": true
  }
}
```

| Field | Required | Notes |
|---|---|---|
| `msg_type` | no | defaults to `"control"` |
| `timestamp` | **yes** (default mode) | OpenCDA/CARLA simulation time [s]; drives the ns-3 clock. Ignored under `--realtime` |
| `entities[]` | first time only | the sender's state **must** have appeared at least once (same datagram is fine — entities are processed before the packet). Afterwards optional but recommended every tick to keep positions fresh |
| `packet.sender` | **yes** | `origin_ID` of the transmitting vehicle/VRU |
| `packet.receiver` | **yes** | **`"BS"`** = the MEC server at the gNB site |
| `packet.size_bytes` | **yes** | payload size of the simulated UDP packet |
| `packet.packet_id` | **yes** | unique integer, echoed in the confirmation — OpenCDA's correlation key |
| `packet.type` | **yes** | `"detection"` or `"warning"` |
| `packet.request_reply` | **yes, `true`** | without it no confirmation is ever sent |

**2. Delivery confirmation** (bridge → OpenCDA, one UDP datagram back to the exact
source address:port of the request). Received when the packet has been delivered to the
MEC server over the simulated NR uplink — OpenCDA can then proceed with its processing:

```json
{
  "msg_type": "reply",
  "packet_id": 42,
  "type": "detection",
  "status": "delivered",
  "latency_ms": 5.065,
  "sim_time_ms": 12850.065
}
```

OpenCDA should treat the transmission as successfully completed **iff**
`msg_type == "reply"` **and** `packet_id` matches the request **and**
`status == "delivered"`. Any other `status` means the packet did **not** reach the MEC
server: `timeout` = lost on the radio link (all HARQ attempts failed, or not delivered
within `--timeoutMs`); `error`/`error_duplicate` = the request itself was invalid
(details in `error_msg`).

Timing of the confirmation: with the default `--drainMs` (20 ms ≥ typical uplink
latency) a `delivered` confirmation arrives right after the request datagram is
processed, within the same external tick. A `timeout` verdict arrives only once later
commands have advanced simulation time past send + `--timeoutMs` (see "Time
synchronization"), so OpenCDA should wait for the confirmation with a bounded timeout
or poll its socket non-blockingly each tick, matching replies to requests via
`packet_id` — not assume strict request/reply alternation.

### Example: VAM (VRU Awareness Message)

A VAM is an *awareness* message sent by a VRU, so in this schema it is simply a packet
of `type: "detection"` whose **sender is a VRU** (`origin_vehicle_type` `"pedestrian"`,
`"cyclist"`, …) and whose receiver is the MEC server. There is no separate message
type: awareness-style uplink messages (CAM/CPM from vehicles, VAM from VRUs) all use
`"detection"`, and the sender's entity state tells them apart.

```json
{
  "msg_type": "control",
  "timestamp": 12.40,
  "entities": [
    {
      "timestamp": 12.40,
      "origin_ID": "ped_1",
      "origin_vehicle_type": "pedestrian",
      "Position": { "x_m": 25.1, "y_m": 3.8, "z_m": 1.5 },
      "Velocity": 1.25,
      "Heading": 0.7854
    }
  ],
  "packet": {
    "sender": "ped_1",
    "receiver": "BS",
    "size_bytes": 350,
    "packet_id": 43,
    "type": "detection",
    "request_reply": true
  }
}
```

Confirmation (identical semantics to the vehicle case — OpenCDA proceeds on
`status == "delivered"`):

```json
{
  "msg_type": "reply",
  "packet_id": 43,
  "type": "detection",
  "status": "delivered",
  "latency_ms": 6.565,
  "sim_time_ms": 12906.565
}
```

### Example: warning message (downlink, MEC → vehicle or VRU)

Warnings travel the opposite way: after OpenCDA has processed the received
detections/VAMs and *decides* that someone must be alerted (this decision belongs to
OpenCDA — the bridge never generates warnings on its own), it commands a packet of
`type: "warning"` with **sender `"BS"`** and the endangered vehicle or VRU as receiver.
The receiver must already be known to the bridge (it normally is, since it has been
reporting its state); including its fresh entity state in the same datagram is allowed
and keeps its position current:

```json
{
  "msg_type": "control",
  "timestamp": 12.45,
  "entities": [
    {
      "timestamp": 12.45,
      "origin_ID": "veh_2",
      "origin_vehicle_type": "truck",
      "Position": { "x_m": -62.0, "y_m": 44.3, "z_m": 1.5 },
      "Velocity": 10.0,
      "Heading": 3.1416
    }
  ],
  "packet": {
    "sender": "BS",
    "receiver": "veh_2",
    "size_bytes": 200,
    "packet_id": 44,
    "type": "warning",
    "request_reply": true
  }
}
```

Confirmation that the warning was delivered to the vehicle over the simulated downlink
(downlink is typically the fastest path, ≈2 ms):

```json
{
  "msg_type": "reply",
  "packet_id": 44,
  "type": "warning",
  "status": "delivered",
  "latency_ms": 2.171,
  "sim_time_ms": 12952.171
}
```

Warning a VRU works identically with `receiver: "ped_1"`. A direct
vehicle-to-vehicle/VRU warning (e.g. `sender: "veh_1"`, `receiver: "ped_1"`) is also
supported — it traverses the full Uu path (uplink + core + downlink, ≈7–10 ms), since
this bridge models infrastructure-mode V2X without sidelink. Note that a `timeout` on a
warning is safety-relevant information in itself: it tells OpenCDA the alert never
reached the target, so it can re-issue the warning with a new `packet_id`.

## CSV formats

Both files are created with a header row and flushed after every row (rows survive a
crash/kill). All timestamps are **simulation time** in ms with 3 decimals (µs
resolution); in the default mode this maps 1:1 onto the external simulator's clock
(offset by `--warmupMs`), while under `--realtime` it tracks wall-clock time since
program start. The two files join on `packet_id`.

`--csvSend` (one row per packet command, written at send time — including rejected
commands, so the files stay join-able):

```
packet_id,receiver,sender,size_bytes,tx_sim_time_ms,type
42,BS,veh_1,300,15231.203,detection
```

`--csvRecv` (one row per outcome; `latency_ms` empty unless delivered):

```
packet_id,rx_sim_time_ms,latency_ms,type,status
42,15234.417,3.214,detection,delivered
43,16561.000,,warning,timeout
44,16612.883,,detection,lost
```

`status` is `delivered` | `timeout` | `lost` (discarded by `--injected-loss-perc`, at
the sim time the packet reached its destination) | `error` | `error_duplicate`.

## Network / addressing plan

| Element | Address / port |
|---|---|
| real control listener | `0.0.0.0:5555` (UDP, `--listenPort`) |
| real replies | source addr:port of the incoming command |
| simulated app traffic | UDP, src port = dst port = `6000` (`--simAppPort`) |
| UEs | `7.0.0.2 …` (EPC UE pool, assigned by `NrPointToPointEpcHelper`) |
| BS/MEC endpoint (remote host) | `1.0.0.2` (PGW↔host p2p link `1.0.0.0/8`) |
| EPC internals | S1-U `10.0.0.0/24`, S5 `14.0.0.0/24`, S11 `13.0.0.0/24` (helper defaults) |

## Architecture notes

- **"BS" endpoint = MEC remote host.** A gNB in ns-3/5G-LENA is a radio node, not an IP
  application host. The reserved `"BS"` endpoint is therefore an edge server: the EPC
  remote host behind a **zero-delay 100 Gb/s** point-to-point link to the PGW
  (`NrEpcHelper::SetupRemoteHost`), i.e. "an application co-located with the gNB site".
  Measured latency to/from `"BS"` is the radio + core path only.
- **Pre-created UE pool.** 5G-LENA (v4.1.1) does not support installing/attaching UEs
  after `Simulator::Run()` — every in-tree example attaches before the run. The bridge
  therefore creates and attaches all `--maxUes` UEs at t=0, parked a few metres from the
  gNB, and *activates* one (teleports it to its reported position) the first time its
  `origin_ID` appears. The `origin_ID → UE` mapping is permanent.
- **Beamforming refresh on movement.** Ideal beamforming vectors are refreshed every
  100 ms by default; a packet sent immediately after a large position jump would
  otherwise go out on a stale beam and (for the 23 dBm uplink, under RLC UM with no
  retransmissions) be lost after HARQ exhaustion. The bridge therefore calls
  `IdealBeamformingHelper::Run()` whenever a scene update actually placed or moved a UE,
  before any packet of the same datagram is sent, so beams always match the just-applied
  positions.
- **Packet identity (and application payload) in the payload.** Every simulated packet
  is laid out as

  ```
  [0..7]   packet id        (big-endian uint64)
  [8..11]  payload length L (big-endian uint32, 0 = no payload)
  [12..]   the L payload bytes
  ```

  with `L = 0` when the command carried no `payload`; a packet shorter than 12 B carries
  the id alone, with no length field, and is accepted as payload-less. A payload survives
  RLC segmentation/reassembly and GTP-U tunnelling unconditionally, which ns-3 packet tags
  are not guaranteed to do across the whole UE↔EPC path. All other per-packet metadata
  (send time, type, reply address) stays host-side in a pending map; the timeout event
  is cancelled on delivery.
- **UE→UE traffic** hairpins through the core (UE → gNB → SGW/PGW → gNB → UE): correct
  Uu behaviour for infrastructure-mode V2X (no sidelink/PC5). A dedicated
  `NGBR_LOW_LAT_EMBB` bearer with a TFT on `--simAppPort` classifies these flows in both
  directions.
- **nr v4.1.1 quirk worked around:** `SetupRemoteHost` returns the **PGW-side** address
  of the PGW↔host link rather than the remote host's; using it as a destination gets
  ICMP port-unreachable from the PGW. The bridge reads the remote host's own interface
  address instead (see comment in `v2x-bridge.cc`).
- **RLC UM (default with EPC)** means lost transport blocks after HARQ exhaustion are
  real, permanent losses — which is what makes the `timeout` status meaningful. Switch
  the RRC attribute `ns3::NrGnbRrc::EpsBearerToRlcMapping` to AM if you ever want
  reliable delivery instead of loss visibility.

## Limitations

- **External-clock mode**: simulation time only advances when commands (with
  timestamps) arrive — that is the point of following the external clock, but it means
  a packet whose delivery/timeout falls beyond the current tick + `--drainMs` is
  resolved (row written, reply sent) only when a later command advances time past it.
  With NR latencies of 2–10 ms and the default 20 ms drain this only affects `timeout`
  outcomes (500 ms), which resolve a few ticks later.
- **Real-time overrun** (`--realtime` only): the 60 MHz / numerology-1 PHY with many
  UEs can be heavier than wall-clock time allows, especially in a non-optimized build
  profile. The simulator runs in best-effort mode: it never aborts, sim time just slips
  behind wall clock (CSV latencies, being pure sim time, stay correct; reply arrival
  gets delayed). Mitigations: keep `--maxUes` modest, set `--channelUpdateMs=100`, or
  build an optimized profile. The default external-clock mode has no such issue — a
  slow machine simply computes each tick more slowly.
- One control datagram must fit one UDP datagram (~64 kB — >300 entities per message).
- Malformed JSON datagrams are dropped (logged to stderr); no reply is sent for them.
- `Velocity`/`Heading` do not drive ns-3 mobility (positions come from the external
  simulator); the Doppler seen by the channel model is therefore that of a static node
  between updates.