## ns-3 5G NR V2X bridge application

This repository contains an ns-3 application (V2X bridge) that works with [5G LENA](https://5g-lena.cttc.es/), to enable simulation of a 5G NR network with real messages coming from an external traffic/driving simulator (CARLA/OpenCDA, or a Python program like the provided test_harness.py). This enables the simulation of the MAC and PHY layers of a 5G network for Vehicle-to-Network (V2N) simulations, leaving the management of the higher layers to external applications or driving simulators. 

The v2x-bridge application can be installed as follows:
- Install ns-3 with 5G LENA following the instructions available here: [https://gitlab.com/cttc-lena/nr](https://gitlab.com/cttc-lena/nr); the V2X bridge has been tested with **ns-3.46** with **5g-lena-v4.1.1**. Even though it should work also with newer ns-3 versions, we recommend using ns-3.46.
- Move into the "scratch" directory, and clone the content of this repository: `git clone https://github.com/DriveX-devs/ns-3-v2x-bridge`
- Configure and build ns-3
- You can then run the V2X bridge following the instructions below

## Running the bridge

You can run the bridge as follows (we provide here a relevant example, with a gNB placed at (-42,25,10) meters and choosing 5555 as the port used to interface with the external application:
```sh
# gNB at the intersection centre, 32 UE slots
./ns3 run "v2x-bridge --gnbX=-42 --gnbY=25 --gnbZ=10 --maxUes=32 --listenPort=5555"
```

You can also test the bridge with an included test harness Python script, mimiking an external driving simulator like CARLA:
```sh
python3 scratch/v2x-bridge/test_harness.py --num 12 --shutdown
```

When running the bridge, you should expect the following output: 
- log lines on the terminal
- a summary such as `sent=12 delivered=12 timed_out=0 mean_latency=6.047 ms`
- two CSV logs (`v2x-send.csv`, `v2x-recv.csv`) written in the ns-3 root, containing information about sent and received packets

Some useful additional options are: `--realtime` (use real-time PC wall-clock instead of following an external clock reference stepping the ns-3 simulated time), `--injected-loss-perc` (drop a given percentage of packets on purpose), `--scenario` (to select the 3GPP channel scenario; by default `UMi` (Urban Microcell), but also `UMa` (Urban Macrocell) can be selected), `--verbose`.

## How it works

The bridge couples an external driving simulator (CARLA/OpenCDA) to a simulated 5G NR
network. The external process sends **JSON control datagrams** over a real OS UDP socket which port is defined by `--listenPort`; each UDP message should carry the current scene, i.e., vehicle/VRU positions keyed by a unique `origin_ID` for each actor, and, optionally, a descriptor of one packet to transmit (sender, receiver, size, base64 payload). The bridge moves the corresponding UEs and generates that packet, with the specified payload, **inside** the simulation, over the NR Uu interface. The UDP datagrams sent to ns-3 are commands, not tunnelled traffic: no *TapBridge* or *FdNetDevice* has been used.

When a packet is delivered, the payload extracted at the destination node is returned to the external application in a JSON reply, together with the measured one-way application-layer latency; when packets do not arrive after a given timeout (`--timeoutMs`, by default 500 ms) a failure is reported. Every transmission with its related outcome is logged to CSV.

By default the ns-3 clock is **managed by the external simulator**: simulation time is frozen while waiting for commands and advances exactly to the timestamp each command carries, so a paused or slower-than-real-time external simulator (such as CARLA) works without the need of any change. A UE pool is created and attached at *t* = 0 (5G-LENA cannot attach UEs later); each new `origin_ID` permanently claims a slot on first appearance.

The V2X bridge currently simulates the following radio configuration: band n78 (3.5 GHz), 60 MHz, numerology 1, TDD pattern `DL|S|UL|UL|DL|DL|S|UL|UL|DL|`, IPv4 only. However, it can be extended by editing the file `v2x-bridge.cc`.

## Used libraries

To manage the parsing of JSON messages, this project uses the _nlohmann json_ librar, available [here](https://github.com/nlohmann/json), and included as-is in this repository in the _json.hpp_ file.
This library is *not* licensed under the same license as the rest of the repository, and it retains its MIT license (see [here](https://github.com/nlohmann/json/blob/develop/LICENSE.MIT)).

## Disclaimer

Code development has been assisted by Claude Code with Claude Fable 5 and Claude Opus 5.
