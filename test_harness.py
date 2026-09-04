#!/usr/bin/env python3
"""Standalone test harness for the v2x-bridge ns-3 program.

This Python script plays the role of CARLA/OpenCDA (or a similar external application):
it sends JSON control datagrams (scene state + packet commands) to the bridge over real UDP,
asks for replies, and prints the JSON replies it receives. See DOCS.md for more details.

Usage (the ns-3 application must be already running):
    python3 test_harness.py --num 6 --shutdown
"""

import argparse
import json
import math
import socket
import sys
import time

# Sender/receiver pairs cycled by the harness: exercises UE->BS, BS->UE and
# UE->UE paths. "BS" is the reserved ID of the base-station/MEC endpoint.
PAIRS = [
    ("veh_1", "BS"),
    ("BS", "veh_1"),
    ("veh_1", "veh_2"),
    ("veh_2", "veh_1"),
    ("ped_1", "BS"),
    ("BS", "ped_1"),
]

# Entity catalogue (position is set per message, roughly circling the gNB).
ENTITIES = {
    "veh_1": {"type": "car", "radius": 60.0, "speed": 13.9},
    "veh_2": {"type": "truck", "radius": 90.0, "speed": 10.0},
    "ped_1": {"type": "pedestrian", "radius": 25.0, "speed": 1.25},
}


def entity_state(origin_id, step, ext_time):
    """Entity-state object in the agreed schema; entities orbit the origin in this simple test harness."""
    cfg = ENTITIES[origin_id]
    angle = 0.35 * step + hash(origin_id) % 7
    return {
        "timestamp": ext_time,
        "origin_ID": origin_id,
        "origin_vehicle_type": cfg["type"],
        "Position": {
            "x_m": cfg["radius"] * math.cos(angle),
            "y_m": cfg["radius"] * math.sin(angle),
            "z_m": 1.5,
        },
        "Velocity": cfg["speed"],
        "Heading": (angle + math.pi / 2) % (2 * math.pi),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1", help="bridge address")
    ap.add_argument("--port", type=int, default=5555, help="bridge control port")
    ap.add_argument("--num", type=int, default=6, help="number of packet commands")
    ap.add_argument("--interval", type=float, default=0.5, help="seconds between commands")
    ap.add_argument("--size", type=int, default=300, help="simulated packet size [bytes]")
    ap.add_argument("--timeout", type=float, default=2.0, help="reply wait timeout [s]")
    ap.add_argument("--first-id", type=int, default=1, help="first packet_id")
    ap.add_argument("--shutdown", action="store_true", help="send shutdown when done")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)
    dest = (args.host, args.port)

    sent = delivered = timed_out = 0
    latencies = []

    for i in range(args.num):
        sender, receiver = PAIRS[i % len(PAIRS)]
        packet_id = args.first_id + i
        ptype = "warning" if i % 3 == 2 else "detection"

        # Virtual external clock, like CARLA's fixed-delta simulation time:
        # one tick of `--interval` seconds per command. In the bridge's
        # default (external-clock) mode this timestamp is the time
        # reference; in --realtime mode it is ignored
        ext_time = i * args.interval

        entities = [entity_state(e, i, ext_time) for e in {sender, receiver} if e != "BS"]

        command = {
            "msg_type": "control",
            "timestamp": ext_time,
            "entities": entities,
            "packet": {
                "sender": sender,
                "receiver": receiver,
                "size_bytes": args.size,
                "packet_id": packet_id,
                "type": ptype,
                "request_reply": True,
            },
        }

        print(f"--> id={packet_id} {sender} -> {receiver} ({ptype}, {args.size} B)")
        sock.sendto(json.dumps(command).encode(), dest)
        sent += 1

        try:
            data, _ = sock.recvfrom(65536)
            reply = json.loads(data.decode())
            print("<--", json.dumps(reply))
            if reply.get("status") == "delivered":
                delivered += 1
                if reply.get("latency_ms") is not None:
                    latencies.append(reply["latency_ms"])
            elif reply.get("status") == "timeout":
                timed_out += 1
        except socket.timeout:
            print("<-- TIMEOUT waiting for reply", file=sys.stderr)

        time.sleep(args.interval)

    if args.shutdown:
        sock.sendto(json.dumps({"msg_type": "shutdown"}).encode(), dest)
        try:
            data, _ = sock.recvfrom(65536)
            print("<--", data.decode())
        except socket.timeout:
            pass

    print(f"\nsummary: sent={sent} delivered={delivered} timed_out={timed_out}", end="")
    if latencies:
        print(f" mean_latency={sum(latencies) / len(latencies):.3f} ms", end="")
    print()


if __name__ == "__main__":
    main()
