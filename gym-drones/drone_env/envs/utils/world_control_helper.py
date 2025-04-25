#!/usr/bin/env python3
import sys
import argparse
from gz.transport14 import Node
from gz.msgs11.world_control_pb2 import WorldControl
from gz.msgs11.boolean_pb2 import Boolean


def main():
    p = argparse.ArgumentParser(
        description="Pause/unpause or reset a Gazebo world via a fresh Node()"
    )
    p.add_argument("--world", default="cranavgym_drone_sim")
    p.add_argument("--timeout", type=int, default=2000)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--pause", action="store_true")
    group.add_argument("--unpause", action="store_true")
    group.add_argument("--reset", action="store_true")
    args = p.parse_args()

    node = Node()
    req = WorldControl()
    if args.pause:
        req.pause.all = True
    elif args.unpause:
        # Ignition sometimes uses `play`, but pause=false also works.
        req.pause.all = False
    else:  # reset
        req.reset.all = True

    ok, resp = node.request(
        f"/world/{args.world}/control", req, WorldControl, Boolean, args.timeout
    )
    if not ok or not resp.data:
        print("[ERROR] service call failed", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
