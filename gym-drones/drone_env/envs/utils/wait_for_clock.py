#!/usr/bin/env python3
import sys
import argparse
import threading

from gz.transport14 import Node
import gz.msgs11.clock_pb2 as Clock


def main():
    p = argparse.ArgumentParser(
        description="Block until a minimum number of /clock messages have been received"
    )
    p.add_argument(
        "--min-msgs",
        "-n",
        type=int,
        default=10,
        help="Minimum number of /clock messages to wait for (default: 10)",
    )
    p.add_argument(
        "--timeout",
        "-t",
        type=float,
        default=30.0,
        help="Timeout in seconds (default: 30.0)",
    )
    args = p.parse_args()

    count = 0
    done = threading.Event()
    node = Node()

    def clock_cb(msg: Clock.Clock):
        nonlocal count
        count += 1
        if count >= args.min_msgs:
            done.set()

    if not node.subscribe(Clock.Clock, "/clock", clock_cb):
        print("[ERROR] Failed to subscribe to /clock", file=sys.stderr)
        sys.exit(1)

    if not done.wait(args.timeout):
        print(
            f"[ERROR] Timed out after {args.timeout}s (got {count} messages)",
            file=sys.stderr,
        )
        sys.exit(2)

    # success
    print(f"[INFO] Received {count} /clock messages; proceeding")
    sys.exit(0)


if __name__ == "__main__":
    main()
