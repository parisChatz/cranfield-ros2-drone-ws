# wait_for_clock_helper.py

import threading
import time
import logging
from gz.transport14 import Node
import gz.msgs11.clock_pb2 as Clock


class ClockTimeoutError(Exception):
    pass


def wait_for_clock(min_msgs: int = 10, timeout: float = 30.0) -> int:
    """
    Block until we've seen `min_msgs` on /clock, or raise ClockTimeoutError.
    Returns the actual number of messages received.
    """
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
    )
    count = 0
    done_event = threading.Event()
    lock = threading.Lock()

    node = Node()

    def clock_cb(msg: Clock.Clock):
        nonlocal count
        with lock:
            count += 1
            logging.info(
                f"Received /clock msg #{count}: sim_time={msg.sim.sec}.{msg.sim.nsec}"
            )
            if count >= min_msgs:
                done_event.set()

    if not node.subscribe(Clock.Clock, "/clock", clock_cb):
        raise RuntimeError("Failed to subscribe to /clock")

    logging.info(f"Waiting for {min_msgs} /clock messages (timeout {timeout}s)…")
    if not done_event.wait(timeout=timeout):
        raise ClockTimeoutError(f"Timed out after {timeout}s ({count} msgs)")

    logging.info(f"Success: got {count} /clock messages")
    return count
