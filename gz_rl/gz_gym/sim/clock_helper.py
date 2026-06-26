import logging
import time

from gz.transport13 import Node

logger = logging.getLogger(__name__)


def wait_until_ready(node: Node, world_name: str, timeout_s: float = 30.0) -> bool:
    """Block until the WorldControl service is discoverable.

    Polls service_list() — a pure C++ call — rather than subscribing to /clock
    with a Python callback. Python subscription callbacks block gz.transport's
    dispatch thread while node.request() holds the GIL, causing a deadlock.
    Service discovery via service_list() is callback-free and GIL-safe.
    """
    control_svc = f"/world/{world_name}/control"
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if control_svc in node.service_list():
            return True
        time.sleep(0.1)
    logger.warning("Timed out waiting for %s after %.0fs", control_svc, timeout_s)
    return False


def wait_until_gui_ready(node: Node, world_name: str, timeout_s: float = 30.0) -> bool:
    """Block until the Gazebo GUI rendering engine is discoverable.

    Polls service_list() for /world/<world_name>/gui/info, which gz-sim
    registers once the GUI client has connected and initialized.  Same
    GIL-safe approach as wait_until_ready().
    """
    gui_svc = f"/world/{world_name}/gui/info"
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if gui_svc in node.service_list():
            return True
        time.sleep(0.1)
    logger.warning("Timed out waiting for GUI (%s) after %.0fs", gui_svc, timeout_s)
    return False
