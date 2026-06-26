import logging
import os
import subprocess
import time
from typing import Optional

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.serialized_map_pb2 import SerializedStepMap
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node, NodeOptions

from gz_gym.sim.clock_helper import wait_until_ready, wait_until_gui_ready

logger = logging.getLogger(__name__)


class GzInterface:
    def __init__(
        self,
        world_name: str,
        world_file: str,
        gz_partition: str,
        headless: bool = True,
        step_size_s: float = 0.005,
    ) -> None:
        self._world_name = world_name
        self._world_file = world_file
        self._gz_partition = gz_partition
        self._headless = headless
        self._step_size_ns = int(step_size_s * 1_000_000_000)
        self._process: Optional[subprocess.Popen] = None
        self._control_service = f"/world/{world_name}/control"
        self._state_service = f"/world/{world_name}/state"

        opts = NodeOptions()
        opts.partition = gz_partition
        self._node = Node(opts)

    def launch(self) -> None:
        """Spawn gz sim paused and block until the WorldControl service is discoverable."""
        cmd = ["gz", "sim", "-v", "1"]
        if self._headless:
            cmd.append("-s")
            cmd.append("--headless-rendering")
        cmd.append(self._world_file)

        env = os.environ.copy()
        env["GZ_PARTITION"] = self._gz_partition

        self._process = subprocess.Popen(cmd, env=env)
        logger.info(
            "Launched gz sim pid=%d partition=%s world=%s",
            self._process.pid,
            self._gz_partition,
            self._world_file,
        )

        if not wait_until_ready(self._node, self._world_name):
            self.shutdown()
            raise RuntimeError(
                f"Gazebo did not become ready: "
                f"/world/{self._world_name}/control not found in time"
            )

        if not self._headless:
            if not wait_until_gui_ready(self._node, self._world_name):
                logger.warning(
                    "GUI service not found in time; window may still be loading"
                )

    def pause(self) -> None:
        req = WorldControl()
        req.pause = True
        ok, _ = self._node.request(
            self._control_service, req, WorldControl, Boolean, 5000
        )
        if not ok:
            raise RuntimeError("WorldControl pause service call failed")

    def unpause(self) -> None:
        req = WorldControl()
        req.pause = False
        ok, _ = self._node.request(
            self._control_service, req, WorldControl, Boolean, 5000
        )
        if not ok:
            raise RuntimeError("WorldControl unpause service call failed")

    def step(self, n_steps: int) -> None:
        """Advance the sim by exactly n_steps physics ticks, then block until done.

        Uses WorldControl.run_to_sim_time so Gazebo pauses at exactly
        current_sim_time + n_steps * step_size.  Requires pause() to have been
        called beforehand (per the D2 step protocol).
        """
        pre_ns = self._read_sim_time_ns()
        target_ns = pre_ns + n_steps * self._step_size_ns

        req = WorldControl()
        req.pause = False  # unpause so sim runs toward target
        req.run_to_sim_time.sec = target_ns // 1_000_000_000
        req.run_to_sim_time.nsec = target_ns % 1_000_000_000
        ok, _ = self._node.request(
            self._control_service, req, WorldControl, Boolean, 5000
        )
        if not ok:
            raise RuntimeError(f"WorldControl run_to_sim_time service call failed")

        # Poll until sim pauses at (or past) the target time.
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            ok, state = self._node.request(
                self._state_service,
                Empty(),
                Empty,
                SerializedStepMap,
                3,
            )
            if ok and state.stats.paused:
                t = state.stats.sim_time
                if t.sec * 1_000_000_000 + t.nsec >= target_ns:
                    return
            time.sleep(0.001)
        raise RuntimeError(
            f"step({n_steps}) timed out: sim_time never reached {target_ns}ns"
        )

    def reset_world(self) -> None:
        req = WorldControl()
        req.reset.all = True
        ok, _ = self._node.request(
            self._control_service, req, WorldControl, Boolean, 5000
        )
        if not ok:
            raise RuntimeError("WorldControl reset service call failed")

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def shutdown(self) -> None:
        if self.is_running():
            pid = self._process.pid
            self._process.terminate()
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
            logger.info("gz sim pid=%d terminated", pid)
        self._process = None

    def _read_sim_time_ns(self) -> int:
        ok, state = self._node.request(
            self._state_service, Empty(), Empty, SerializedStepMap, 5000
        )
        if not ok:
            raise RuntimeError("Failed to read sim_time from /world/state")
        t = state.stats.sim_time
        return t.sec * 1_000_000_000 + t.nsec

    def __enter__(self) -> "GzInterface":
        return self

    def __exit__(self, *_) -> None:
        self.shutdown()
