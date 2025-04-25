from gz.transport14 import Node
from gz.msgs11.world_control_pb2 import WorldControl
from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.entity_factory_pb2 import EntityFactory
from scipy.spatial.transform import Rotation
import time
from pathlib import Path
import os
import subprocess
import random

# This file contains helper functions to interact with the Gazebo transport layer
__all__ = ["world_reset", "spawn_entity", "launch_sim"]

_node = Node()
# TODO wrong way to handle nodes. Try subprocesses.


def spawn_entity(
    *,
    model_uri: str = "model://x500_mono_cam",
    name: str = "x500",
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    world: str = "cranavgym_drone_sim",
) -> None:

    # --------------------------------------------------
    if model_uri.startswith("model://"):
        model_name = model_uri[len("model://") :]
        sdf_path = None
        # check any GZ_SIM_RESOURCE_PATH entries
        for root in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep):
            cand = Path(root) / model_name / "model.sdf"
            if cand.is_file():
                sdf_path = cand
                break
        if sdf_path is None:
            raise FileNotFoundError(f"[ERROR] Could not find SDF for '{model_uri}'")
    else:
        # treat model_uri as an existing SDF path
        if not Path(model_uri).is_file():
            raise FileNotFoundError(f"[ERROR] SDF file not found: {model_uri}")
    # --------------------------------------------------
    # 1) Build your request
    req = EntityFactory()
    req.sdf_filename = model_uri
    req.name = name
    req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z

    # Convert Euler angles to quaternion
    quat = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()

    (
        req.pose.orientation.y,
        req.pose.orientation.x,
        req.pose.orientation.z,
        req.pose.orientation.w,
    ) = (quat[0], quat[1], quat[2], quat[3])

    # 2) Call `request`
    result, response = _node.request(
        f"/world/{world}/create", req, EntityFactory, Boolean, 2000
    )

    if not result:
        raise TimeoutError("[ERROR] Spawn service timed out")
    if not response.data:
        raise RuntimeError("[ERROR] Spawn service reported failure")

    print(f"[INFO] Agent {name} spawned successfully")


def spawn_many_agents():
    raise NotImplementedError("Function is not implemented yet")


if __name__ == "__main__":

    time.sleep(2.0)
    spawn_entity(
        model_uri="model://goal",
        name="goal26",
        x=5,
        y=0,
        z=0.0,
    )
    time.sleep(2.0)
    spawn_entity(
        model_uri="model://red_box",
        name="goal3",
        x=5,
        y=5,
        z=0.0,
    )
    time.sleep(2.0)
    spawn_entity(
        model_uri="model://green_box",
        name="goal4",
        x=0,
        y=5,
        z=0.0,
    )

    time.sleep(1.0)
