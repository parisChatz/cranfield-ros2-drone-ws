#!/usr/bin/env python3
"""
Launch a ROS2+Gazebo simulation, wait until six /clock messages
have been published, then start RL training.
"""
import subprocess
import psutil
import threading
import sys
import signal
import os

from gz.transport14 import Node
import gz.msgs11.clock_pb2 as Clock

from stable_baselines3 import PPO
from drone_env.envs.flight_arena import DroneEnv

HEADLESS_SIM = False
MODEL_NAME = "x500_mono_cam"
_node = Node()


# --------------------------------------------------------------------------- #
#  Helpers
# --------------------------------------------------------------------------- #
def wait_for_clock(*, min_msgs: int = 10, timeout: float = 30.0):
    """
    Block until at least `min_msgs` /clock messages have been received,
    or raise TimeoutError after `timeout` seconds.
    """
    count = 0
    done_event = threading.Event()

    # Callback to be invoked on each /clock message
    def clock_cb(msg: Clock.Clock):
        nonlocal count
        count += 1
        if count >= min_msgs:
            done_event.set()

    # Subscribe to /clock
    if not _node.subscribe(Clock.Clock, "/clock", clock_cb):
        raise RuntimeError("Failed to subscribe to /clock")

    # Wait for enough messages or timeout
    if not done_event.wait(timeout):
        raise TimeoutError(
            f"Timed out waiting for {min_msgs} /clock messages (got {count})"
        )


def launch_ros2_sim(headless_sim: bool = True, model_name: str = "x500_mono_cam"):

    # 1) point explicitly at your system ROS2, not the one in the venv
    ROS2 = "/opt/ros/jazzy/bin/ros2"

    # 2) build the same launch command
    cmd = [
        ROS2,
        "launch",
        "my_drone_sim",
        "my_world_new.launch.py",
        f"headless:={'true' if headless_sim else 'false'}",
        f"model:={model_name}",
    ]

    # 3) copy env and strip out your venv’s bin directories so the child
    #    can only see system Python / ROS
    sim_env = os.environ.copy()
    paths = sim_env.get("PATH", "").split(os.pathsep)
    sim_env["PATH"] = os.pathsep.join(
        p for p in paths if ".venv" not in p and "gym-drones" not in p
    )
    # also drop any leftover Qt envs
    for var in ("QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"):
        sim_env.pop(var, None)

    return subprocess.Popen(cmd, env=sim_env)


def launch_gazebo_sim(
    sdf_path: str = "/home/paris/cranfield-ros2-drone-ws/src/my_drone_sim/worlds/flight_arena_objects.sdf",
    seed: int = 0,
) -> None:
    """
    Launches Ignition Gazebo (gz) simulation on the given SDF world file
    with the specified random seed.

    :param sdf_path: Path to your .sdf world file
    :param seed:     RNG seed for reproducibility
    :raises CalledProcessError: if gz exits with a non-zero status
    """
    cmd = ["gz", "sim", "--seed", str(seed), sdf_path]
    print(f"[INFO] Running: {' '.join(cmd)}")
    # This will block until the sim process exits
    subprocess.run(cmd, check=True)


def kill_process_tree(pid, sig=signal.SIGTERM, timeout=5.0):
    """
    Terminate all children of the given PID, then the PID itself.
    If they dont exit in `timeout` seconds, send SIGKILL.
    """
    try:
        parent = psutil.Process(pid)
    except psutil.NoSuchProcess:
        return
    children = parent.children(recursive=True)
    # send SIGTERM to children first
    for p in children:
        try:
            p.send_signal(sig)
        except psutil.NoSuchProcess:
            pass
    # then parent
    try:
        parent.send_signal(sig)
    except psutil.NoSuchProcess:
        pass

    # wait for processes to terminate
    gone, alive = psutil.wait_procs(children + [parent], timeout=timeout)
    if alive:
        # one last SIGKILL to anyone still around
        for p in alive:
            try:
                p.kill()
            except psutil.NoSuchProcess:
                pass


# --------------------------------------------------------------------------- #
#  Main Loop
# --------------------------------------------------------------------------- #
def main():
    sim_proc = None
    exit_code = 0
    try:
        # 1) start the sim as its own process
        # sim_proc = launch_ros2_sim(HEADLESS_SIM, MODEL_NAME)
        launch_gazebo_sim()
        # 2) wait for /clock
        wait_for_clock(min_msgs=10, timeout=30.0)
        print("[INFO] Simulation is ready - starting RL training")

        # # 3) train
        env = DroneEnv(render_mode="human")
        model = PPO("CnnPolicy", env, verbose=1, n_steps=100)
        model.learn(total_timesteps=2046, progress_bar=True)

    except TimeoutError as exc:
        print(f"[ERROR] {exc}")
        exit_code = 1
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        exit_code = 2
    finally:
        if sim_proc:
            # cleanly kill ros2-launch + all its children (gz-sim, gui, etc.)
            kill_process_tree(sim_proc.pid)
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
