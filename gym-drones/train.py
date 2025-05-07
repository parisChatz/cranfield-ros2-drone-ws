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

from stable_baselines3 import PPO
from drone_env.envs.flight_arena_v1 import DroneEnv
from drone_env.envs.utils.wait_for_clock_helper import wait_for_clock, ClockTimeoutError


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
    # # also drop any leftover Qt envs
    # for var in ("QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"):
    #     sim_env.pop(var, None)

    return subprocess.Popen(cmd, env=sim_env)


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
HEADLESS_SIM = False
MODEL_NAME = "x500_mono_cam"


def main():
    sim_proc = None
    exit_code = 0
    episode_max_steps = 2000

    try:
        # 1) start the sim as its own process
        sim_proc = launch_ros2_sim(HEADLESS_SIM, MODEL_NAME)
        # 2) wait for /clock
        n = wait_for_clock(min_msgs=20, timeout=5.0)
        print("[INFO] Simulation is ready - starting RL training" * 100)

        # # 3) train
        env = DroneEnv(render_mode="human", max_episode_steps=episode_max_steps)
        model = PPO("CnnPolicy", env, verbose=0)
        model.learn(total_timesteps=200046, progress_bar=True)

    except TimeoutError as exc:
        print(f"[ERROR] {exc}")
        exit_code = 1
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        exit_code = 2
    except ClockTimeoutError as e:
        print("Clock never started:", e)
    except Exception as e:
        print(f"[ERROR] Caught in main: {e}", file=sys.stderr)
    finally:
        if sim_proc:
            # cleanly kill ros2-launch + all its children (gz-sim, gui, etc.)
            print("[INFO] Shutting down simulation" * 100)
            kill_process_tree(sim_proc.pid)
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
