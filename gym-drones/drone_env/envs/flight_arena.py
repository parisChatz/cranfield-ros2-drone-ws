import gymnasium as gym
import numpy as np
import cv2
from gz.transport14 import Node
from gz.msgs11.image_pb2 import Image
from gz.msgs11.twist_pb2 import Twist
import sys
import subprocess
import threading
from gz.msgs11.world_control_pb2 import WorldControl
from gz.msgs11.boolean_pb2 import Boolean


class DroneEnv(gym.Env):
    """
    Gymnasium environment for controlling a drone in Gazebo.
    Observations: camera frames.
    Actions: velocity commands (Twist).
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, max_episode_steps=500):
        super().__init__()

        # 1. Create a Gazebo transport node
        self.gz_node = Node()

        # 2. Threading primitives for non-blocking I/O
        self.lock = threading.Lock()
        self.action_event = threading.Event()
        self.stop_event = threading.Event()

        # 3. Internal state
        self.latest_image = None
        self.latest_obs = None
        self.step_count = 0
        self.max_episode_steps = max_episode_steps
        self.render_mode = render_mode
        self.world_name = "cranavgym_drone_sim"

        # 4. Spaces
        self.observation_space = gym.spaces.Box(
            low=0, high=255, shape=(160, 160, 3), dtype=np.uint8
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )

        # 5. Create publisher and subscriber
        self.cmd_pub = self.gz_node.advertise(
            topic="/x500/command/twist", msg_type=Twist
        )
        self.image_sub = self.gz_node.subscribe(
            topic="/x500/camera",
            callback=self._camera_callback,
            msg_type=Image,
        )

        # 6. Latest action buffer
        self.latest_action = np.zeros(4, dtype=np.float32)

        # 7. Start background publisher thread
        threading.Thread(target=self._publisher_loop, daemon=True).start()

    def _publisher_loop(self):
        """
        Background thread: waits for new actions and publishes them without blocking step().
        """
        twist = Twist()
        while not self.stop_event.is_set():
            # Wait until an action is set
            if not self.action_event.wait(timeout=0.005):
                continue
            with self.lock:
                action = self.latest_action.copy()
                self.action_event.clear()
            # Build and publish
            twist.linear.x = float(action[0])
            twist.linear.y = float(action[1])
            twist.linear.z = float(action[2])
            twist.angular.z = float(action[3])
            self.cmd_pub.publish(twist)

    def _camera_callback(self, msg: Image):
        """
        Background callback: stores latest image with thread-safety.
        """
        with self.lock:
            self.latest_image = msg.data

    def _get_observation(self):
        """
        Return the latest camera image (or a zero array if no image yet).
        """
        with self.lock:
            data = self.latest_image
        if data is None:
            return np.zeros((160, 160, 3), dtype=np.uint8)
        img_np = np.frombuffer(data, dtype=np.uint8).reshape(160, 160, 3)
        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        self.latest_obs = img_bgr
        return self.latest_obs

    def _calculate_reward(self):
        """
        Compute the reward based on the current state (obs).
        """
        return 0.0

    def reset(self, seed=None, options=None):
        """
        Reset the environment at the start of an episode.
        """
        super().reset(seed=seed)
        self.step_count = 0
        # Perform world reset synchronously
        self.world_reset()

        obs = self._get_observation()
        info = {}
        return obs, info

    def step(self, action):
        """
        Takes an action, signals publisher thread, and returns step result.
        """
        self.step_count += 1
        terminated = False

        # 1. Buffer action and notify publisher thread
        with self.lock:
            self.latest_action = np.array(action, dtype=np.float32)
        self.action_event.set()

        # 2. Get obs and reward
        obs = self._get_observation()
        reward = self._calculate_reward()

        # 3. Done flags
        truncated = self.step_count >= self.max_episode_steps
        goal_reached = False  # Placeholder
        collision = False  # Placeholder
        terminated = collision or goal_reached

        if terminated or truncated:
            print(
                f"terminated: {terminated}, truncated: {truncated}, reward: {reward}, step: {self.step_count}"
            )
        info = {}
        return obs, reward, terminated, truncated, info

    def render(self):
        pass
        # if self.render_mode == "human" and self.latest_obs is not None:
        #     cv2.imshow("Drone Camera View", self.latest_obs)
        #     cv2.waitKey(1)
        # elif self.render_mode == "rgb_array":
        #     return (
        #         self.latest_image
        #         if self.latest_image is not None
        #         else np.zeros((160, 160, 3), dtype=np.uint8)
        #     )

    def close(self):
        """
        Clean up: signal threads and shut down node.
        """
        self.stop_event.set()
        self.gz_node.shutdown()
        super().close()

    # --------------------------------------------------
    # Helper functions (world control)
    # --------------------------------------------------
    def pause_world(self):
        """Asynchronously pause the Gazebo world."""
        threading.Thread(
            target=world_control,
            args=(self.gz_node, "pause", self.world_name),
            daemon=True,
        ).start()

    def unpause_world(self):
        """Asynchronously unpause the Gazebo world."""
        threading.Thread(
            target=world_control,
            args=(self.gz_node, "unpause", self.world_name),
            daemon=True,
        ).start()

    def world_reset(self):
        """Perform world reset synchronously"""
        try:
            world_control(self.gz_node, "reset", self.world_name)
        except Exception as e:
            raise RuntimeError(f"World reset failed: {e}")


def world_control(
    node: Node,
    action: str,
    world_name: str = "cranavgym_drone_sim",
    timeout: int = 2000,
) -> None:
    """
    Perform a world control action ('reset', 'pause', 'unpause') via transport request.
    Raises RuntimeError on failure.
    """
    req = WorldControl()
    if action == "reset":
        req.reset.all = True
    elif action == "pause":
        req.pause.all = True
    elif action == "unpause":
        req.pause.all = False
    else:
        raise ValueError(f"Unknown world control action: {action}")

    service = f"/world/{world_name}/control"
    ok, resp = node.request(service, req, WorldControl, Boolean, timeout)
    if not ok or not resp.data:
        raise RuntimeError(
            f"World control '{action}' failed (ok={ok}, data={getattr(resp, 'data', None)})"
        )
