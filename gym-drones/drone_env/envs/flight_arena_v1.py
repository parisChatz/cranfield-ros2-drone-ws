import gymnasium as gym
import numpy as np
import cv2
import threading
import time
from gz.transport14 import Node
from gz.msgs11.image_pb2 import Image
from gz.msgs11.twist_pb2 import Twist
from gz.msgs11.world_control_pb2 import WorldControl
from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.odometry_pb2 import Odometry
import math


def world_control(
    node: Node, action: str, world_name: str, step_size: int = 100, timeout: int = 1000
):
    """
    Control Gazebo Harmonic world:
    - "reset": reset the world to its initial state (sdf).
    - "pause": pause physics updates.
    - "unpause": unpause physics updates.
    - "step": unpause, run `step_size` physics iterations, then re-pause.
    """

    req = WorldControl()
    if action == "reset":
        req.reset.all = True
        req.pause = True
    elif action == "pause":
        req.pause = True
    elif action == "unpause":
        req.pause = False
    elif action == "step":
        req.multi_step = step_size
    else:
        raise ValueError(f"Unknown world control action: {action}")

    svc = f"/world/{world_name}/control"
    ok, resp = node.request(svc, req, WorldControl, Boolean, timeout)

    # TODO this is not a good way to wait for the service to be ready
    print(f"[DBG] Service {svc}  with {action!r} returned ok={ok}, resp={resp}")
    return ok, resp


class Agent:
    def __init__(self, node: Node = None):
        if node is None:
            self.agent_node = Node()
        else:
            self.agent_node = node

        self._img_lock = threading.Lock()
        self._img_event = threading.Event()

        self.latest_image = None
        self.agent_node.subscribe(Image, "/x500/camera", self._cb_on_image)

        self._pos_lock = threading.Lock()
        self.latest_position = np.zeros(7, dtype=np.float32)
        self.agent_node.subscribe(Odometry, "/x500/odometry", self._cb_on_odometry)

        self._action_lock = threading.Lock()
        self.latest_action = np.zeros(4, dtype=np.float32)
        self.pub = self.agent_node.advertise("/x500/command/twist", Twist)

    # ——— Image callback + getter ———
    def _cb_on_image(self, msg: Image):
        with self._img_lock:
            self.latest_image = msg.data
            self._img_event.set()  # signal “we got an image”

    def wait_for_next_image(self, timeout=10.0):
        self._img_event.clear()
        got_one = self._img_event.wait(timeout)
        if not got_one:
            raise RuntimeError("Timed out waiting for image after reset")

    def get_observation(self) -> np.ndarray:
        with self._img_lock:
            data = self.latest_image
        if data is None:
            return np.zeros((160, 160, 3), dtype=np.uint8)
        img = np.frombuffer(data, dtype=np.uint8).reshape(160, 160, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # ——— Odometry callback + getter ———
    def _cb_on_odometry(self, msg: Odometry):
        p = msg.pose.position
        o = msg.pose.orientation
        with self._pos_lock:
            self.latest_position = np.array(
                [p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=np.float32
            )

    def get_position(self) -> np.ndarray:
        with self._pos_lock:
            return self.latest_position[0:3]

    def get_orientation(self) -> np.ndarray:
        with self._pos_lock:
            return self.latest_position[3:]

    def set_action(self, action: np.ndarray):
        with self._action_lock:
            msg = Twist()
            msg.linear.x = float(action[0])
            msg.linear.y = float(action[1])
            msg.linear.z = float(action[2])
            msg.angular.z = float(action[3])
            self.pub.publish(msg)


class GzInterface:
    def __init__(self, world_name: str = "cranavgym_drone_sim"):
        self.gz_node = Node()
        self.world_name = world_name
        self.agent = Agent()
        # self.agent = Agent(node=self.gz_node)
        self.req = WorldControl()

    def reset_world(self):
        threading.Thread(
            target=world_control,
            args=(self.gz_node, "reset", self.world_name),
            daemon=True,
        ).start()

        # If you want to block until the world is reset, comment the above and
        # uncomment the following lines:
        # req = WorldControl()
        # req.reset.all = True
        # svc = f"/world/{self.world_name}/control"
        # ok, resp = self.gz_node.request(svc, req, WorldControl, Boolean, 500)

    def pause_world(self):
        threading.Thread(
            target=world_control,
            args=(self.gz_node, "pause", self.world_name),
            daemon=True,
        ).start()

    def unpause_world(self):
        threading.Thread(
            target=world_control,
            args=(self.gz_node, "unpause", self.world_name),
            daemon=True,
        ).start()

    def step_world(self):
        # threading.Thread(
        #     target=world_control,
        #     args=(self.gz_node, "step", self.world_name),
        #     daemon=True,
        # ).start()
        req = WorldControl()
        req.multi_step = 100  # run 10 physics iterations
        req.pause = True  # pause after the step
        svc = f"/world/{self.world_name}/control"
        ok, resp = self.gz_node.request(svc, req, WorldControl, Boolean, 500)
        print(f"[DBG] Service MULTI STEP returned ok={ok}, resp={resp}")
        print(" ")


class DroneEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, max_episode_steps=500):
        super().__init__()
        self.render_mode = render_mode
        self.step_count = 0
        self.max_episode_steps = max_episode_steps

        self.gz = GzInterface()
        self.agent = self.gz.agent

        self.observation_space = gym.spaces.Box(
            0, 255, shape=(160, 160, 3), dtype=np.uint8
        )
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)

        self.goal_position = np.array([8.0, 0.0, 0.5], dtype=np.float32)
        self.goal_radius = 2

    def reset(self, **kwargs):
        self.step_count = 0
        self.gz.reset_world()
        # self.agent.wait_for_next_image()
        obs = self.agent.get_observation()
        return obs, {}

    def step(self, action):

        self.step_count += 1
        self.agent.set_action(action)

        self.gz.step_world()
        # self.gz.pause_world()

        obs = self.agent.get_observation()
        pos = self.agent.get_position()
        orientation = self.agent.get_orientation()

        dist = np.linalg.norm(pos - self.goal_position)
        goal_reached = dist < self.goal_radius

        # Count as "Collided" if orientation is too steep, i.e., if the drone is pitched or rolled bewond saving.
        collision = True if self.too_steep(orientation) else False

        # Count as "Collided" if the drone is outside the arena.
        collision = True if pos[0] < -10 or pos[0] > 10 else collision
        collision = True if pos[1] < -10 or pos[1] > 10 else collision

        reward = 1.0 if goal_reached else 0.0
        reward = reward + 0.01 if pos[2] > 0.6 and pos[2] < 1 else reward - 0.01
        reward = reward - 1.0 if collision else reward

        terminated = True if (goal_reached or collision) else False

        truncated = self.step_count >= self.max_episode_steps
        info = {}

        # if terminated or truncated:
        print(
            f"terminated={terminated}, truncated={truncated}, goal_reached={goal_reached}, collision={collision}, "
            f"step={self.step_count}, reward={reward}, distance={dist}, position={pos}, orientation={orientation}"
        )

        return obs, reward, terminated, truncated, info

    def render(self):
        return self.agent.get_observation()

    def close(self):
        self.gz.pause_world()
        super().close()

    def too_steep(self, quat):
        x, y, z, w = quat
        sinr = 2 * (w * x + y * z)
        cosr = 1 - 2 * (x * x + y * y)
        sinp = 2 * (w * y - z * x)
        TH = math.sqrt(2) / 2  # sin(45°)
        # pitch >45°?
        if abs(sinp) > TH:
            return True
        # roll  >45°?
        if abs(sinr) > abs(cosr):
            return True
        return False
