import gymnasium as gym
import numpy as np
import cv2
from gz.transport14 import Node
from gz.msgs11.image_pb2 import Image
from gz.msgs11.twist_pb2 import Twist
import time

# Reset shit
from gz.transport14 import Node
from gz.msgs11.pose_pb2 import Pose
from gz.msgs11.boolean_pb2 import Boolean
import math
import time
import subprocess


def world_reset2(world_name: str = "cranavgym_drone_sim"):
    from gz.msgs11.world_control_pb2 import WorldControl
    from gz.msgs11.boolean_pb2 import Boolean

    reset_node = Node()

    # 1) Build your request
    req = WorldControl()
    req.reset.all = True

    # 2) Call `request`, passing:
    #    - topic
    #    - your WorldControl request msg
    #    - the WorldControl class as `request_type`
    #    - the Boolean class     as `response_type`
    #    - timeout in ms (e.g. 2000)
    result, response = reset_node.request(
        f"/world/{world_name}/control", req, WorldControl, Boolean, 100000
    )

    # 3) Check success and handle the Boolean reply
    if not result:
        raise RuntimeError("[ERROR] World reset request timed out")
    if not response.data:
        raise RuntimeError("[ERROR] World reset failed on the server side")
    print("[INFO] World reset successfully")


class DroneEnv(gym.Env):
    """
    Gymnasium environment for controlling a drone in Gazebo via ROS2.
    Observations: camera frames or other sensor data.
    Actions: velocity commands (Twist).
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, max_episode_steps=50):
        super().__init__()

        # 1. Create a Gazebo transport node (no rclpy or ROS2)
        self.gz_node = Node()

        # 2. Create a publisher for velocity commands
        self.cmd_pub = self.gz_node.advertise(
            topic="/x500/command/twist", msg_type=Twist  # The Gazebo twist topic
        )

        self.image_sub = self.gz_node.subscribe(
            topic="/x500/camera",  # The Gazebo camera topic
            callback=self._camera_callback,  # Your callback
            msg_type=Image,  # The Gazebo image message type
        )

        # 3. Gymnasium spaces:
        # Observation space (example: 84x84 grayscale image)
        self.observation_space = gym.spaces.Box(
            low=0, high=255, shape=(160, 160, 3), dtype=np.uint8
        )

        # Action space (example: [vx, vy, vz, yaw_rate], each in [-1, 1])
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4,),
            dtype=np.float32,
        )

        # 4. Internal variables
        self.latest_image = None
        self.latest_obs = None
        self.step_count = 0
        self.max_episode_steps = max_episode_steps

        # 5. Render mode handling (if you want to do live visualization)
        self.render_mode = render_mode

        # 1) Make your transport node once, then reuse it
        self.node = Node()

    def world_reset(self):
        self.move_model()

    def move_model(self):
        cmd = [
            "gz",
            "service",
            "-s",
            "/world/cranavgym_drone_sim/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--resptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'.format(
                name="x500", x=0.0, y=0.0, z=0.1
            ),
        ]
        subprocess.run(cmd)

    def move_model2(
        self,
        model_name: str,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        world_name: str = "cranavgym_drone_sim",
        timeout_ms: int = 5000,
    ):
        """
        Teleport `model_name` to the given pose in Gazebo/Harmonic.
        Rolls, pitches and yaws are in radians.

        Raises RuntimeError on timeout or server-side failure.
        """

        # give the discovery handshake a moment

        # 2) Build the Pose request
        req = Pose()
        req.name = model_name
        req.position.x = x
        req.position.y = y
        req.position.z = z

        # simple RPY→quaternion conversion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)

        req.orientation.x = qx
        req.orientation.y = qy
        req.orientation.z = qz
        req.orientation.w = qw

        # 3) Call the `/world/<world>/set_pose` service
        # note: Ignition CLI uses exactly this path :contentReference[oaicite:0]{index=0}
        result, response = self.node.request(
            f"/world/{world_name}/set_pose",
            req,
            Pose,  # request message type
            Boolean,  # response message type
            timeout_ms,
        )

        # 4) Check for errors
        if not result:
            raise RuntimeError("[ERROR] move_model request timed out")
        if not response.data:
            raise RuntimeError("[ERROR] move_model failed on the server side")

        print(
            f"[INFO] {model_name} moved to "
            f"({x:.2f}, {y:.2f}, {z:.2f}, r={roll:.2f}, p={pitch:.2f}, y={yaw:.2f})"
        )

    def _camera_callback(self, msg: Image):
        self.latest_image = msg.data

    def _get_observation(self):
        """
        Return the latest camera image (or a zero array if no image yet).
        """
        if self.latest_image is None:
            # Return a dummy frame if we haven't received one yet
            return np.zeros((160, 160, 3), dtype=np.uint8)

        # Make sure to shape it to (160,160,3) or consistent with observation_space
        img_np = np.frombuffer(self.latest_image, dtype=np.uint8).reshape(160, 160, 3)
        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

        self.latest_obs = img_bgr
        return self.latest_obs

    def _calculate_reward(self, obs):
        """
        Compute the reward based on the current state (obs).
        This is entirely dependent on your task (hover, navigate, etc.).
        """
        return 0.0

    def reset(self, seed=None, options=None):
        """
        Reset the environment at the start of an episode.
        Return (observation, info).
        """
        # super().reset(seed=seed)
        self.world_reset()
        self.step_count = 0

        # Possibly reset the simulation or drone pose in Gazebo
        # e.g., call a service to reset the world or reposition the drone

        obs = self._get_observation()
        info = {}
        return obs, info

    def step(self, action):
        """
        Takes an action, publishes velocity commands, steps simulation,
        and returns (observation, reward, terminated, truncated, info).
        """
        self.step_count += 1
        terminated = False

        # 1. Publish Twist
        twist_msg = Twist()
        twist_msg.linear.x = float(action[0])
        twist_msg.linear.y = float(action[1])
        twist_msg.linear.z = float(action[2])
        twist_msg.angular.z = float(action[3])
        self.cmd_pub.publish(twist_msg)

        obs = self._get_observation()
        reward = self._calculate_reward(obs)

        # 5. Determine if episode is terminated or truncated
        # For example, if the drone “crashed” or “landed,” set terminated = True
        collision = False
        if collision:
            pass

        # If we hit a max step limit, set truncated = True
        truncated = self.step_count >= self.max_episode_steps
        if terminated or truncated:
            print(
                f"terminated: {terminated}, truncated: {truncated}, reward: {reward}, step: {self.step_count}"
            )
        info = {}
        self.render()
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human" and self.latest_obs is not None:
            cv2.imshow("Drone Camera View", self.latest_obs)
            cv2.waitKey(1)
        elif self.render_mode == "rgb_array":
            return (
                self.latest_image
                if self.latest_image is not None
                else np.zeros((160, 160, 3), dtype=np.uint8)
            )

    def close(self):
        """
        Clean up the ROS2 node and any other resources.
        """
        cv2.destroyAllWindows()
        self.gz_node.shutdown()
        super().close()
