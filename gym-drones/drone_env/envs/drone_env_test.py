import gymnasium as gym
import numpy as np
import cv2
from gz.transport14 import Node
from gz.msgs11.image_pb2 import Image
from gz.msgs11.twist_pb2 import Twist


class DroneEnv(gym.Env):
    """
    Gymnasium environment for controlling a drone in Gazebo via ROS2.
    Observations: camera frames or other sensor data.
    Actions: velocity commands (Twist).
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, max_episode_steps=200):
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
        # Example placeholder: always return 0
        return 0.0

    def reset(self, seed=None, options=None):
        """
        Reset the environment at the start of an episode.
        Return (observation, info).
        """
        super().reset(seed=seed)
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
        # If we hit a max step limit, set truncated = True
        terminated = False
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
