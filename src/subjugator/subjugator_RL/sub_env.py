import gym
from gym import spaces
import numpy as np
from geometry_msgs.msg import Wrench, Vector3
from sensor_msgs.msg import Imu
import random
import time
import os
import threading
import subprocess
import imu_subscriber
import cam_subscriber
import os, subprocess, pathlib
from rclpy.executors import MultiThreadedExecutor
import rclpy
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist

# Shape of the image. L, W, # of channels
SHAPE = [50,80,3]

# ROS environment crashes in spanwed terminal without using this - cause by OpenCv depednencies
def clean_ros_env() -> dict:
    env = os.environ.copy()

    # 1) Remove only the variables that the opencv wheel polluted
    for v in ("QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH", "QML2_IMPORT_PATH"):
        env.pop(v, None)

    # 2) Strip every LD_LIBRARY_PATH component that contains "/cv2/"
    if "LD_LIBRARY_PATH" in env:
        env["LD_LIBRARY_PATH"] = ":".join(
            p for p in env["LD_LIBRARY_PATH"].split(":") if "/cv2/" not in p
        )

    
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    pathlib.Path(env["XDG_RUNTIME_DIR"]).mkdir(parents=True, exist_ok=True)

    return env

class SubEnv(gym.Env):   
    proc = None

    def __init__(self, render_mode = "rgb_array"):
        
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('gym_env_node')
        
        # Store initial pose for reference (not used for reset anymore)
        self.initial_pose = Pose()
        self.initial_pose.position.x = 0.0
        self.initial_pose.position.y = 0.0
        self.initial_pose.position.z = -1.0  # Adjust based on your submarine's starting depth
        self.initial_pose.orientation.w = 1.0
        
        # Run the launch file to start gazebo-- Runs only once
        self.proc = subprocess.Popen(
            [
                "gnome-terminal", "--",
                "bash", "-c",
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/mil2/install/setup.bash && "
                "ros2 launch subjugator_bringup gazebo.launch.py; exec bash"
            ],
            env=clean_ros_env(),
        )

        #creating IMU and Camera subscriber
        imu_node = imu_subscriber.ImuSubscriber()
        cam_node = cam_subscriber.CamSubscriber()

        exec_ = MultiThreadedExecutor(num_threads=2)   # Reduced to 2 threads
        exec_.add_node(imu_node)
        exec_.add_node(cam_node)
        exec_.add_node(self.node) #for the gym env

        spin_thread = threading.Thread(target=exec_.spin, daemon=True)
        spin_thread.start()

        # Wait for 25 seconds for gazebo to open
        time.sleep(25)

        #camera rgb space
        self.observation_space = spaces.Dict({
                'image'                 : spaces.Box(0, 255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=np.uint8),
                'orientation': spaces.Box(low=-1.0, high=1.0, shape=(4,)),
                'angular_velocity': spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                'linear_acceleration': spaces.Box(low=-np.inf, high=np.inf, shape=(3,)),
                'object_distance'       : spaces.Box(low = 0, high = 100, shape=(1,), dtype=np.float32),
            })

        #Force_x
        self.action_space = spaces.Dict( 
            {'force'  : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32),
             'torque' : spaces.Box(low=-50, high=50, shape=(3,), dtype=np.float32)
            })



    def _remove_submarine(self):
        """Remove submarine from Gazebo using gz service"""
        try:
            cmd = [
                'gz', 'service', 
                '-s', '/world/robosub_2024/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--req', 'name: "sub9", type: MODEL'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print("Successfully removed submarine from Gazebo")
                return True
            else:
                print(f"Failed to remove submarine: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print("Timeout while removing submarine")
            return False
        except Exception as e:
            print(f"Error removing submarine: {e}")
            return False

    def _spawn_submarine(self):
        """Spawn submarine back into Gazebo using gz service"""
        try:
            cmd = [
                'gz', 'service',
                '-s', '/world/robosub_2024/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--req', 'sdf_filename: "../simulation/subjugator_description/urdf/sub9.urdf.xacro"'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print("Successfully spawned submarine in Gazebo")
                return True
            else:
                print(f"Failed to spawn submarine: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print("Timeout while spawning submarine")
            return False
        except Exception as e:
            print(f"Error spawning submarine: {e}")
            return False

    def _reset_submarine_gazebo(self):
        """Reset submarine by removing and respawning it"""
        print("Resetting submarine using Gazebo services...")
        
        # Step 1: Remove the submarine
        if not self._remove_submarine():
            print("Failed to remove submarine, attempting fallback reset...")
            return False
        
        # Small delay to ensure removal is complete
        time.sleep(1)
        
        # Step 2: Spawn the submarine back
        if not self._spawn_submarine():
            print("Failed to spawn submarine, attempting fallback reset...")
            return False
        
        # Small delay to let physics settle
        time.sleep(2)
        
        return True



    def calculate_red_amount(self, image):
        if image is None:
            return 0, 0, 0
        
        # Extract red channel (first channel in RGB)
        red_channel = image[:, :, 0]
        
        # Calculate metrics
        total_red = np.sum(red_channel)
        avg_red = np.mean(red_channel)
        
        # Calculate percentage (255 is max value for each pixel)
        max_possible_red = 255 * image.shape[0] * image.shape[1]
        red_percentage = (total_red / max_possible_red) * 100
        
        return total_red, avg_red, red_percentage

    def calculate_reward(self, observation):
        image = observation['image']
        total_red, avg_red, red_percentage = self.calculate_red_amount(image)
        reward = red_percentage #trying to maximize the red percentage so that it goes to the red buoy
        return reward
        
    def _get_obs(self):
        # Get image from RL_subscriber through thread
        image = cam_subscriber.cam_node.cam_data
        
        # imu version of above based on imu_node --
        imu = imu_subscriber.imu_node.imu_data
        
        # Get object distance via the buoy_finder (either through subscriber thread, or pipe) --       
        object_distance = 10.0  # Placeholder - implement your distance calculation
        
        return {"image": image, "imu": imu, "object_distance": object_distance}
    
    def _get_info(self):
        print("Getting info")
        return {}

    def step(self, action):
        # Send action to the environment
        self._publish_action_as_wrench(action)

        # Get observation from the environment (especially object_distance)
        observation = self._get_obs()

        # Get extra info (mostly for debugging)
        info = self._get_info()

        reward = self.calculate_reward(observation)

        # Define termination
        terminated = False
        object_distance = observation.get('object_distance', float('inf'))
        if object_distance < 2:
            terminated = True
    
        return observation, reward, terminated, False, info
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Reset submarine by removing and respawning it
        success = self._reset_submarine_gazebo()
        
        if not success:
            print("Gazebo service reset failed!")
            # You could raise an exception here if you want the environment to fail
            # raise RuntimeError("Failed to reset submarine using Gazebo services")

        # Small delay to let physics settle
        time.sleep(2)

        observation = self._get_obs()
        info = self._get_info()

        return observation, info



    def _publish_action_as_wrench(self, action):
        force_action = action['force']
        torque_action = action['torque']
        wrench_msg = Wrench()

        wrench_msg.force = Vector3(
            x=float(force_action[0]),
            y=float(force_action[1]),
            z=float(force_action[2])
        )

        wrench_msg.torque = Vector3(
            x=float(torque_action[0]),
            y=float(torque_action[1]),
            z=float(torque_action[2])
        )

        with open("wrench_msg_pipe", 'wb') as pipe:
            print("Writing to wrench publisher")
            pipe.write(wrench_msg.SerializeToString())  # Fixed serialization

    def close(self):
        """Clean up resources"""
        if self.proc:
            self.proc.terminate()
            self.proc.wait()
        
        if hasattr(self, 'node'):
            self.node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    env = SubEnv()
    try:
        obs, info = env.reset()
        print("Environment reset successfully!")
        
        # Test with random actions
        # for i in range(10):
        #     action = {
        #         'force': np.random.uniform(-10, 10, 3),
        #         'torque': np.random.uniform(-5, 5, 3)
        #     }
        #     obs, reward, terminated, truncated, info = env.step(action)
        #     print(f"Step {i}: Reward = {reward}")
            
        #     if terminated:
        #         obs, info = env.reset()
        #         print("Episode terminated, reset environment")
                
    finally:
        env.close()