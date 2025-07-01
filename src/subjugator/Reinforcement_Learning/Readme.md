--This week
-defeat the ender dragon
-get the velocity to 0 after reset
-change gym spaces to gymnasium spaces

--Next week
-start training
-may change the range of motion from inf to smaller range
-launch file, put the localization stuff in there

--LATER
-Make subRL a rospkg / submodule
-Reward calculation

----AFTER FINISH SETTING UP
-Speed up sim
-opencv to detect two blobs drive between them,
reward: distance from center of camera for both white and red, would change based on this distance,
white needs to be on the left and red needs to be on right

## Useful commands

ros2 launch subjugator_RL rl.launch.py

ros2 service call /sub9_velocity_reset_plugin/reset_sub9_velocity std_srvs/srv/Empty


## How to install bullet

# Add the OSRF GPG key
sudo curl https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add the APT source for Gazebo Harmonic
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update APT
sudo apt update

# Fix the expired ROS GPG key (if not done already)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Install
sudo apt install libgz-physics7-bullet