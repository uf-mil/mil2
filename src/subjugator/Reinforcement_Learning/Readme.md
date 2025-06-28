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




-- To install the bullet engine
sudo apt install libgz-physics7-bullet

ros2 launch subjugator_RL rl.launch.py

ros2 service call /sub9_velocity_reset_plugin/reset_sub9_velocity std_srvs/srv/Empty