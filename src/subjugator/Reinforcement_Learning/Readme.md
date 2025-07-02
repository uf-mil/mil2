--This week
-defeat the ender dragon
- get random locations, use same location for obj distance and to calc reward
-reset rostime for the ekf to work
-Reward calculation

--Next week
-start real training 


--after finishes training
-Test the model 
-retrain the model
-

----AFTER FINISH SETTING UP
-opencv to detect two blobs drive between them,
reward: distance from center of camera for both white and red, would change based on this distance,
white needs to be on the left and red needs to be on right




-- To install the bullet engine
sudo apt install libgz-physics7-bullet

ros2 launch subjugator_RL rl.launch.py

ros2 service call /sub9_velocity_reset_plugin/reset_sub9_velocity std_srvs/srv/Empty