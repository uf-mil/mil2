--To do:

-to obs space we need add sub localization - DVL sensor 
-integrate movement_publisher into sub env

-run localizaiton with the gymnode-enable ekf ndoe 
-call the unpause gazebo service before resetting
-increase gazbeo wait time to see if that changes anythign 
-launch file, put the localization stuff in there
-Make subRL a rospkg / submodule 
-see sub move in gazebo from random actions
-may change the range of motion from inf to smaller range


-LATER
-Reward calculation 



----AFTER FINISH SETTING UP
-Speed up sim
-opencv to detect two blobs drive between them, 
reward: distance from center  of camera for both white and red, would change based on this distance, 
white needs to be on the left and red needs to be on right
