import rclpy
import time
import imu_subscriber
import cam_subscriber
        
imu_subscriber.run()
cam_subscriber.run()

# Now monitor the data
while True:
    data = imu_subscriber.imu_node.imu_data
    data2 = cam_subscriber.cam_node.cam_data
    if data:
        print(data)
    else:
        print("Waiting for IMU data...")
    if data2 is not None:
        print(data2)
    else:
        print("Waiting for CAM data...")

    time.sleep(0.5)
