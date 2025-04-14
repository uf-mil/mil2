import time
import imu_subscriber

imu_subscriber.run()

# Now monitor the data
while True:
    data = imu_subscriber.imu_node.imu_data
    if data:
        print(data)
    else:
        print("Waiting for IMU data...")
    time.sleep(0.5)
