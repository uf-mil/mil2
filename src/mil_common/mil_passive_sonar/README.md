# mil_passive_sonar
This package includes the main scripts for receiving the location of a pinger. Location is published to a ros2 topic /hydrophones/solved.

# Running:
Run 'ros2 run mil_passive_sonar pipeline.sh' or 'ros2 run mil_passive_sonar fakeping_pipeline.sh' for testing with a simulated pinger.

Then run 'ros2 run mil_passive_sonar ping_publisher.py'

Now, you can listen to published messages via 'ros2 topic echo /hydrophones/solved'
