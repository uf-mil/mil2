# Calibrating Cameras
// TODO: Currently written by AI to quickly update to ROS2. Add more information later.

When using a camera, calibration is required to ensure that we are able to receive images with no distortion. If we receive distorted images, it may be more difficult to reconstruct a 3D scene from the camera. Examples of camera distortion and a more detailed process on calibrating cameras can be found on the [OpenCV website](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

Camera calibration should only need to be performed once per camera model, as the distortion produced by one camera should generally be the same as another camera of the same model. However, if two cameras of the same model have different settings, then calibrating both cameras individually may be required.

## Calibrating a New Camera

Upon receiving a new camera, follow the instructions below to calibrate the camera for ROS 2:

1. **Ensure Camera Detection:**  
   Confirm that your camera is connected and visible in ROS 2. You can check available topics by running:
   ```bash
   ros2 topic list
   ```

2. **Create a Launch File:**  
   Create a new ROS 2 launch file for the camera. For example, create a file named `camera_calibration.launch.py` with the following content:
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory
   import os

   def generate_launch_description():
       config_file = os.path.join(
           get_package_share_directory('your_camera_package'),
           'config',
           'camera_params.yaml'
       )
       return LaunchDescription([
           Node(
               package='usb_camera_driver',
               executable='usb_camera_node',
               name='camera',
               output='screen',
               parameters=[config_file],
               remappings=[('/image_raw', '/camera/image_raw')]
           )
       ])
   ```

3. **Run the Camera Calibration Tool:**  
   Launch the ROS 2 camera calibration tool by running:
   ```bash
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 --camera_name camera
   ```
   *Note: Modify `--size` (checkerboard dimensions) and `--square` (square size in meters) according to your calibration pattern.*

4. **Calibrate:**  
   A GUI will appear. Move a sturdy checkerboard in front of the camera until the GUI accumulates enough samples. Then click "Calibrate" to generate the camera's parameters.

5. **Save Calibration Parameters:**  
   Once calibration is successful, save the new calibration parameters to a YAML file. This file will be referenced by your launch file.

6. **Update the Launch File:**  
   Modify your camera launch file to include the path to the new calibration YAML file so that the camera node uses the correct parameters.

7. **Verify Calibration:**  
   Verify that calibration is successful by checking the **rectified image topic**:
   ```bash
   ros2 topic echo /camera/image_rect
   ```
   **Warning:** Only verify using the rectified image topic. The raw or color image topics may still display distortion.

## Additional Resources

For more details on camera calibration with ROS 2, refer to:
- [OpenCV Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- ROS 2 documentation on [Camera Calibration](https://docs.ros.org/en/galactic/Tutorials/Camera-Calibrations.html) *(replace with the appropriate version if necessary)*

:::{warning}
Ensure that you use the updated ROS 2 commands (`ros2 run` instead of `rosrun`), Python launch files instead of XML, and `get_package_share_directory()` instead of `$(find ...)` in your configuration.
:::
```