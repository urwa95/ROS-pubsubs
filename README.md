# ROS-pubsub-cameracalib
Certainly! Here's a concise description, file structure, and build instructions for your ROS 2 package:

### Description:
This ROS 2 package contains two nodes for demonstrating camera calibration data exchange. The `CameraCalibrationPublisher` node publishes simulated camera calibration data, including dimensions, distortion model, and camera matrix. The `CameraCalibrationSubscriber` node subscribes to these messages and logs the received data. This setup exemplifies the publisher-subscriber model in ROS 2 using custom messages (`sensor_msgs/msg/CameraInfo`).

### File Structure:
```
your_package/
|-- src/
|   |-- camera_calibration_publisher.cpp
|   |-- camera_calibration_subscriber.cpp
|-- CMakeLists.txt
|-- package.xml
```

- **src/**: Contains the source files.
  - **camera_calibration_publisher.cpp**: Implements the publisher node.
  - **camera_calibration_subscriber.cpp**: Implements the subscriber node.
- **CMakeLists.txt**: CMake file for building the ROS 2 package.
- **package.xml**: Contains package information and dependencies.

### Build Instructions:
1. **Place the Package**: Copy your package directory into the `src` folder of your ROS 2 workspace.
   
2. **Compile the Package**:
   - Navigate to the root of your ROS 2 workspace.
   - Run `colcon build --packages-select your_package_name` to build your package. Replace `your_package_name` with the actual name of your package.
   - After building, source the setup script: `source install/setup.bash`.

3. **Run the Nodes**:
   - To run the publisher node: `ros2 run your_package_name camera_calibration_publisher`.
   - Open a new terminal, source the setup script again, and run the subscriber node: `ros2 run your_package_name camera_calibration_subscriber`.

4. **Verify Operation**: Observe the output in the subscriber node's terminal to ensure it is receiving and logging the data published by the publisher node.

This package is an excellent example for learning ROS 2 communication mechanisms, specifically focusing on custom message types and the publisher-subscriber pattern. It can be used as a foundation for more complex robotics applications involving sensor data processing and communication.
