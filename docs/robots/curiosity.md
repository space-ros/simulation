## Curiosity Mars Rover

The Curiosity Mars Rover is a robotic vehicle that was launched by NASA in 2011. It is a part of the Mars Science Laboratory mission and is designed to explore the Gale Crater on Mars. The rover is equipped with a variety of scientific instruments to study the Martian surface and environment. The Curiosity Mars Rover is one of the most advanced robotic vehicles ever built and has provided valuable data about the geology and climate of Mars.

### Scene Info

| Simple Environment                                             | Curiosity Mars Rover on Mars Surface                                                              |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| ![Simple Environment](../resources/images/curiosity-rover.png) | ![Curiosity Mars Rover on Mars Surface](../resources/images/mars-environment-curiosity-rover.png) |

  - The simple environment is available in the `./assets/Robots/Curiosity_Mars_Rover/CuriosityMarsRover.usd`.
  - The Curiosity Mars Rover on Mars Surface environment is available in the `./assets/Scenes/MarsEnvironment.usd`.

### Required extensions

  - ROS2 Bridge - To communicate with the Curiosity Mars Rover using ROS2.
  - Rover Simple Controller - To control the Curiosity Mars Rover using the keyboard.

### Action Graphs

The Curiosity Mars Rover has the following action graphs:

1. **IMU Publish Action Graph**. Available at Prim Path: `/curiosity_mars_rover/Graph/ROS_Imu`.
2. **Mobile Base Controller Action Graph**. Available at Prim Path: `/curiosity_mars_rover/Graph/MobilePlatformController`.
3. **Tool Arm Controller Action Graph**. Available at Prim Path: `/curiosity_mars_rover/Graph/ArmController`.
4. **Mast Arm Controller Action Graph**. Available at Prim Path: `/curiosity_mars_rover/Graph/MastArmController`.
5. **Odometry Publish Action Graph**. Available at Prim Path: `/curiosity_mars_rover/Graph/ROS_Odometry`.
6. **Camera Publish Action Graph**. Available at Prim Path: `/curiosity_mars_rover/Graph/ROS_Camera`.

The action graphs mentioned above are some of the common action graphs used across the Space ROS assets. The action graphs are used to publish the data from the Curiosity Mars Rover to the ROS2 bridge. The data includes the IMU data, odometry data, camera data, and the control commands for the mobile base, tool arm, and mast arm. You can find more information about the action graphs in the detailed [documentation](../action_graph.md).

### ROS2 Interface

The Curiosity Mars Rover publishes the following topics:

| Topic Name                                     | Message Type              | Description                                                                                                                             |
| ---------------------------------------------- | ------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `/curiosity/tf`                                | `tf2_msgs/TFMessage`      | The transformation data from the Curiosity Mars Rover. The message contains the transformation data.                                    |
| `/curiosity/imu`                               | `sensor_msgs/Imu`         | The IMU data from the Curiosity Mars Rover. The message contains the linear acceleration, angular velocity, and orientation.            |
| `/curiosity/odometry`                          | `nav_msgs/Odometry`       | The odometry data from the Curiosity Mars Rover. The message contains the position, orientation, linear velocity, and angular velocity. |
| `/curiosity/mast_camera/camera_info`           | `sensor_msgs/CameraInfo`  | The camera information from the Curiosity Mars Rover. The message contains the camera calibration data.                                 |
| `/curiosity/mast_camera/rgb`                   | `sensor_msgs/Image`       | The camera data from the Curiosity Mars Rover. The message contains the image data.                                                     |
| `/curiosity/mast_camera/depth`                 | `sensor_msgs/Image`       | The depth data from the Curiosity Mars Rover. The message contains the depth image data.                                                |
| `/curiosity/mast_camera/depth_pcl`             | `sensor_msgs/PointCloud2` | The point cloud data from the Curiosity Mars Rover. The message contains the point cloud data.                                          |
| `/curiosity/mast_camera/instance_segmentation` | `sensor_msgs/Image`       | The instance segmentation data from the Curiosity Mars Rover. The message contains the instance segmentation image data.                |
| `/curiosity/mast_camera/semantic_segmentation` | `sensor_msgs/Image`       | The semantic segmentation data from the Curiosity Mars Rover. The message contains the semantic segmentation image data.                |
| `/curiosity/mast_camera/semantic_labels`       | `std_msgs/String`         | The semantic labels from the Curiosity Mars Rover. The message contains the semantic labels.                                            |
| `/curiosity/arm/joint_command`                 | `sensor_msgs/JointState`  | The joint commands for the Curiosity Mars Rover's tool arm. The message contains the joint positions, velocities, and efforts.          |
| `/curiosity/mast_arm/joint_command`            | `sensor_msgs/JointState`  | The joint commands for the Curiosity Mars Rover's mast arm. The message contains the joint positions, velocities, and efforts.          |
| `/curiosity/cmd_vel`                           | `geometry_msgs/Twist`     | The velocity commands for the Curiosity Mars Rover's mobile base. The message contains the linear and angular velocities.               |
| `/clock`                                       | `rosgraph_msgs/Clock`     | The ROS clock message. The message contains the ROS time.                                                                               |

## Usage

1. Open IsaacSim.
2. Open the scene with Curiosity Mars Rover.
3. Press 'Play' button from the left panel to start the simulation.
4. You should now see the topic names in the terminal using the command as shown below:
```bash
ros2 topic list

# Output
# /clock
# /curiosity/arm/joint_command
# /curiosity/cmd_vel
# /curiosity/imu
# /curiosity/mast_arm/joint_command
# /curiosity/mast_camera/camera_info
# /curiosity/mast_camera/depth
# /curiosity/mast_camera/depth_pcl
# /curiosity/mast_camera/instance_segmentation
# /curiosity/mast_camera/rgb
# /curiosity/mast_camera/semantic_labels
# /curiosity/mast_camera/semantic_segmentation
# /curiosity/odometry
# /curiosity/tf
# /parameter_events
# /rosout
```

5. You can now publish the joint commands to the Curiosity Mars Rover's tool arm and mast arm using the `/curiosity/arm/joint_command` and `/curiosity/mast_arm/joint_command` topics respectively. You can also publish the velocity commands to the Curiosity Mars Rover's mobile base using the `/curiosity/cmd_vel` topic. There are examples provided in the demos repository for the same. You can find more information [here](https://github.com/space-ros/demos/tree/main/curiosity).