## Ingenuity Helicopter

The Ingenuity Helicopter is a small robotic helicopter that is part of the Mars 2020 mission. It is designed to demonstrate the technology of powered flight in the thin atmosphere of Mars. The helicopter is equipped with a camera and other sensors to help it navigate and explore the Martian surface.

The robot is available in the IsaacSim as a simulation asset. You can import the robot in your scene and use it for various tasks such as visual odometry, navigation, and exploration. The robot can be found at a location `./assets/Robots/Ingenuity_Helicopter`.

### Scene Info

| Simple Environment                                                  | Ingenuity Helicopter with Mars Replica Environment                                             |
| ------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| ![Simple Environment](../resources/images/ingenuity-helicopter.png) | ![Ingenuity Helicopter with Mars Surface](../resources/images/ingenuity-helicopter-simple.png) |

- The simple environment is available in the `./assets/Robots/Ingenuity_Helicopter/IngenuityV3.usd`.
- The Ingenuity Helicopter with Mars Surface environment is available in the `./assets/Scenes/SimpleIngenuityHelicopterScene.usd`.

### Required extensions

- ROS2 Bridge - To communicate with the Ingenuity Helicopter using ROS2.

### Action Graphs

The Ingenuity Helicopter has the following action graph as presented below:

 - **ROS IMU Action Graph**. The graph is available at Prim Path: `/Ingenuity/Graphs/ImuSensorGraph`.
 - **Downward Camera Action Graph**. The graph is available at Prim Path: `/Ingenuity/Graphs/DownFacingCamera`.

The above action graphs pull the sensor data from the Ingenuity Helicopter and publish them through the ROS2 bridge. You can also send commands to the Ingenuity Helicopter using the same action graphs. The same action graphs are used to control the Ingenuity Helicopter in the simulation. You can find more information about the action graphs in the detailed [documentation](../action_graph.md).

### ROS2 Interface

| Topic Name                          | Message Type             | Description                                                                                                                       |
| ----------------------------------- | ------------------------ | --------------------------------------------------------------------------------------------------------------------------------- |
| `/ingenuity_helicopter/imu`         | `sensor_msgs/Imu`        | The IMU sensor data of the Ingenuity Helicopter. The message contains the linear acceleration, angular velocity, and orientation. |
| `/ingenuity_helicopter/camera_info` | `sensor_msgs/CameraInfo` | The camera info topic for the Ingenuity Helicopter.                                                                               |
| `/ingenuity_helicopter/rgb`         | `sensor_msgs/Image`      | The raw image topic for the Ingenuity Helicopter.                                                                                 |
| `/clock`                            | `rosgraph_msgs/Clock`    | The clock topic for the simulation.                                                                                               |

## Usage

1. Open IsaacSim.
2. Open the scene with Ingenuity Helicopter.
3. Press 'Play' button from the left panel to start the simulation.
4. You should now see the topic names in the terminal using the command as shown below:
```bash
ros2 topic list

# /ingenuity_helicopter/imu
# /ingenuity_helicopter/camera_info
# /ingenuity_helicopter/rgb
# /clock
# /parameter_events
# /rosout
```

5. You can now subscribe to the topics and visualize the data from the Ingenuity Helicopter. There is an example provided in the demos repository for the same. You can find more information [here](https://github.com/space-ros/demos/tree/main/ingenuity-helicopter).

