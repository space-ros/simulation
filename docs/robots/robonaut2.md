## Robonaut 2

The Robonaut 2 is a humanoid robot developed by NASA and General Motors. It is designed to assist astronauts with various tasks on the International Space Station (ISS). The robot is equipped with a variety of sensors and tools to help it perform tasks such as maintenance, repairs, and experiments.

There are two versions of the Robonaut 2: R2A and R2B. The R2A version was sent to the ISS in 2011 and was used for various experiments and demonstrations. The R2B version is a ground-based version of the robot that is used for research and development.

### Scene Info

| Simple Environment                                                | Robonaut 2 in Simple Chess Environment                                                               |
| ----------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| ![Simple Environment](../resources/images/robonaut-upperbody.png) | ![Robonaut 2 in Simple Chess Environment](../resources/images/robonaut2-chessboard-manipulation.png) |

  - The simple environment is available in the `./assets/Robots/Robonaut2/Robonaut2UpperBody.usd`.
  - The Robonaut 2 in Simple Chess Environment is available in the `./assets/Scenes/Robonaut2Manipulation.usd`.

### Required extensions

  - ROS2 Bridge - To communicate with the Robonaut 2 using ROS2.

### Action Graphs

The Robonaut 2 has the following action graphs:

1. **IMU Publish Action Graph**. Available at Prim Path: `/Robonaut2/Graph/IMUPublishGraph`.
2. **Joint State Publish Action Graph**. Available at Prim Path: `/Robonaut2/Graph/ROS_JointStates`.

The action graphs mentioned above are some of the common action graphs used across the Space ROS assets. The action graphs are used to publish the data from the Robonaut 2 to the ROS2 bridge. The data includes the IMU data and joint states. You can find more information about the action graphs in the detailed [documentation](../action_graph.md).

### ROS2 Interface

The Robonaut 2 publishes the following topics:

| Topic Name                 | Message Type             | Description                                                                                                        |
| -------------------------- | ------------------------ | ------------------------------------------------------------------------------------------------------------------ |
| `/robonaut2/imu`           | `sensor_msgs/Imu`        | The IMU data from the Robonaut 2. The message contains the linear acceleration, angular velocity, and orientation. |
| `/robonaut2/joint_states`  | `sensor_msgs/JointState` | The joint states from the Robonaut 2. The message contains the joint positions, velocities, and efforts.           |
| `/robonaut2/joint_command` | `sensor_msgs/JointState` | The joint commands for the Robonaut 2. The message contains the joint positions, velocities, and efforts.          |
| `/clock`                   | `rosgraph_msgs/Clock`    | The ROS clock message. The message contains the ROS time.                                                          |

## Usage

1. Open IsaacSim.
2. Open the scene with Robonaut 2 in Simple Chess Environment.
3. Press 'Play' button from the left panel to start the simulation.
4. You should now see the topic names in the terminal using the command as shown below:
```bash
ros2 topic list

# Output
# /clock
# /robonaut2/imu
# /robonaut2/joint_command
# /robonaut2/joint_states
# /rosout
# /parameter_events
```
