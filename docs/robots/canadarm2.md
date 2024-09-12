## SSRMS Canadarm2

The Space Station Remote Manipulator System (SSRMS) is a robotic system used to move equipment and supplies around the International Space Station (ISS). The SSRMS is also known as Canadarm2, because it was built by the Canadian Space Agency.

The Canadarm2 is a large robotic arm that is attached to the ISS. It is used to move cargo, equipment, and astronauts around the station. The arm is controlled by astronauts inside the station, who use joysticks and computer screens to operate it.

### Scene Info

| Simple Environment                                       | Canadarm2 with ISS                                           |
| -------------------------------------------------------- | ------------------------------------------------------------ |
| ![Simple Environment](../resources/images/canadarm2.png) | ![Canadarm2 with ISS](../resources/images/canadarm2-iss.png) |

 - The simple environment is available in the `./assets/Robots/Canadarm2/SSRMS_Canadarm2.usd`.
 - The Canadarm2 with ISS environment is available in the `./assets/Scenes/InternationalSpaceStationCanadarm2.usd`.

### Required extensions

 - ROS2 Bridge - To communicate with the Canadarm2 using ROS2.

### Action Graphs

The canadarm2 has one action graph as presented below:

- **Canadarm2 Joint Control Action Graph**. The graph is available at Prim Path: `/SSRMS_Canadarm2/JointControlGraph`.

The above action graph pulls the joint states of the Canadarm2 and publishes them through the ROS2 bridge. You can also send joint states to the Canadarm2 using the same action graph. The same action graph is used to control the Canadarm2 in the simulation. You can find more information about the action graphs in the detailed [documentation](../action_graph.md).

### ROS2 Interface

| Topic Name                 | Message Type             | Description                                                                                              |
| -------------------------- | ------------------------ | -------------------------------------------------------------------------------------------------------- |
| `/canadarm2/joint_states`  | `sensor_msgs/JointState` | The joint states of the Canadarm2. The message contains the joint positions, velocities, and efforts.    |
| `/canadarm2/joint_command` | `sensor_msgs/JointState` | The joint commands for the Canadarm2. The message contains the joint positions, velocities, and efforts. |
| `/clock`                   | `rosgraph_msgs/Clock`    | The clock topic for the simulation.                                                                      |

## Usage

1. Open IsaacSim.
2. Open the scene with Canadarm2.
3. Press 'Play' button from the left panel to start the simulation.
4. You should now see the topic names in the terminal using the command as shown below:
```bash
ros2 topic list

# /canadarm2/joint_command
# /canadarm2/joint_states
# /clock
# /parameter_events
# /rosout
```
5. You can now publish the joint commands to the Canadarm2 using the `/canadarm2/joint_command` topic. There is an example provided in the demos repository for the same. You can find more information [here](https://github.com/space-ros/demos/tree/main/canadarm2).