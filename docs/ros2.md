# ROS Bridge for IsaacSim

This document provides an overview of the ROS2 interface for the IsaacSim simulation environment. The ROS2 interface allows you to communicate with the IsaacSim simulation environment using ROS2 messages and services. You can use the ROS2 interface to send commands to the robots, receive sensor data, and interact with the simulation environment.

## Overview

You can refer to the official documentation for the ROS2 interface [here](https://docs.omniverse.nvidia.com/isaacsim/latest/features/external_communication/ext_omni_isaac_ros_bridge.html).

More information will be added soon.


## Usage

1. Make sure the ros2 bridge extension is enabled in IsaacSim.
2. All robot assets are equipped with omnigraphs that publish the data to the ROS2 bridge. You can find the action graphs for the robots in the `Graph` folder of the robot asset.
3. You can use the ROS2 interface to communicate with the robots using the topics and services provided by the ROS2 bridge.
4. You can find more information about the ROS2 interface in the official documentation [here](https://docs.omniverse.nvidia.com/isaacsim/latest/features/external_communication/ext_omni_isaac_ros_bridge.html).

### References
1. You can find more information on using ros2 bridge omnigraph nodes [here](./action_graph.md).