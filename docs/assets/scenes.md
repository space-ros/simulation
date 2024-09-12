# Working with IsaacSim Scenes

Scenes are the core of the IsaacSim simulation environment. They are the virtual world where you can place robots, objects, and other assets to simulate various scenarios. This document provides an overview of how to work with scenes in IsaacSim.

You are already provided with a set of scenes in the IsaacSim. You can find the scenes in the directory `./assets/Scenes`. These scenes are simulation ready and can be used in the IsaacSim.


## List of Scenes

| Scene Name              | Description                                                     | Preview                                                                      |
| ----------------------- | --------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| ISS                     | International Space Station                                     | ![ISS](../resources/images/canadarm2-iss.png)                                |
| Robotnaut2              | Robotnaut2 in Manipulation Room                                 | ![Robotnaut2](../resources/images/robonaut2-chessboard-manipulation.png)     |
| Curiosity Rover in Mars | Mars Surface with Curiosity Rover for exploration related tasks | ![Mars](../resources/images/mars-environment-curiosity-rover.png)            |
| Ingenutiy Helicopter    | Ingenuity Helicopter in Simple Mars Replica Environment         | ![Ingenuity Helicopter](../resources/images/ingenuity-helicopter-simple.png) |


## Custom Scenes

You can create your own custom scenes using the IsaacSim Scene Editor. The Scene Editor allows you to create a new scene from scratch or modify an existing scene. You can add robots, objects, and other assets to the scene and configure their properties. The Scene Editor provides a visual interface to place and configure assets in the scene. The assets can be imported from the Asset Browser or created using the Asset Builder.

The available assets are listed here:
 - `./assets/Robots` - Contains robot models like Canadarm2, Robotnaut2, etc.
 - `./assets/Props` - Contains objects like earth, etc.
 - `./assets/Environments` - Contains environment models like Mars, ISS, etc.
 - `./assets/Scenes` - Contains the scenes that are simulation ready.

To creaste a custom scene, the simulation environment should have three components:

1. **Environment**: The environment is the background of the scene. It can be a room, a building, a planet, etc. The environment provides the context for the scene. You can choose from the available environment models or create a custom environment using the Asset Builder.
2. **Robot**: The robot is the main actor in the scene. It can be a robot arm, a mobile robot, a humanoid robot, etc. You can choose from the available robot models or create a custom robot using the Asset Builder.
3. **Physics**: The physics engine simulates the physical interactions between the objects in the scene. You can configure the physics properties like gravity, friction, etc. to simulate the real-world physics.
   1. **Gravity**: The gravity in the scene can be set to simulate the real-world gravity. You can set the gravity in the scene using the Scene Editor.
   2. **Friction**: The friction in the scene can be set to simulate the real-world friction. You can set the friction in the scene using the Scene Editor.
   3. **Collision**: The collision properties in the scene can be set to simulate the real-world collisions. You can set the collision properties in the scene using the Scene Editor.
   4. **Dynamics**: The dynamics properties in the scene can be set to simulate the real-world dynamics. You can set the dynamics properties in the scene using the Scene Editor.
