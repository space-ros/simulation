# Space ROS IsaacSim Extension Package

This repository is an extension module for Nvidia's IsaacSim. It provides the following features:

 - Simulation Assets for Space Robotics
 - ROS2 Interface for Space Robotics
 - Extenstions for IsaacSim for Space Robotics

## System Requirements

The following are the system requirements to run the project:

| Requirement | Description                      |
| ----------- | -------------------------------- |
| OS          | Ubuntu 22.04                     |
| RTX GPU     | Nvidia RTX 30XX Series or higher |
| RAM         | 16 GB or higher                  |
| CPU         | Intel i7 or higher               |


## Prerequisites

The work is based on Nvidia's IsaacSim. The changes are tested on Ubuntu 22.04. The following dependencies are required to run the project:

| Dependency | Version  | Description                                                                                                                                                                         |
| ---------- | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| IsaacSim   | >=4.0.0  | Nvidia's IsaacSim. You can setup the simulation from official website. Instructions [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) |
| ROS2       | ==Humble | ROS2 Humble acts as a middle layer. This will be soon ported to the native project. Installation Instructions [here](https://docs.ros.org/en/humble/Installation.html)              |

## Get Started

1. Make sure the prerequisites are installed. If not, follow the instructions in the prerequisites section.
2. Clone the repository to your local machine in your desired location. Run the following command in the terminal:
```bash
git clone https://github.com/space-ros/simulation.git

# This project used git-lfs for large files. Make sure you have git-lfs installed.
# If not, you can install it by running the following command:
# sudo apt-get install git-lfs

# Now, you can pull the repository with git-lfs
cd simulation
git lfs pull
```
3. The repository contains two folders in the root directory:
    - `extensions`: Contains the extensions for IsaacSim
    - `assets`: Contains the ROS2 interface for IsaacSim
4. For easier access to the assetss, you can bookmark the assets folder in the IsaacSim by following the steps:
    1. Open IsaacSim.
    2. In the Context Browser, navigate to the root of the repository and right-click on the `assets` folder.
    3. Click on the `Add Bookmark` option.
    4. You will be prompted to enter the name of the bookmark. Enter "Space ROS Assets" and click on `OK`.
    5. Now, you can access the assets from the bookmarks section in the Context Browser.
5. For the extensions, you need to allow the extension manager of IsaacSim to know the location of the extensions. Follow the steps:
    1. Open IsaacSim.
    2. Go to `Windows` in the top menu bar and click on `Extensions`.
    3. Click on the hamburger icon on the top right corner of the Search bar in the extension manager and select `Settings`.
    4. On the right panel of the extension manager, find the section for extension search paths and click on the `+` icon at the end of the list.
    5. Navigate to the `extensions` folder in the repository and select the `extensions` folder.
    6. The extensions of Space ROS will now be available in the extension manager. You can search for the extensions in the search bar. The list of extensions is provided in the documentation below.
    7. Click on the extension and enable it by toggling the switch on the right side of the extension. You can also check the `Auto-Enable` option to enable the extension automatically when IsaacSim starts.
    8. You can repeat the steps for other extensions as well.
6. Now, you can start using the assets and extensions in the IsaacSim.

> NOTE: Make sure ROS2 Bridge is extension is enabled in the IsaacSim. You can enable it by the previous steps mentioned for the extensions.

## Documentation

This is a brief overview of the documentation provided in the repository. The detailed documentation can be found in the `docs` folder in the repository.

| Section                                                    | Description                                                                                                                                                                                                                                                                                                           |
| ---------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Space Robotics Environments](docs/assets/environments.md) | The environment assets provided by Space ROS for Nvidia's IsaacSim:<br>1. International Space Station<br>2. Interior of International Space Station<br>3. Lunar Surface<br>4. Mars Surface<br>5. Simple Chess Environment for Robonaut2<br>6. Simple Mars Replica Environment for Ingenuity Helicopter                |
| [Space Robotics Robots](docs/assets/robots.md)             | The robot assets provided by Space ROS for Nvidia's IsaacSIM:<br>[1. Curiosity Mars Rover](./docs/robots/curiosity.md)<br>2. Perseverance Mars Rover<br>[3.Ingenuity Helicopter V3](./docs//robots/ingenuity-helicopter.md)<br>[4.Canadarm2](./docs/robots/canadarm2.md)<br>[5.Robonaut2](./docs/robots/robonaut2.md) |
| [Space Robotics Scenes](docs/assets/scenes.md)             | The environment assets provided by Space ROS for Nvidia's IsaacSim:<br>1. International Space Station with Canadarm2<br>2. Curiosity Mars Rover on Mars Surface<br>3. Simple Chess Environment for Robonaut2<br>4. Simple Mars Replica Environment for Ingenuity Helicopter                                           |
| [ROS2 Interface](docs/ros2.md)                             | The ROS2 bridge and usage description for integration with Space ROS                                                                                                                                                                                                                                                  |
| [Extensions for IsaacSim](docs/extensions/index.md)        | The extensions provided by Space ROS for Nvidia's IsaacSim:<br>1. Example Omnigraph Extension for Space ROS<br>2. Omnigraph Extension for Rover Simple Mobile Base Controller                                                                                                                                         |