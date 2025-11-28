cmake_minimum_required(VERSION 3.8)
project(simulation)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-physics5 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-rendering6 REQUIRED)

# Set Gazebo plugin and sim versions
set(IGNITION_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(IGNITION_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
set(IGNITION_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGNITION_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})
set(IGNITION_SENSORS_VER ${ignition-sensors6_VERSION_MAJOR})

# Add the plugins directory to the include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Plugins)

# Add the SolarPanelPlugin library
add_library(SolarPanelPlugin SHARED
  plugins/SolarPanelPlugin.cc
)

# Link the SolarPanelPlugin library with required dependencies
target_link_libraries(SolarPanelPlugin
  PRIVATE ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
  PRIVATE ignition-common${IGNITION_COMMON_VER}::ignition-common${IGNITION_COMMON_VER}
  PRIVATE ignition-rendering${IGNITION_RENDERING_VER}::ignition-rendering${IGNITION_RENDERING_VER}
  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
)

# Add the RadioisotopeThermalGeneratorPlugin library
add_library(RadioisotopeThermalGeneratorPlugin SHARED
  plugins/RadioisotopeThermalGeneratorPlugin.cc
)

# Link the RadioisotopeThermalGeneratorPlugin library with required dependencies
target_link_libraries(RadioisotopeThermalGeneratorPlugin
  PRIVATE ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
)

# Add the RechargeableBatteryPlugin library
add_library(RechargeableBatteryPlugin SHARED
  plugins/RechargeableBatteryPlugin.cc
)

# Link the RechargeableBatteryPlugin library with required dependencies
target_link_libraries(RechargeableBatteryPlugin
  PRIVATE ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
  # PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGNITION_COMMON_VER}
)


# add the SensorPowerSystemPlugin library
add_library(SensorPowerSystemPlugin SHARED
  plugins/SensorPowerSystemPlugin.cc
)

# Link the SensorPowerSystemPlugin library with required dependencies
target_link_libraries(SensorPowerSystemPlugin
  PRIVATE ignition-plugin${IGNITION_PLUGIN_VER}::ignition-plugin${IGNITION_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::core
  PRIVATE ignition-gazebo${IGNITION_SIM_VER}::ignition-gazebo${IGNITION_SIM_VER}
  PRIVATE ignition-sensors${IGNITION_SENSORS_VER}::ignition-sensors${IGNITION_SENSORS_VER}
)


# Create the models directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models/curiosity_path)

# Install directories
install(DIRECTORY
  models
  ${CMAKE_CURRENT_BINARY_DIR}/models
  DESTINATION share/${PROJECT_NAME}/
)

# Install the plugin library
install(TARGETS SolarPanelPlugin RadioisotopeThermalGeneratorPlugin RechargeableBatteryPlugin SensorPowerSystemPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()