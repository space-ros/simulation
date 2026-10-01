cmake_minimum_required(VERSION 3.8)
project(simulation)

if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO $ENV{ROS_DISTRO})
endif()

find_package(ament_cmake REQUIRED)

# The plugins are written against the Gazebo Harmonic API and build unchanged
# against Ignition Fortress through its gz/ redirect headers (see
# plugins/gz_compat.hh). Only the packages and link targets differ.
if(ROS_DISTRO STREQUAL "humble")
  # Ignition Fortress, from the system packages.
  find_package(ignition-plugin1 REQUIRED COMPONENTS register)
  find_package(ignition-gazebo6 REQUIRED)
  find_package(ignition-common4 REQUIRED COMPONENTS profiler)
  find_package(ignition-rendering6 REQUIRED)
  find_package(ignition-sensors6 REQUIRED)

  set(SIM_PLUGIN_LIB ignition-plugin${ignition-plugin1_VERSION_MAJOR}::ignition-plugin${ignition-plugin1_VERSION_MAJOR})
  set(SIM_SIM_LIB ignition-gazebo${ignition-gazebo6_VERSION_MAJOR}::core)
  set(SIM_COMMON_LIB ignition-common${ignition-common4_VERSION_MAJOR}::ignition-common${ignition-common4_VERSION_MAJOR})
  set(SIM_PROFILER_LIB ignition-common${ignition-common4_VERSION_MAJOR}::profiler)
  set(SIM_RENDERING_LIB ignition-rendering${ignition-rendering6_VERSION_MAJOR}::ignition-rendering${ignition-rendering6_VERSION_MAJOR})
  set(SIM_SENSORS_LIB ignition-sensors${ignition-sensors6_VERSION_MAJOR}::ignition-sensors${ignition-sensors6_VERSION_MAJOR})
else()
  # Gazebo Harmonic, through the ROS vendor packages so the plugins link
  # against the same libraries as ros_gz and gz_ros2_control.
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim REQUIRED)
  find_package(gz_plugin_vendor REQUIRED)
  find_package(gz-plugin REQUIRED COMPONENTS register)
  find_package(gz_common_vendor REQUIRED)
  find_package(gz-common REQUIRED COMPONENTS profiler)
  find_package(gz_rendering_vendor REQUIRED)
  find_package(gz-rendering REQUIRED)
  find_package(gz_sensors_vendor REQUIRED)
  find_package(gz-sensors REQUIRED)

  set(SIM_PLUGIN_LIB gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR})
  set(SIM_SIM_LIB gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR})
  set(SIM_COMMON_LIB gz-common${gz-common_VERSION_MAJOR}::gz-common${gz-common_VERSION_MAJOR})
  set(SIM_PROFILER_LIB gz-common${gz-common_VERSION_MAJOR}::profiler)
  set(SIM_RENDERING_LIB gz-rendering${gz-rendering_VERSION_MAJOR}::gz-rendering${gz-rendering_VERSION_MAJOR})
  set(SIM_SENSORS_LIB gz-sensors${gz-sensors_VERSION_MAJOR}::gz-sensors${gz-sensors_VERSION_MAJOR})
endif()

add_library(SolarPanelPlugin SHARED plugins/SolarPanelPlugin.cc)
target_link_libraries(SolarPanelPlugin PRIVATE
  ${SIM_SIM_LIB} ${SIM_PLUGIN_LIB} ${SIM_COMMON_LIB} ${SIM_RENDERING_LIB})

add_library(RadioisotopeThermalGeneratorPlugin SHARED plugins/RadioisotopeThermalGeneratorPlugin.cc)
target_link_libraries(RadioisotopeThermalGeneratorPlugin PRIVATE
  ${SIM_SIM_LIB} ${SIM_PLUGIN_LIB} ${SIM_PROFILER_LIB})

add_library(RechargeableBatteryPlugin SHARED plugins/RechargeableBatteryPlugin.cc)
target_link_libraries(RechargeableBatteryPlugin PRIVATE
  ${SIM_SIM_LIB} ${SIM_PLUGIN_LIB} ${SIM_COMMON_LIB} ${SIM_PROFILER_LIB})

add_library(SensorPowerSystemPlugin SHARED plugins/SensorPowerSystemPlugin.cc)
target_link_libraries(SensorPowerSystemPlugin PRIVATE
  ${SIM_SIM_LIB} ${SIM_PLUGIN_LIB} ${SIM_SENSORS_LIB})

set_property(TARGET SolarPanelPlugin RadioisotopeThermalGeneratorPlugin
  RechargeableBatteryPlugin SensorPowerSystemPlugin PROPERTY CXX_STANDARD 17)

# Create the models directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models/curiosity_path)

# Install directories
install(DIRECTORY
  models
  ${CMAKE_CURRENT_BINARY_DIR}/models
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS SolarPanelPlugin RadioisotopeThermalGeneratorPlugin
  RechargeableBatteryPlugin SensorPowerSystemPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
