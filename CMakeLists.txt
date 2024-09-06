cmake_minimum_required(VERSION 3.8)
project(simulation)
find_package(ament_cmake REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-common4 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGN_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

include_directories(${CMAKE_SOURCE_DIR}/plugins/rotor/include)

add_library(RotorPlugin SHARED 
  plugins/rotor/src/RotorPlugin.cpp 
  plugins/rotor/src/BladeFlappingModel.cpp 
  plugins/rotor/src/BladeForceModel.cpp 
  plugins/rotor/src/InflowModel.cpp 
  plugins/rotor/src/Matrix.cpp
)

target_link_libraries(RotorPlugin
  PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_SIM_VER}::ignition-gazebo${IGN_SIM_VER}
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models/curiosity_path)

install(DIRECTORY
  models
  ${CMAKE_CURRENT_BINARY_DIR}/models
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS RotorPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Include directories for AtmospherePlugin
include_directories(${CMAKE_SOURCE_DIR}/plugins/atmosphere/include)

# Add AtmospherePlugin library
add_library(AtmospherePlugin SHARED 
  plugins/atmosphere/src/AtmospherePlugin.cpp 
  plugins/atmosphere/src/AtmosphereModel.cpp 
)

# Link libraries for AtmospherePlugin
target_link_libraries(AtmospherePlugin
  PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_SIM_VER}::ignition-gazebo${IGN_SIM_VER}
)

install(TARGETS AtmospherePlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
