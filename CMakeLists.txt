CMAKE_MINIMUM_REQUIRED(VERSION 3.14.4)
PROJECT(ut_automata)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Build Mode: ${CMAKE_BUILD_MODE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# Set CMake policies
cmake_policy(SET CMP0071 NEW)

# Configure RPATH for proper library linking
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

IF(CMAKE_VERSION VERSION_LESS "3.7.0")
    SET(CMAKE_INCLUDE_CURRENT_DIR ON)
ENDIF()

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g -Wno-error=class-memaccess")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

message(STATUS "Building with ROS 2")
add_compile_definitions(ROS2)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(amrl_msgs REQUIRED)
find_package(amrl_maps REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Eigen3 REQUIRED)

FIND_PACKAGE(Qt5 COMPONENTS Core Widgets Gui WebSockets OpenGL REQUIRED)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "Build-Specific Libraries: ${BUILD_SPECIFIC_LIBRARIES}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules/config_reader/include)

# Generate ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  "msg/CarStatusMsg.msg"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs visualization_msgs nav_msgs
)

# Ensure generated message headers are visible to targets in this package
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Provide the rosidl fastrtps C++ typesupport target for linking against the
# generated type support library for this package's messages.
## Link helper that prefers the generated FastRTPS C++ typesupport target for
## this package, but falls back to the introspection C++ typesupport if the
## fastrtps target isn't available. The generated target names use the
## convention: <pkg>__rosidl_typesupport_fastrtps_cpp.
## Candidate package-specific rosidl typesupport targets. Use generator
## expressions so CMake will link whichever one was generated for this
## package (fastrtps, introspection, or the generic cpp typesupport).
set(UT_AUTOMATA_TYPESUPPORT_LINK
  $<TARGET_NAME_IF_EXISTS:${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp>
  $<TARGET_NAME_IF_EXISTS:${PROJECT_NAME}__rosidl_typesupport_introspection_cpp>
  $<TARGET_NAME_IF_EXISTS:${PROJECT_NAME}__rosidl_typesupport_cpp>
)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared)

SET(libs glog gflags amrl_shared_lib
    ${BUILD_SPECIFIC_LIBRARIES} X11 lua5.1 boost_system)

IF(${CMAKE_BUILD_MODE} MATCHES "Hardware")
  MESSAGE(STATUS "Building hardware drivers")

  add_executable(vesc_driver
                 src/vesc_driver/serial.cc
                 src/vesc_driver/vesc_driver_node.cpp
                 src/vesc_driver/vesc_driver.cpp
                 src/vesc_driver/vesc_interface.cpp
                 src/vesc_driver/vesc_packet.cpp
                 src/vesc_driver/vesc_packet_factory.cpp
                 src/vesc_driver/ekf_fusion.cpp
                 ${CMAKE_CURRENT_SOURCE_DIR}/../mpu6050driver/src/mpu6050sensor.cpp)
  ament_target_dependencies(vesc_driver rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs amrl_msgs rosidl_default_runtime tf2_ros Eigen3)
  target_include_directories(vesc_driver PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
    ${CMAKE_CURRENT_SOURCE_DIR}/../mpu6050driver/include)
  target_link_libraries(vesc_driver ${libs} ${UT_AUTOMATA_TYPESUPPORT_LINK} i2c)

  # Install hardware executables for ROS 2
  install(TARGETS
    vesc_driver
    DESTINATION lib/${PROJECT_NAME})
ENDIF()

add_executable(gui
      src/gui/gui_main.cc
      src/gui/gui_mainwindow.cc
      src/gui/vector_display.cc
      ${GUI_MOC_SRCS})
ament_target_dependencies(gui rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs amrl_msgs cv_bridge OpenCV rosidl_default_runtime)
target_include_directories(gui PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
TARGET_LINK_LIBRARIES(gui
      Qt5::Core
      Qt5::Gui
      Qt5::Widgets
      Qt5::OpenGL
      ${OpenCV_LIBS}
${libs} ${OPENGL_LIBRARY} GL ${UT_AUTOMATA_TYPESUPPORT_LINK})

add_executable(joystick
  src/joystick/joystick.cc
  src/joystick/joystick_driver.cc)
ament_target_dependencies(joystick rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs amrl_msgs rosidl_default_runtime)
target_include_directories(joystick PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
target_link_libraries(joystick ${libs} ${UT_AUTOMATA_TYPESUPPORT_LINK})

# Install common executables
install(TARGETS
  gui
  joystick
  DESTINATION lib/${PROJECT_NAME})

MESSAGE(STATUS "Building simulator")

add_executable(simulator
    src/simulator/simulator.cc
    src/simulator/vector_map.cc
    src/simulator/simulator_main.cc)
ament_target_dependencies(simulator rclcpp ament_index_cpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs amrl_msgs tf2 tf2_geometry_msgs tf2_ros rosidl_default_runtime)
target_include_directories(simulator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
target_link_libraries(simulator ${libs} ${UT_AUTOMATA_TYPESUPPORT_LINK})

add_executable(step_simulator
    src/simulator/simulator.cc
    src/simulator/vector_map.cc
    src/simulator/step_simulator_main.cc)
ament_target_dependencies(step_simulator rclcpp ament_index_cpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs amrl_msgs tf2 tf2_geometry_msgs tf2_ros rosidl_default_runtime)
target_include_directories(step_simulator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
target_link_libraries(step_simulator ${libs} ${UT_AUTOMATA_TYPESUPPORT_LINK})

add_executable(websocket
    src/websocket/websocket_main.cc
    src/websocket/websocket.cc
)
ament_target_dependencies(websocket rclcpp ament_index_cpp std_msgs geometry_msgs nav_msgs sensor_msgs amrl_msgs rosidl_default_runtime)
target_include_directories(websocket PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
target_link_libraries(websocket
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::WebSockets
  ${libs} ${UT_AUTOMATA_TYPESUPPORT_LINK})

# Install executables for ROS 2
install(TARGETS
  simulator
  step_simulator
  websocket
  DESTINATION lib/${PROJECT_NAME})

# ROS 2 package finalization
# Install launch files, config files, scripts, etc.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Install scripts directory with proper permissions
install(DIRECTORY
  scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
  OPTIONAL)

ament_export_dependencies(rosidl_default_runtime)

# ament_package() MUST be the last line
ament_package()
