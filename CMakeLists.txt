PROJECT(ut_automata)
CMAKE_MINIMUM_REQUIRED(VERSION 3.6)

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Build Mode: ${CMAKE_BUILD_MODE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")
MESSAGE(STATUS "ROS_VERSION: ${ROS_VERSION}")

# SET(CMAKE_AUTOMOC ON)
# SET(CMAKE_AUTORCC ON)
# SET(CMAKE_AUTOUIC ON)

IF(CMAKE_VERSION VERSION_LESS "3.7.0")
    SET(CMAKE_INCLUDE_CURRENT_DIR ON)
ENDIF()

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

if(${ROS_VERSION} EQUAL "1")
  INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  ROSBUILD_INIT()
  SET(ROS_BUILD_STATIC_LIBS true)
  SET(ROS_BUILD_SHARED_LIBS false)
  ROSBUILD_GENMSG()
  SET(libs roslib roscpp glog gflags amrl_shared_lib rosbag X11 lua5.1 boost_system)
elseif(${ROS_VERSION} EQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(amrl_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CarStatusMsg.msg"
    "msg/VescState.msg"
    "msg/VescStateStamped.msg"
    DEPENDENCIES std_msgs
  )
  SET(libs glog gflags amrl_shared_lib X11 lua5.1)
endif()

FIND_PACKAGE(Qt5 COMPONENTS Core Widgets Gui WebSockets OpenGL REQUIRED)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules/config_reader/include)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared)

if(${ROS_VERSION} EQUAL "1")
  rosbuild_add_executable(vesc_driver
                          src/vesc_driver/serial.cc
                          src/vesc_driver/vesc_driver_node.cpp
                          src/vesc_driver/vesc_driver.cpp
                          src/vesc_driver/vesc_interface.cpp
                          src/vesc_driver/vesc_packet.cpp
                          src/vesc_driver/vesc_packet_factory.cpp)
  target_link_libraries(vesc_driver ${libs})

  ROSBUILD_ADD_EXECUTABLE(gui
        src/gui/gui_main.cc
        src/gui/gui_mainwindow.cc
        src/gui/vector_display.cc
        ${GUI_MOC_SRCS})
  TARGET_LINK_LIBRARIES(gui
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::OpenGL
        ${libs} ${OPENGL_LIBRARY} GL)

  ROSBUILD_ADD_EXECUTABLE(joystick
    src/joystick/joystick.cc
    src/joystick/joystick_driver.cc)
  TARGET_LINK_LIBRARIES(joystick ${libs})

  ROSBUILD_ADD_EXECUTABLE(simulator
      src/simulator/simulator.cc
      src/simulator/vector_map.cc
      src/simulator/simulator_main.cc)
  TARGET_LINK_LIBRARIES(simulator ${libs})

  ROSBUILD_ADD_EXECUTABLE(step_simulator
      src/simulator/simulator.cc
      src/simulator/vector_map.cc
      src/simulator/step_simulator_main.cc)
  TARGET_LINK_LIBRARIES(step_simulator ${libs})

  ROSBUILD_ADD_EXECUTABLE(websocket
      src/websocket/websocket_main_ros1.cc
      src/websocket/websocket.cc
  )
  TARGET_LINK_LIBRARIES(websocket
      Qt5::Core
      Qt5::Gui
      Qt5::Widgets
      Qt5::WebSockets
      ${libs})
elseif(${ROS_VERSION} EQUAL "2")
  ament_package()
endif()
