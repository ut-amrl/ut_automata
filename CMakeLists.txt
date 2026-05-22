cmake_minimum_required(VERSION 3.16)
project(ut_automata)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_compile_options(-Wall -Wextra -Wpedantic)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-dangling-reference -Wno-unused-parameter)
endif()
add_compile_definitions(ROS2)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(amrl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenGL REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets Gui WebSockets OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(X11 REQUIRED)

find_library(GFLAGS_LIBRARY gflags REQUIRED)
find_library(GLOG_LIBRARY glog REQUIRED)
find_library(LUA_LIBRARY lua5.1 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarStatusMsg.msg"
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_subdirectory(src/shared)

set(UT_AUTOMATA_ROS_DEPS
  ament_index_cpp
  amrl_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

set(UT_AUTOMATA_COMMON_LIBS
  amrl_shared_lib
  ${Boost_SYSTEM_LIBRARY}
  ${GFLAGS_LIBRARY}
  ${GLOG_LIBRARY}
  ${LUA_LIBRARY}
  Threads::Threads
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/shared
  ${PROJECT_BINARY_DIR}
)

function(configure_ut_automata_target target)
  ament_target_dependencies(${target} ${UT_AUTOMATA_ROS_DEPS})
  target_link_libraries(${target}
    ${UT_AUTOMATA_COMMON_LIBS}
    ${cpp_typesupport_target}
  )
endfunction()

add_executable(simulator
  src/simulator/simulator.cc
  src/simulator/vector_map.cc
  src/simulator/simulator_main.cc
)
configure_ut_automata_target(simulator)

add_executable(step_simulator
  src/simulator/simulator.cc
  src/simulator/vector_map.cc
  src/simulator/step_simulator_main.cc
)
configure_ut_automata_target(step_simulator)

add_executable(websocket
  src/websocket/websocket_main.cc
  src/websocket/websocket.cc
  src/websocket/websocket.h
)
configure_ut_automata_target(websocket)
target_link_libraries(websocket
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::WebSockets
)

add_executable(vesc_driver
  src/vesc_driver/serial.cc
  src/vesc_driver/vesc_driver_node.cpp
  src/vesc_driver/vesc_driver.cpp
  src/vesc_driver/vesc_interface.cpp
  src/vesc_driver/vesc_packet.cpp
  src/vesc_driver/vesc_packet_factory.cpp
)
configure_ut_automata_target(vesc_driver)

add_executable(gui
  src/gui/gui_main.cc
  src/gui/gui_mainwindow.cc
  src/gui/gui_mainwindow.h
  src/gui/vector_display.cc
  src/gui/vector_display.h
)
configure_ut_automata_target(gui)
target_link_libraries(gui
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL
  OpenGL::GL
  ${X11_LIBRARIES}
)

add_executable(joystick
  src/joystick/joystick.cc
  src/joystick/joystick_driver.cc
)
configure_ut_automata_target(joystick)

install(TARGETS
  simulator
  step_simulator
  websocket
  vesc_driver
  gui
  joystick
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keyboard_teleop.py
  scripts/joystick_teleop.py
  scripts/find_available_utcs_host.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  assets
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  visualization.rviz
  web_rviz.html
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
