PROJECT(ut_automata)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Build Mode: ${CMAKE_BUILD_MODE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -Wno-stringop-truncation -Wno-deprecated-copy -g -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

find_package(Qt5 COMPONENTS Core Widgets Gui OpenGL WebSockets REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "Build-Specific Libraries: ${BUILD_SPECIFIC_LIBRARIES}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/submodules/config_reader/include)

ROSBUILD_GENMSG()

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared)

SET(libs roslib roscpp glog gflags amrl-shared-lib
    ${BUILD_SPECIFIC_LIBRARIES} rosbag X11 lua5.1 boost_system)

# node executable
IF(${CMAKE_BUILD_MODE} MATCHES "Hardware")
  MESSAGE(STATUS "Building hardware drivers")
  rosbuild_add_executable(vesc_driver
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
ENDIF()

MESSAGE(STATUS "Building simulator")

ROSBUILD_ADD_EXECUTABLE(simulator
    src/simulator/simulator.cc
    src/simulator/vector_map.cc
    src/simulator/simulator_main.cc)
TARGET_LINK_LIBRARIES(simulator ${libs})

ROSBUILD_ADD_EXECUTABLE(websocket
    src/websocket/websocket_main.cc
    src/websocket/websocket.cc
)
TARGET_LINK_LIBRARIES(websocket
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::WebSockets
    ${libs} ${OPENGL_LIBRARY} GL)
