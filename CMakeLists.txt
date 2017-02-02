cmake_minimum_required(VERSION 2.8.3)
project(sensor_pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  std_msgs
  nav_msgs
  geometry_msgs
 )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp mavros std_msgs nav_msgs geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/Node.h
    include/SensorPidNode.h
)

set(CPP_FILES
    src/Node.cpp
    src/SensorPidNode.cpp
)

add_executable(sensor_pid_node src/sensor_pid_node.cpp ${CPP_FILES})
add_dependencies(sensor_pid_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_pid_node ${catkin_LIBRARIES})
