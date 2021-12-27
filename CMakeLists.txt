cmake_minimum_required(VERSION 3.0.2)
project(comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
        serial
)

catkin_package(
INCLUDE_DIRS src
LIBRARIES comm
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        comm
        src/main.cpp
)

target_link_libraries(comm ${catkin_LIBRARIES})