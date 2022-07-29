cmake_minimum_required(VERSION 3.0.2)
project(imhus_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs tf gazebo_plugins gazebo_ros message_runtime


)

# add_message_files(DIRECTORY msg
#   FILES
#   Action.msg
#   GoToAction.msg
#   WaitAction.msg
#   Goal.msg
#   CompoundTask.msg
#   Pose.msg
#   PoseVel.msg
#   publishAction.msg
# )


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf gazebo_plugins gazebo_ros message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(elevator_plugin src/elevator_plugin.cc)
target_link_libraries(elevator_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_library(actor_plugin src/actor_plugin.cc)
target_link_libraries(actor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
