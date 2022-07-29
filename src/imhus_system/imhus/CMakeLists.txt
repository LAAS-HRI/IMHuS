cmake_minimum_required(VERSION 3.0.2)
project(imhus)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  genmsg
  actionlib_msgs
  tf2_geometry_msgs
  actionlib
  roslib
  move_base_msgs
  tf
  cmake_modules
  pcl_ros
  # imhus_navigation
)

  find_package(TinyXML REQUIRED)

add_message_files(
  FILES
  AsyncAction_msgs.msg
  # Action.msg
  # GoToAction.msg
  # WaitAction.msg
  # Goal.msg
  # CompoundTask.msg
  # Pose.msg
  # PoseVel.msg
  # publishAction.msg
)

# add_service_files(
#   FILES
#   ComputePlan.srv
#   ActionBool.srv
# )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  move_base_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES imhus
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs actionlib_msgs move_base_msgs message_runtime tf2_geometry_msgs pcl_ros#imhus_navigation
#  DEPENDS system_lib TinyXML
 
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  imhusManager_node src/imhusManager_node.cpp 
  src/agents/human.cpp src/agents/robot.cpp src/agents/agent.cpp src/agents/entity.cpp src/agents/group.cpp
  src/agents/state/entityState.cpp src/agents/state/idleState.cpp src/agents/state/waitState.cpp src/agents/state/goToState.cpp src/agents/state/conflictState.cpp
  src/actions/lookAtAction.cpp src/actions/turningAction.cpp src/actions/goToAction.cpp
  src/actions/publishAction.cpp src/actions/asyncActionRes.cpp src/actions/asyncActionReq.cpp
  src/actions/waitAction.cpp src/actions/navigateAction.cpp src/actions/commandAction.cpp
  src/map/map.cpp src/map/object.cpp src/map/pose.cpp src/map/objects.cpp src/map/poses.cpp
  src/map/socialSpace.cpp src/map/socialSpaces.cpp src/map/socialPose.cpp
  src/imhus.cpp src/actions/compoundTask.cpp src/actions/compoundTasks.cpp
  src/scenario/scenarios.cpp src/scenario/scenario.cpp src/scenario/steps.cpp src/scenario/step.cpp
  )
add_dependencies(imhusManager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imhusManager_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

add_executable(boss_node src/bossUI_node.cpp) 
add_dependencies(boss_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(boss_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})


#all the ones below are unused anymore (surely)
# add_executable(humanBehaviorModel_node src/humanBehaviorModel_node.cpp src/types.cpp)
# add_dependencies(humanBehaviorModel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(humanBehaviorModel_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# add_executable(taskPlanner_node src/taskPlanner_node.cpp src/task.cpp)
# add_dependencies(taskPlanner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(taskPlanner_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# add_executable(logManager_node src/logManager_node.cpp)
# add_dependencies(logManager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(logManager_node ${catkin_LIBRARIES})

# add_executable(lowLevelController_node src/lowLevelController_node.cpp)
# add_dependencies(lowLevelController_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(lowLevelController_node ${catkin_LIBRARIES})


