cmake_minimum_required(VERSION 2.8.3)
project(imhus_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roslib
  cohan_msgs
  pcl_ros
  cmake_modules
)

find_package(TinyXML REQUIRED)

add_service_files(
  FILES
  # PlaceRobot.srv
  # PlaceAgents.srv
)
add_message_files(
  FILES
  # PoseVel.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  cohan_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf cohan_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(placeAgents_pub_node src/placeAgents_pub_node.cpp)
# add_dependencies(placeAgents_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(placeAgents_pub_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# add_executable(placeRobot_pub_node src/placeRobot_pub_node.cpp)
# add_dependencies(placeRobot_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(placeRobot_pub_node ${catkin_LIBRARIES})

# add_executable(placeRobot_client_node src/placeRobot_client_node.cpp)
# add_dependencies(placeRobot_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(placeRobot_client_node ${catkin_LIBRARIES})

# add_executable(input_stage_node src/interface_stage_input_node.cpp)
# add_dependencies(input_stage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(input_stage_node ${catkin_LIBRARIES})

# add_executable(output_stage_node src/interface_stage_output_node.cpp)
# add_dependencies(output_stage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(output_stage_node ${catkin_LIBRARIES})

# add_executable(input_morse_node src/interface_morse_input_node.cpp)
# add_dependencies(input_morse_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(input_morse_node ${catkin_LIBRARIES})

# add_executable(output_morse_node src/interface_morse_output_node.cpp)
# add_dependencies(output_morse_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(output_morse_node ${catkin_LIBRARIES})

# add_executable(logManagerCohan_node src/logManagerCohan_node.cpp)
# add_dependencies(logManagerCohan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(logManagerCohan_node ${catkin_LIBRARIES})

# add_executable(agentsBridge_node src/agents_bridge.cpp)
# add_dependencies(agentsBridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(agentsBridge_node ${catkin_LIBRARIES})

#Olivier's part
# add_executable(input_gazebo_node src/interface_gazebo_input_node.cpp)
# add_dependencies(input_gazebo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(input_gazebo_node ${catkin_LIBRARIES})

# add_executable(output_gazebo_node src/interface_gazebo_output_node.cpp)
# add_dependencies(output_gazebo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(output_gazebo_node ${catkin_LIBRARIES})

# install
# foreach(dir maps launch/stage_launch)
#    install(DIRECTORY ${dir}/
#       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)
