cmake_minimum_required(VERSION 3.0.2)
project(joy_to_dynamixel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# joy_position_control
add_executable(joy_to_extended_position_node
  src/joy_to_extended_position.cpp
)
target_link_libraries(joy_to_extended_position_node
  ${catkin_LIBRARIES}
)
add_dependencies(joy_to_extended_position_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# joy_position_control
add_executable(joy_position_control_node
  src/joy_position_control.cpp
)
target_link_libraries(joy_position_control_node
  ${catkin_LIBRARIES}
)
add_dependencies(joy_position_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# launchファイルのインストール
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# param/configファイルのインストール
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
