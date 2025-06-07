cmake_minimum_required(VERSION 3.0.2)
project(feedback_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  LIBRARIES feedback_control
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(feedback_control_node src/tension_feedback_control.cpp)
target_link_libraries(feedback_control_node ${catkin_LIBRARIES})

# Launchファイルをインストール
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
