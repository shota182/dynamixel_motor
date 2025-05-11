cmake_minimum_required(VERSION 3.0.2)
project(bilateral_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(position_follow_1d_node src/position_follow_1d_node.cpp)
target_link_libraries(position_follow_1d_node ${catkin_LIBRARIES})

add_executable(position_follow_2d_node src/position_follow_2d_node.cpp)
target_link_libraries(position_follow_2d_node ${catkin_LIBRARIES})

add_executable(bilateral_2d_node src/bilateral_2d_node.cpp)
target_link_libraries(bilateral_2d_node ${catkin_LIBRARIES})

# === 以下を追加 ===

# launch ファイルのインストール
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# config (YAML) ファイルのインストール
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
