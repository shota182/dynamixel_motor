cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_control)

##############################################################################
# 依存パッケージ
##############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Dynamixel SDK（apt 版なら /opt/ros/noetic/lib/pkgconfig/dynamixel_sdk.pc あり）
find_package(dynamixel_sdk REQUIRED)

##############################################################################
# コンパイルオプション
##############################################################################
add_compile_options(-std=c++14)

##############################################################################
# catkin パッケージ設定
##############################################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS dynamixel_sdk
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
)

##############################################################################
# 実行ファイル
##############################################################################
add_executable(multi_pos_ctrl_node src/multi_pos_ctrl.cpp)
target_link_libraries(multi_pos_ctrl_node ${catkin_LIBRARIES} ${dynamixel_sdk_LIBRARIES})
add_dependencies(multi_pos_ctrl_node ${catkin_EXPORTED_TARGETS})

add_executable(multi_pos_only_ctrl_node src/multi_pos_only_ctrl.cpp)
target_link_libraries(multi_pos_only_ctrl_node ${catkin_LIBRARIES} ${dynamixel_sdk_LIBRARIES})
add_dependencies(multi_pos_only_ctrl_node ${catkin_EXPORTED_TARGETS})

##############################################################################
# インストール
##############################################################################
# バイナリ
install(TARGETS multi_pos_ctrl_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS multi_pos_only_ctrl_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch ファイル
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##############################################################################
# テスト (必要なら)
##############################################################################
# if(CATKIN_ENABLE_TESTING)
#   ...
# endif()

