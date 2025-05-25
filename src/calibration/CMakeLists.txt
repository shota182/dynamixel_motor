cmake_minimum_required(VERSION 3.0.2)
project(calibration)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## 実行ファイル: 磁気センサキャリブレーションノード
add_executable(mag_calibration_node src/mag_calibration_node.cpp)
target_link_libraries(mag_calibration_node ${catkin_LIBRARIES})

add_executable(zero_point_calibration_node src/zero_point_calibration_node.cpp)
target_link_libraries(zero_point_calibration_node ${catkin_LIBRARIES})
add_dependencies(zero_point_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## インストール: 実行ファイル
install(TARGETS mag_calibration_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## インストール: パラメータファイル
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

## インストール: launchファイル
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
                 PATTERN "*.launch.xml"
)

## インストール: 参考用ソースコード（任意）
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
  FILES_MATCHING PATTERN "*.cpp"
                 PATTERN "*.h"
)
