cmake_minimum_required(VERSION 2.8.3)
project(vrpn_client_ros)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES vrpn_client_ros
  CATKIN_DEPENDS geometry_msgs tf2_ros
  DEPENDS VRPN EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall")

cs_add_library(vrpn_client_ros
  src/vrpn_client_ros.cpp
)


cs_add_executable(vrpn_tracker_node
  src/vrpn_tracker_node.cpp)


target_link_libraries(vrpn_tracker_node
  vrpn_client_ros
  ${catkin_LIBRARIES}
)

cs_add_executable(vrpn_client_node
  src/vrpn_client_node.cpp)

target_link_libraries(vrpn_client_node
  vrpn_client_ros
  ${catkin_LIBRARIES}
)

cs_add_executable(vrpn_client_pose_deal
  src/vrpn_client_pose_deal.cpp)

target_link_libraries(vrpn_client_pose_deal
  vrpn_client_ros
  ${catkin_LIBRARIES}
)

install(TARGETS vrpn_client_ros vrpn_tracker_node vrpn_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
  roslaunch_add_file_check(launch)
endif()

add_executable(trans_node
src/trans_cam_vicon.cpp
)
target_link_libraries(trans_node ${catkin_LIBRARIES})
