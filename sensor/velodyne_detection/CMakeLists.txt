cmake_minimum_required(VERSION 2.8.3)
project(velodyne_detection)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  nodelet
  sensor_msgs
  dynamic_reconfigure
  roslint
)
find_package( OpenCV REQUIRED )
find_package (PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/DetectionNode.cfg
)

catkin_package(CATKIN_DEPENDS
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

add_executable( ${PROJECT_NAME}_node
  src/detection.cpp
  src/detection_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg
)

target_link_libraries ( ${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
#   ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_nodelet
  src/detection.cpp
  src/detection_nodelet.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
#  ${PROJECT_NAME}
)


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  COMPONENT main
)

install(TARGETS ${PROJECT_NAME}_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

roslint_cpp()
