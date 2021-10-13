cmake_minimum_required(VERSION 2.8.3)
project(lidar_detection)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)
find_package( OpenCV REQUIRED )

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable( lidar_detection src/lidar_detection.cpp )
target_link_libraries ( lidar_detection ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS lidar_detection
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

