cmake_minimum_required(VERSION 2.8.3)
project(lidar_lane_detector)

add_compile_options(-std=c++11)

set(OpenCV_DIR /woonghyun/opencv/opencv-3.4.0/build/unix-install)

find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
  	pcl_ros
 	roscpp
	sensor_msgs
	std_msgs
	visualization_msgs
)

find_package(Boost REQUIRED)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES lidar_lane_detector
	CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs visualization_msgs
	DEPENDS system_lib
)

include_directories(
	include/lidar_lane_detector
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_library(lidar_lane_detector_lib SHARED
	src/lidar_lane_detector.cpp
	include/lidar_lane_detector/lidar_lane_detector.h
)

add_executable(lidar_lane_detector src/lidar_lane_detector_node.cpp)

target_include_directories(lidar_lane_detector_lib PRIVATE
	${Boost_INCLUDE_DIRS}
)

target_link_libraries(lidar_lane_detector_lib
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

target_include_directories(lidar_lane_detector PRIVATE
	include/lidar_lane_detector)

target_link_libraries(lidar_lane_detector
	lidar_lane_detector_lib)

