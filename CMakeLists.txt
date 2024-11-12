cmake_minimum_required(VERSION 2.8.3)
project(SemLoop_data)

set(CMAKE_CXX_FLAGS "-std=c++17")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  dynamic_reconfigure
  rosbag
  cv_bridge
  image_transport
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  	include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
        ${rosbag_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(BEV_projection src/BEV_projection.cpp src/read.cpp)
target_link_libraries(BEV_projection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

