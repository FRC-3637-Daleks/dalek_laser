cmake_minimum_required(VERSION 3.0.2)
project(olarta)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  tf2_eigen
  tf2_ros
)


catkin_package(
CATKIN_DEPENDS roscpp std_msgs pcl_conversions pcl_ros laser_geometry tf2_eigen tf2_ros

#worked without laser_geometry but still works
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(olarta src/main.cpp)
target_link_libraries(olarta ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES})

