cmake_minimum_required(VERSION 3.5)
project(frc_basics)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ROS Dependencies
set(dependencies
  roscpp

  std_msgs
  sensor_msgs
  geometry_msgs
  frc_msgs
)

# find dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(wpilib REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${dependencies} cmake_modules
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  ${catkin_INCLUDE_DIRS} 
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/frc_basics.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/FmsInfo/
               include/DrivetrainInterface/
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${dependencies}
)

add_executable(fms_info src/FmsInfo/FmsInfoNode.cpp src/FmsInfo/FmsInfoClass.cpp src/FmsInfo/FmsInfoTable.cpp )
target_link_libraries(fms_info ntcore wpiutil ${catkin_LIBRARIES})
add_dependencies(fms_info ${catkin_EXPORTED_TARGETS})

add_executable(drivetrain_interface src/DrivetrainInterface/DrivetrainInterfaceNode.cpp src/DrivetrainInterface/DrivetrainInterfaceClass.cpp src/DrivetrainInterface/DrivetrainInterfaceTable.cpp)
target_link_libraries(drivetrain_interface ntcore wpiutil ${catkin_LIBRARIES})
add_dependencies(drivetrain_interface ${catkin_EXPORTED_TARGETS})

# ## Mark executables and/or libraries for installation
# install(TARGETS
#   fms_info
#   drivetrain_interface
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )


