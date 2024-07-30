cmake_minimum_required(VERSION 3.0.2)
project(compare_ipc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system filesystem)



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
#  INCLUDE_DIRS include
#  LIBRARIES compare_ipc
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_pub_ros src/pointcloud_pub_ros.cpp)
target_link_libraries(pointcloud_pub_ros ${catkin_LIBRARIES})

add_executable(pointcloud_sub_ros src/pointcloud_sub_ros.cpp)
target_link_libraries(pointcloud_sub_ros ${catkin_LIBRARIES})


add_executable(pointcloud_pub_shared_mem src/pointcloud_pub_shared_mem.cpp)
target_link_libraries(pointcloud_pub_shared_mem ${catkin_LIBRARIES} Boost::system Boost::filesystem rt)

add_executable(pointcloud_sub_shared_mem src/pointcloud_sub_shared_mem.cpp)
target_link_libraries(pointcloud_sub_shared_mem Boost::system Boost::filesystem rt ${catkin_LIBRARIES})