cmake_minimum_required(VERSION 3.0.2)

project(fusion_recon)

## Compile as C++11
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++17)

# supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             pcl_ros
             roscpp
             rospy
             sensor_msgs
             std_msgs
             nav_msgs
             tf
             sensor_msgs
             filters
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_odomlaser
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    #${OCTOMAP_INCLUDE_DIRS}
)


aux_source_directory(src DIR_SRCS)
aux_source_directory(src/tools DIR_SRCS)

###### build ########
add_executable(fusion_recon ${DIR_SRCS})


target_link_libraries(fusion_recon ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
