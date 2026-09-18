
cmake_minimum_required(VERSION 3.16)
project(simple_frame)

## Compile as C++17
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++17)

# ROS packages
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
             fusion_msgs
)

# PCL package
find_package(CGAL REQUIRED)
# VTK package
find_package(VTK COMPONENTS
vtkCommonCore
vtkCommonDataModel
vtkRenderingCore
vtkRenderingOpenGL2
)
# glog package
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)

# cmake include cgal Configuration Files
include( ${CGAL_USE_FILE} )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES record_odomlaser
    #  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
    #  DEPENDS system_lib
    )
    
    
include_directories(
	include
    ${GLOG_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    #${OCTOMAP_INCLUDE_DIRS}
)


aux_source_directory(src DIR_SRCS)

###### build ########
add_executable(simple_frame ${DIR_SRCS})

# Ensure message headers from fusion_msgs are generated before compiling.
add_dependencies(simple_frame ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_frame ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} glog)

# 
add_to_cached_list( CGAL_EXECUTABLE_TARGETS simple_frame )
target_link_libraries(simple_frame ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
