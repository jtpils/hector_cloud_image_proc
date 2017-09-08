cmake_minimum_required(VERSION 2.8.3)
project(pcl_to_cv_proc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_geometry
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pcl_to_cv_proc
  CATKIN_DEPENDS
    cv_bridge
    eigen_conversions
    geometry_msgs
    image_geometry
    pcl_ros
    roscpp
    sensor_msgs
    tf
    tf_conversions
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/to_cv_depth_img.h
  src/to_cv_depth_img.cpp
  include/${PROJECT_NAME}/to_cv_intensity_img.h
  src/to_cv_intensity_img.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#install(TARGETS to_cv_depth_img
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
