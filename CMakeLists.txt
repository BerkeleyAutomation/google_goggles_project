cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(OpenCV REQUIRED)

#find_package(PCL REQUIRED)

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

#include_directories(/usr/include/pcl-1.5/)

include_directories(/usr/include/vtk-5.2/)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#common commands for building c++ executables and libraries
rosbuild_add_library(google_goggles_point_cloud
  src/tracker/cloud_ops.cpp
  src/tracker/plane_finding.cpp
  src/tracker/utils_cv.cpp
  src/tracker/utils_pcl.cpp
  src/tracker/tracker.cpp
  src/tracker/tracker_ros.cpp
  src/tracker/dist_math.cpp
  src/tracker/table.cpp
  src/tracker/config.cpp
  src/tracker/table_config.cpp
)
target_link_libraries(google_goggles_point_cloud ${OpenCV_LIBS} boost_program_options)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(build_ref_point_cloud src/build_ref_point_cloud.cpp)
#target_link_libraries(build_ref_point_cloud google_goggles_point_cloud)

#rosbuild_add_executable(icp_test src/icp_test.cpp)

#rosbuild_add_executable(convert_cloud src/convert_cloud.cpp)
#target_link_libraries(convert_cloud google_goggles_point_cloud)

#rosbuild_add_executable(process_ref_cloud src/process_ref_point_cloud.cpp)
#target_link_libraries(process_ref_cloud google_goggles_point_cloud)

rosbuild_add_executable(pose_estimator src/pose_estimator.cpp)
target_link_libraries(pose_estimator google_goggles_point_cloud)

include_directories(/usr/include/vtk-5.8)
rosbuild_add_executable(point_cloud_to_mesh src/point_cloud_to_mesh.cpp)
target_link_libraries(point_cloud_to_mesh google_goggles_point_cloud libvtkCommon.so)

#rosbuild_add_executable(image_file_publisher src/image_file_publisher.cpp)
