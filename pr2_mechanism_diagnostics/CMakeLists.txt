cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN REQUIRED eigen3)
include_directories(${PC_EIGEN_INCLUDE_DIRS})
add_definitions(${PC_EIGEN_CFLAGS_OTHER})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} 
  src/controller_diagnostics.cpp 
  src/joint_diagnostics.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} accumulators)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(pr2_mechanism_diagnostics_node src/pr2_mechanism_diagnostics.cpp)
target_link_libraries(pr2_mechanism_diagnostics_node ${PROJECT_NAME})
SET_TARGET_PROPERTIES(pr2_mechanism_diagnostics_node PROPERTIES OUTPUT_NAME pr2_mechanism_diagnostics)


rosbuild_add_rostest(test/launch/mech_diag_test_main.launch)
rosbuild_add_rostest(test/launch/overrun_test.launch)
rosbuild_add_rostest(test/launch/uncal_test.launch)
rosbuild_add_rostest(test/launch/nan_test.launch)
