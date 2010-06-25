cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

rosbuild_add_library(pr2_controller_manager src/controller_manager.cpp src/scheduler.cpp)
rosbuild_remove_compile_flags(pr2_controller_manager -W)
rosbuild_link_boost(pr2_controller_manager thread)

rosbuild_add_library(controller_test test/controllers/test_controller.cpp)
rosbuild_add_executable(test_controllers test/test.cpp)
rosbuild_add_executable(test_robot test/robot.cpp)
rosbuild_link_boost(test_robot system)
target_link_libraries(test_robot ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_controllers)

rosbuild_check_for_vm(vm)
if(NOT vm)
  rosbuild_add_rostest(test/controller_test.launch)
endif(NOT vm)

