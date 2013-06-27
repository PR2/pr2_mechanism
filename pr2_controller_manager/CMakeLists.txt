# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_controller_manager)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pr2_hardware_interface pr2_mechanism_model pr2_mechanism_diagnostics pr2_description pr2_controller_interface pr2_mechanism_msgs diagnostic_msgs sensor_msgs realtime_tools roscpp pluginlib)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(pr2_controller_manager src/controller_manager.cpp src/scheduler.cpp)

target_link_libraries(pr2_controller_manager ${Boost_LIBRARIES})

add_library(controller_test test/controllers/test_controller.cpp)

add_executable(test_controllers test/test.cpp)
add_executable(test_robot test/robot.cpp)

target_link_libraries(test_robot ${Boost_LIBRARIES} ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(test_controllers ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

#rosbuild_check_for_vm(vm)
#if(NOT vm)
   add_rostest(test/controller_test.launch)
#endif(NOT vm)


catkin_package(
    DEPENDS tinyxml
    CATKIN_DEPENDS pr2_hardware_interface pr2_mechanism_model pr2_mechanism_diagnostics pr2_description pr2_controller_interface pr2_mechanism_msgs diagnostic_msgs sensor_msgs realtime_tools roscpp pluginlib
)
