# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_controller_manager)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules pr2_hardware_interface pr2_mechanism_model pr2_mechanism_diagnostics pr2_description pr2_controller_interface pr2_mechanism_msgs diagnostic_msgs sensor_msgs realtime_tools roscpp pluginlib rostest)

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
   include 
   ${Boost_INCLUDE_DIRS} 
   ${catkin_INCLUDE_DIRS} 
   ${EIGEN3_INCLUDE_DIRS})

catkin_package(
    DEPENDS TinyXML
    CATKIN_DEPENDS pr2_hardware_interface pr2_mechanism_model pr2_mechanism_diagnostics pr2_description pr2_controller_interface pr2_mechanism_msgs diagnostic_msgs sensor_msgs realtime_tools roscpp pluginlib
    INCLUDE_DIRS include
    LIBRARIES pr2_controller_manager
)

add_definitions(-O3)

add_library(pr2_controller_manager src/controller_manager.cpp src/scheduler.cpp)

target_link_libraries(pr2_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})

add_library(controller_test test/controllers/test_controller.cpp)
pr2_enable_rpath(controller_test)
target_link_libraries(controller_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_controllers test/test.cpp)
add_executable(test_robot test/robot.cpp)

target_link_libraries(test_robot ${Boost_LIBRARIES} ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(test_controllers ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest(test/controller_test.launch)

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_python_setup()

install(TARGETS pr2_controller_manager controller_test
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_manager.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES test/controller_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
