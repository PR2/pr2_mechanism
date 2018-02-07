# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_mechanism_diagnostics)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp urdf rospy diagnostic_updater diagnostic_msgs pr2_mechanism_msgs pr2_mechanism_model angles std_srvs std_msgs rostest)

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN REQUIRED eigen3)
include_directories(include ${PC_EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_definitions(${PC_EIGEN_CFLAGS_OTHER})

catkin_package()

add_definitions(-O3)

add_library(${PROJECT_NAME} 
  src/controller_diagnostics.cpp 
  src/joint_diagnostics.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(pr2_mechanism_diagnostics_node src/pr2_mechanism_diagnostics.cpp)
target_link_libraries(pr2_mechanism_diagnostics_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(pr2_mechanism_diagnostics_node PROPERTIES OUTPUT_NAME pr2_mechanism_diagnostics)

add_rostest(test/launch/mech_diag_test_main.launch)
add_rostest(test/launch/overrun_test.launch)
add_rostest(test/launch/uncal_test.launch)
add_rostest(test/launch/nan_test.launch)

install(TARGETS pr2_mechanism_diagnostics_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
