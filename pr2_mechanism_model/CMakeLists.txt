# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_mechanism_model)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp pr2_hardware_interface urdf kdl_parser pluginlib angles hardware_interface rostest rosunit)

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(urdfdom REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS})

catkin_package(
    DEPENDS TinyXML urdfdom
    CATKIN_DEPENDS roscpp pr2_hardware_interface urdf kdl_parser pluginlib angles hardware_interface
    INCLUDE_DIRS include
    LIBRARIES pr2_mechanism_model
)

add_definitions(-O3)

add_library( pr2_mechanism_model
  src/joint.cpp
  src/robot.cpp
  src/chain.cpp
  src/tree.cpp
  src/simple_transmission.cpp
  src/pr2_gripper_transmission.cpp
  src/wrist_transmission.cpp
  src/pr2_belt_transmission.cpp
  src/joint_calibration_simulator.cpp
)

find_package(PkgConfig)
pkg_check_modules(URDFDOM REQUIRED urdfdom)
if ("${URDFDOM_VERSION}" VERSION_EQUAL "1.0.0" OR "${URDFDOM_VERSION}" VERSION_GREATER "1.0.0")
  add_definitions(-DURDFDOM_1_0_0_API)
endif()

pr2_enable_rpath(pr2_mechanism_model)
target_link_libraries(pr2_mechanism_model ${catkin_LIBRARIES} )

catkin_add_gtest(test_chain test/test_chain.cpp)
target_link_libraries(test_chain pr2_mechanism_model ${catkin_LIBRARIES})

catkin_add_gtest(test_wrist_transmission test/test_wrist_transmission.cpp)
target_link_libraries(test_wrist_transmission pr2_mechanism_model ${catkin_LIBRARIES})

# test joint calibration simulator for writing actuator state optical flags
#  in propagatePositionBackwards() call
add_executable(test_joint_calibration_simulator 
   test/test_joint_calibration_simulator.cpp
   src/joint_calibration_simulator.cpp)
target_link_libraries(test_joint_calibration_simulator pr2_mechanism_model ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(test_joint_calibration_simulator gtest gtest_main)
add_dependencies(tests test_joint_calibration_simulator)
set_target_properties(test_joint_calibration_simulator PROPERTIES EXCLUDE_FROM_ALL TRUE)

add_rostest(test/test_joint_calibration_simulator.launch)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS pr2_mechanism_model
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES transmission_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
