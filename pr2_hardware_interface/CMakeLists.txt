cmake_minimum_required(VERSION 2.8.3)
project(pr2_hardware_interface)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs
    INCLUDE_DIRS include
    CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
