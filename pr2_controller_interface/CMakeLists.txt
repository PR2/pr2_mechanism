cmake_minimum_required(VERSION 2.8.3)
project(pr2_controller_interface)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  pr2_mechanism_model
  controller_interface
  )

catkin_package(
   CATKIN_DEPENDS roscpp pr2_mechanism_model controller_interface 
   INCLUDE_DIRS include
   )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
