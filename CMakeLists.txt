cmake_minimum_required(VERSION 3.0.2)
project(acoustics_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

## To declare and build messages, services or actions from within this
## package, follow the steps in the default cmakelists.txt file in a new 
## ROS project

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FloatArray.msg
  PointArray.msg
  SignalArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ComputeWaveforms.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES acoustics_ros
 CATKIN_DEPENDS rospy std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
catkin_install_python(
  PROGRAMS
  scripts/basic_test.py
  nodes/main_node.py  # required??
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )


## Mark other files for installation (e.g. launch and bag files, etc.)
## see http://wiki.ros.org/catkin/CMakeLists.txt#Installing_roslaunch_Files_or_Other_Resources
install(DIRECTORY res/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/res
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
message("sakdfh ${CATKIN_PACKAGE_SHARE_DESTINATION}")

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_acoustics_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
