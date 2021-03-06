cmake_minimum_required(VERSION 2.8.3)
project(active_perception_controller)

set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs std_msgs nav_msgs roslib monarch_situational_awareness)

## System dependencies are found with CMake's conventions
find_package( Boost REQUIRED COMPONENTS python )
find_package( PythonLibs REQUIRED )
find_package( OpenCV REQUIRED )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ActivePerceptionPlan.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ap_utility
  CATKIN_DEPENDS message_runtime monarch_situational_awareness
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(ap_utility
   src/utility.cpp src/person_estimator.cpp src/particle_filter.cpp src/person_particle_filter.cpp src/rfid_sensor_model.cpp
)

add_library(active_perception_interface_lib src/active_perception_interface.cpp)

target_link_libraries(ap_utility gsl gslcblas ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

set_target_properties(ap_utility PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

## Declare a cpp executable
# add_executable(optimization_node src/optimization_node.cpp 
# 								 src/motion_planner.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(active_perception_controller_node active_perception_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(optimization_node
#                      gsl gslcblas ${catkin_LIBRARIES}
#                      ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES})

file(GLOB hdrs "include/active_perception_controller/*.h")

add_executable(person_estimator src/person_estimator.cpp src/particle_filter.cpp src/person_particle_filter.cpp src/rfid_sensor_model.cpp ${hdrs})
target_link_libraries(person_estimator gsl gslcblas ap_utility)
add_executable(person_estimator_coop src/person_estimator_coop.cpp src/particle_filter.cpp src/person_particle_filter.cpp src/rfid_sensor_model.cpp ${hdrs})
target_link_libraries(person_estimator_coop gsl gslcblas ap_utility)
add_executable(person_estimator_coop_sam src/person_estimator_coop_sam.cpp src/particle_filter.cpp src/person_particle_filter.cpp src/rfid_sensor_model.cpp ${hdrs})
target_link_libraries(person_estimator_coop_sam gsl gslcblas ap_utility)
add_executable(person_estimator_desc src/person_estimator_desc.cpp src/particle_filter.cpp src/person_particle_filter_desc.cpp src/rfid_sensor_model.cpp ${hdrs})
target_link_libraries(person_estimator_desc gsl gslcblas ap_utility)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS active_perception_controller active_perception_controller_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_active_perception_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
