cmake_minimum_required(VERSION 3.0.2)
project(twist_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_ALIGN_STATICALLY)


set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# ASan 内存检测
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ASAN_FLAGS}")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  ti_mmwave_rospkg
  dvs_msgs
  cv_bridge
  tf
  eigen_conversions
  image_geometry
#   message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

SET(Ceres_DIR /home/hao/Downloads/ceres-solver-1.14.0/build/install/lib/cmake/Ceres)
find_package(Ceres 1.14 REQUIRED) 
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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

# add_message_files(
#   FILES
#   OpticalFlow.msg
#   OpticalFlowArray.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   dvs_msgs#   geometry_msgs#   sensor_msgs#   std_msgs
# )


# generate_messages(
#   DEPENDENCIES
#   dvs_msgs
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES radar-event
#  CATKIN_DEPENDS dvs_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs ti_mmwave_rospkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  front-end
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/radar-event.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/radar-event_node.cpp)

# add_executable(EventDetector src/EventDetector_node.cpp)
# add_executable(test_detector src/test/testEventDetector.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(EventDetector
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(test_detector
#   ${catkin_LIBRARIES}
# )

# file(GLOB_RECURSE event_flow_detector_file "front-end/event_flow_detector/*.cpp" "front-end/event_flow_detector/*.h")
# file(GLOB_RECURSE radar_ego_velocity_estimator_file "front-end/radar_ego_velocity_estimator/*.cpp" "front-end/radar_ego_velocity_estimator/*.h")
# add_library(twistdetector SHARED
#     ${event_flow_detector_file}
#     ${radar_ego_velocity_estimator_file}
# )

# add_executable(test_detector src/test/testEventDetector.cpp)

include_directories(
  src
  front-end
  back-end
  ${catkin_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  # ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

list(APPEND thirdparty_libraries
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        # ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        glog::glog
)

# 构建前端
add_subdirectory(front-end)

add_subdirectory(back-end)

add_executable(twist_estimator src/twist_node.cpp)
target_link_libraries(twist_estimator
    twistestimator
    twistdetector
    spline_lib
    ${thirdparty_libraries}
)

# 为 test_estimator 添加 ASan 选项
target_compile_options(twist_estimator PRIVATE -fsanitize=address -fno-omit-frame-pointer)
target_link_options(twist_estimator PRIVATE -fsanitize=address -fno-omit-frame-pointer)

# add_executable(twist_node
# src/twist_node.cpp
# src/twist_manager.h
# src/twist_manager.cpp
# )
# target_link_libraries(  
#   test_estimator
#   twistdetector
#   twistestimator
#   ${thirdparty_libraries}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radar-event.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
