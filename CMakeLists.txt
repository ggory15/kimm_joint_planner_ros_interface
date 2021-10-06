cmake_minimum_required(VERSION 3.0.2)
project(kimm_joint_planner_ros_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -Ofast)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

find_package(kimm_hqp_controller REQUIRED)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  plan_joint_path.srv
  action_joint_path.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs kimm_hqp_controller
 DEPENDS system_lib
)

link_directories(${Boost_LIBRARY_DIR})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${kimm_hqp_controller_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_HEADERS
  include/kimm_joint_planner_ros_interface/planner.h
)
SET(${PROJECT_NAME}_SOURCES
  src/planner.cpp
)

ADD_SOURCE_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
add_dependencies(${PROJECT_NAME} kimm_joint_planner_ros_interface_gencpp )

set(EXTRA_LIB ${EXTRA_LIB} ${Boost_LIBRARIES} ${kimm_trajectory_smoother_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${kimm_trajectory_smoother_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${EXTRA_LIB})

#ros_tutorial_srv_server
add_executable(${PROJECT_NAME}_server src/joint_planner_server.cpp)
target_link_libraries(${PROJECT_NAME}_server ${PROJECT_NAME} ${catkin_LIBRARIES} ${kimm_hqp_controller_LIBRARIES})
add_dependencies(${PROJECT_NAME}_server kimm_joint_planner_ros_interface_gencpp )

# #ros_tutorial_srv_client
# add_executable(${PROJECT_NAME}_client src/welding_planner_client.cpp)
# target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES} kimm_trajectory_smoother)
# add_dependencies(${PROJECT_NAME}_client kimm_welding_planner_ros_interface_gencpp )









