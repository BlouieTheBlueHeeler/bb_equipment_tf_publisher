cmake_minimum_required(VERSION 3.0.2)
project(bb_equipment_tf_publisher)

set(CMAKE_VERBOSE_MAKEFILE On)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(catkin_DIR "$ENV{ROS_ROOT}/../../share/catkin/cmake")
#set(sensor_msgs_DIR "$ENV{ROS_ROOT}/../../share/sensor_msgs/cmake")
#set(geometry_msgs_DIR "$ENV{ROS_ROOT}/../../share/geometry_msgs/cmake")
#set(message_runtime_DIR "$ENV{ROS_ROOT}/../../share/message_runtime/cmake")
#set(cpp_common_DIR "$ENV{ROS_ROOT}/../../share/cpp_common/cmake")
#set(rosconsole_DIR "$ENV{ROS_ROOT}/../../share/rosconsole/cmake")
#set(roscpp_DIR "$ENV{ROS_ROOT}/../../share/roscpp/cmake")
#set(roscpp_serialization_DIR "$ENV{ROS_ROOT}/../../share/roscpp_serialization/cmake")
#set(roscpp_traits_DIR "$ENV{ROS_ROOT}/../../share/roscpp_traits/cmake")
#set(xmlrpcpp_DIR "$ENV{ROS_ROOT}/../../share/xmlrpcpp/cmake")
#set(rostime_DIR "$ENV{ROS_ROOT}/../../share/rostime/cmake")
#set(std_msgs_DIR "$ENV{ROS_ROOT}/../../share/std_msgs/cmake")
#set(tf2_DIR "$ENV{ROS_ROOT}/../../share/tf2/cmake")
#set(tf2_msgs_DIR "$ENV{ROS_ROOT}/../../share/tf2_msgs/cmake")
#set(tf2_ros_DIR "$ENV{ROS_ROOT}/../../share/tf2_ros/cmake")
#set(tf2_py_DIR "$ENV{ROS_ROOT}/../../share/tf2_py/cmake")
#set(rospy_DIR "$ENV{ROS_ROOT}/../../share/rospy/cmake")
#set(message_filters_DIR "$ENV{ROS_ROOT}/../../share/message_filters/cmake")
#set(message_generation_DIR "$ENV{ROS_ROOT}/../../share/message_generation/cmake")
#set(gencpp_DIR "$ENV{ROS_ROOT}/../../share/gencpp/cmake")
#set(genmsg_DIR "$ENV{ROS_ROOT}/../../share/genmsg/cmake")
#set(geneus_DIR "$ENV{ROS_ROOT}/../../share/geneus/cmake")
#set(gennodejs_DIR "$ENV{ROS_ROOT}/../../share/gennodejs/cmake")
#set(genlisp_DIR "$ENV{ROS_ROOT}/../../share/genlisp/cmake")
#set(genpy_DIR "$ENV{ROS_ROOT}/../../share/genpy/cmake")
#set(actionlib_msgs_DIR "$ENV{ROS_ROOT}/../../share/actionlib_msgs/cmake")
#set(actionlib_DIR "$ENV{ROS_ROOT}/../../share/actionlib/cmake")
#set(rosgraph_msgs_DIR "$ENV{ROS_ROOT}/../../share/rosgraph_msgs/cmake")
#set(rosgraph_DIR "$ENV{ROS_ROOT}/../../share/rosgraph/cmake")

list(APPEND CMAKE_MODULE_PATH "$ENV{ROS_ROOT}/../../share/tf2/cmake")
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  std_msgs
  message_runtime
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   MapOdomBaseLinkTfs.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  $ENV{ROS_ROOT}/../../include
  /opt/ros/$ENV{ROS_DISTRO}/include
)
link_directories(  
  /opt/ros/$ENV{ROS_DISTRO}/lib
)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_executable(bb_equipment_tf_publisher src/bb_tf_equipment_publisher.cpp src/bb_equipment_tf_publisher_node.cpp)
add_dependencies(bb_equipment_tf_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bb_equipment_tf_publisher
  ${catkin_LIBRARIES} tf2 tf2_ros roscpp rosconsole xmlrpcpp boost_chrono
)

install(TARGETS bb_equipment_tf_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
