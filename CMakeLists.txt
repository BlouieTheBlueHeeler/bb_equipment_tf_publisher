cmake_minimum_required(VERSION 3.0.2)
project(bb_equipment_tf_publisher)

set(CMAKE_VERBOSE_MAKEFILE On)

add_compile_options(-std=c++17)

find_package(Boost REQUIRED) # COMPONENTS thread chrono)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  std_msgs
  roscpp
  tf2
  tf2_ros
  message_runtime
  message_generation
  interactive_markers
  visualization_msgs
)

add_service_files(
   FILES
   MapOdomBaseLinkTfs.srv
   StaticTfs.srv
   StaticTfUpdate.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs geometry_msgs std_msgs roscpp tf2 tf2_ros interactive_markers visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_executable(bb_equipment_tf_publisher
               include/bb_equipment_tf_publisher/bb_equipment_tf_publisher.h
               src/bb_equipment_tf_publisher.cpp
               src/bb_equipment_tf_publisher_node.cpp)
add_dependencies(bb_equipment_tf_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bb_equipment_tf_publisher
  ${catkin_LIBRARIES} boost_chrono
)

add_executable(bb_equipment_tf_marker_server
                include/bb_equipment_tf_publisher/bb_equipment_tf_marker_server.h
                src/bb_equipment_tf_marker_server.cpp)
add_dependencies(bb_equipment_tf_marker_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bb_equipment_tf_marker_server
  ${catkin_LIBRARIES} boost_chrono
)

install(TARGETS bb_equipment_tf_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS bb_equipment_tf_marker_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
