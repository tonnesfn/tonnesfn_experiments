cmake_minimum_required(VERSION 3.1)
project(tonnesfn_experiments)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ltbb -D SFERES_XML_WRITE")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

add_definitions( -DADD_DIVERSITY_FITNESS)
add_definitions( -DSFERES_NO_STATE)

find_package(message_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_dev
  gazebo_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  genmsg
  dynamic_reconfigure
  dyret_common
  dyret_hardware
  dyret_controller
  rosbag
  camera_recorder
)

find_package(Boost REQUIRED COMPONENTS 
  program_options
  serialization
  graph
)

find_package(TBB REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  DataPoint.msg
)


add_service_files(
  FILES
  getMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

add_executable(expGui
               src/expGui.cpp
	       src/expFunctions.cpp
              )
target_link_libraries(expGui ${catkin_LIBRARIES})
target_link_libraries(expGui ${Boost_LIBRARIES})
target_link_libraries(expGui ${TBB_LIBRARIES})
add_dependencies(expGui ${catkin_EXPORTED_TARGETS})

add_executable(gaitAdaptationController
               src/gaitAdaptationController.cpp
	       src/expFunctions.cpp
              )
target_link_libraries(gaitAdaptationController ${catkin_LIBRARIES})
add_dependencies(gaitAdaptationController ${catkin_EXPORTED_TARGETS})

