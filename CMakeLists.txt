cmake_minimum_required(VERSION 2.8.3)
project(tonnesfn_experiments)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -ltbb -D SFERES_XML_WRITE")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

add_definitions( -DSFERES_NO_STATE)

find_package(message_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  genmsg
  dynamic_reconfigure
  dyret_common
  dyret_utils
  dyret_hardware
)

find_package(Boost REQUIRED COMPONENTS 
  program_options
  serialization
  graph
)

find_package(TBB REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

add_executable(exp1Gui
               src/exp1Gui.cpp
              )
target_link_libraries(exp1Gui ${catkin_LIBRARIES})
target_link_libraries(exp1Gui ${Boost_LIBRARIES})
target_link_libraries(exp1Gui ${TBB_LIBRARIES})
add_dependencies(exp1Gui
  ${catkin_EXPORTED_TARGETS})

add_executable(exp2Gui
               src/exp2Gui.cpp
              )
target_link_libraries(exp2Gui ${catkin_LIBRARIES})
target_link_libraries(exp2Gui ${Boost_LIBRARIES})
target_link_libraries(exp2Gui ${TBB_LIBRARIES})
add_dependencies(exp2Gui
  ${catkin_EXPORTED_TARGETS})

add_executable(gazeboTest
               src/gazeboTest.cpp
              )
target_link_libraries(gazeboTest ${catkin_LIBRARIES})
add_dependencies(gazeboTest
  ${catkin_EXPORTED_TARGETS})

add_executable(hardwareTest
               src/hardwareTest.cpp
              )
target_link_libraries(hardwareTest ${catkin_LIBRARIES})
add_dependencies(hardwareTest
  ${catkin_EXPORTED_TARGETS})

