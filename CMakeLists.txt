cmake_minimum_required(VERSION 2.8.3)
project(hiwr_code_scanner)


find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  hiwr_msg
)


catkin_package(
  LIBRARIES hiwr_code_scanner
  DEPENDS roscpp nodelet
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++0x -pthread")


add_library(hiwr_code_scanner src/hiwr_code_scanner.cpp)
target_link_libraries(hiwr_code_scanner ${catkin_LIBRARIES} zbar)
add_dependencies(hiwr_code_scanner ${catkin_EXPORTED_TARGETS})
