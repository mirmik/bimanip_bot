cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(world SHARED world.cc)
target_link_libraries(world ${GAZEBO_LIBRARIES} nos igris crow)

add_library(manipulator SHARED manipulator.cc)
target_link_libraries(manipulator ${GAZEBO_LIBRARIES} nos igris crow)

add_library(cargo SHARED cargo.cc)
target_link_libraries(cargo ${GAZEBO_LIBRARIES} nos igris crow)