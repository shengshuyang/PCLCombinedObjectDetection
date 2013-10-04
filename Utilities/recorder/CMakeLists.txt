cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcdBuffer)

find_package(PCL 1.5 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcdBufferRecorder recorder.cpp pcdBuffer.cpp)
target_link_libraries (pcdBufferRecorder ${PCL_LIBRARIES} ${Boost_LIBRARIES})

