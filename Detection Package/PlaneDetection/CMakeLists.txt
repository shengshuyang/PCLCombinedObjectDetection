cmake_minimum_required(VERSION 2.8)

PROJECT(PlaneDetection)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET( PROJECT_SRCS
 extremeFinder.h
 geonDetector.h
 extremeFinder.cpp
 geonDetector.cpp
 hough_accumulator.cpp
 includes.h
 intersection_test.h
 main.cpp
 range_image_creation.cpp
 region_growing_rgb_segmentation.cpp
 shadow_detection.cpp
 std_include.h
 vector3.h
)

ADD_EXECUTABLE(PlaneDetection ${PROJECT_SRCS})
target_link_libraries(PlaneDetection ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_LIBRARIES})