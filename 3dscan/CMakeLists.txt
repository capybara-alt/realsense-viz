cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(3dscan)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp)

target_link_libraries (main ${realsense2_LIBRARY})
target_link_libraries (main ${OpenCV_LIBS})
target_link_libraries (main ${PCL_LIBRARIES})
