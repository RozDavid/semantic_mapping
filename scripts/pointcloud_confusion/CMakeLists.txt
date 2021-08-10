cmake_minimum_required(VERSION 3.19)
project(semantic_cloud_confusion)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(semantic_cloud_confusion main.cpp)
target_link_libraries (semantic_cloud_confusion ${PCL_LIBRARIES})
