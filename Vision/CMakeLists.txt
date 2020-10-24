cmake_minimum_required(VERSION 3.5)
project(opencv)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11)

add_executable(blob blob.cpp)
add_executable(calib calib.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(blob ${OpenCV_LIBS})
target_link_libraries(calib ${OpenCV_LIBS})