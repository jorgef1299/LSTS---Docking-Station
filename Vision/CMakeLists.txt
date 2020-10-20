cmake_minimum_required(VERSION 3.5)
project(opencv)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_executable(blob blob.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(blob ${OpenCV_LIBS})