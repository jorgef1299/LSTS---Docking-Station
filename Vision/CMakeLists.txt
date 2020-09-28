cmake_minimum_required(VERSION 3.5)
project(opencv)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(circle_detection circle_detection.cpp)
add_executable(template_matching template_matching.cpp)
add_executable(blob blob.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(circle_detection ${OpenCV_LIBS})
target_link_libraries(template_matching ${OpenCV_LIBS})
target_link_libraries(blob ${OpenCV_LIBS})