cmake_minimum_required(VERSION 3.10)

project(opencv2)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

# OpenCV
find_package(OpenCV REQUIRED) # 修改这里
find_package(Eigen3 REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS}) # 修改这里
include_directories(${EIGEN3_INCLUDE_DIR})


message(STATUS "OpenCV include directory: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV include directory: $${EIGEN3_INCLUDE_DIR}")

add_executable(main main.cpp intersection_recognizer.cpp) 

target_link_libraries(main ${OpenCV_LIBS}) # 这里通常不需要修改