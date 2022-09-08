cmake_minimum_required(VERSION 3.16)
project(ingvio_estimator)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(Eigen3 QUIET)
if (NOT Eigen3_FOUND)
    set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
endif ()

find_package(OpenCV 3 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
endif ()

find_package(SuiteSparse REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)

if (catkin_FOUND)
    message(STATUS "ROS *1* version found!")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND)
    message(STATUS "ROS *2* version found! Not supported!")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
    message(STATUS "No ROS version found! Cannot build!")
endif ()


