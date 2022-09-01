cmake_minimum_required(VERSION 2.8.3)
project(camera_model)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

if(EXISTS "/usr/include/eigen3")
    include_directories("/usr/include/eigen3")
else()
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES camera_model
    CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# if(EXISTS "${PROJECT_SOURCE_DIR}/test/")
#     enable_testing()
#     add_subdirectory(${PROJECT_SOURCE_DIR}/test)
# endif()

add_library(camera_model
    src/camera.cpp
    src/MeiCamera.cpp
    src/PinholeCamera.cpp
    src/camera_factory.cpp
)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS})
