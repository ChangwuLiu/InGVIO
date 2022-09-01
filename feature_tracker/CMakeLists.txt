cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_filters
    sensor_msgs
    cv_bridge
    camera_model
    message_generation
)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)


add_message_files(
   FILES
   MonoMeas.msg
   MonoFrame.msg
   StereoMeas.msg
   StereoFrame.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


catkin_package()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    "/usr/include/eigen3"
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(mono_tracker
    src/mono_tracker.cpp
    src/mono_tracker_node.cpp
    src/mono_parameters.cpp
)

add_executable(stereo_tracker
    src/stereo_tracker.cpp
    src/stereo_tracker_node.cpp
    src/stereo_parameters.cpp
)

target_link_libraries(mono_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(stereo_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
