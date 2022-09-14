cmake_minimum_required(VERSION 3.16)

find_package(catkin QUIET COMPONENTS
    roscpp
    roslib
    std_msgs
    message_filters
    sensor_msgs
    eigen_conversions
    geometry_msgs
    tf
    tf_conversions
    nav_msgs
    gnss_comm
)

catkin_package(
    CATKIN_DEPENDS roscpp roslib std_msgs message_filters sensor_msgs eigen_conversions geometry_msgs tf tf_conversions nav_msgs gnss_comm
    INCLUDE_DIRS src/
    DEPENDS OpenCV SUITESPARSE Boost
)

include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
)

add_executable(ingvio 
    ${PROJECT_SOURCE_DIR}/src/IngvioNode.cpp
)

list(APPEND thirdparty_libs 
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
)
target_link_libraries(ingvio
    ${thirdparty_libs}
)

message(STATUS "Tests enabled! Build in-project tests!")

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
    
add_executable(test_state_manager
    ${PROJECT_SOURCE_DIR}/test/TestStateManager.cpp
    ${PROJECT_SOURCE_DIR}/src/AuxGammaFunc.cpp
    ${PROJECT_SOURCE_DIR}/src/State.cpp
    ${PROJECT_SOURCE_DIR}/src/StateManager.cpp
    ${PROJECT_SOURCE_DIR}/src/IngvioParams.cpp
)
target_link_libraries(test_state_manager ${GTEST_BOTH_LIBRARIES} pthread ${thirdparty_libs})
gtest_discover_tests(test_state_manager)

add_executable(test_propagator
    ${PROJECT_SOURCE_DIR}/test/TestPropagator.cpp
    ${PROJECT_SOURCE_DIR}/src/AuxGammaFunc.cpp
    ${PROJECT_SOURCE_DIR}/src/State.cpp
    ${PROJECT_SOURCE_DIR}/src/StateManager.cpp
    ${PROJECT_SOURCE_DIR}/src/IngvioParams.cpp
    ${PROJECT_SOURCE_DIR}/src/ImuPropagator.cpp
)
target_link_libraries(test_propagator ${GTEST_BOTH_LIBRARIES} pthread ${thirdparty_libs})
gtest_discover_tests(test_propagator)

add_executable(test_map_server
    ${PROJECT_SOURCE_DIR}/test/TestMapServer.cpp
    ${PROJECT_SOURCE_DIR}/src/AuxGammaFunc.cpp
    ${PROJECT_SOURCE_DIR}/src/State.cpp
    ${PROJECT_SOURCE_DIR}/src/StateManager.cpp
    ${PROJECT_SOURCE_DIR}/src/IngvioParams.cpp
    ${PROJECT_SOURCE_DIR}/src/ImuPropagator.cpp
    ${PROJECT_SOURCE_DIR}/src/MapServerManager.cpp
)
target_link_libraries(test_map_server ${GTEST_BOTH_LIBRARIES} pthread ${thirdparty_libs})
gtest_discover_tests(test_map_server)

add_executable(test_triangulator
    ${PROJECT_SOURCE_DIR}/test/TestTriangulator.cpp
    ${PROJECT_SOURCE_DIR}/src/Triangulator.cpp
    ${PROJECT_SOURCE_DIR}/src/IngvioParams.cpp
    ${PROJECT_SOURCE_DIR}/src/AuxGammaFunc.cpp
)
target_link_libraries(test_triangulator ${GTEST_BOTH_LIBRARIES} pthread ${thirdparty_libs})
gtest_discover_tests(test_triangulator)
