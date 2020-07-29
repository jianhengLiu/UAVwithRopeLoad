cmake_minimum_required(VERSION 2.8.3)
project(tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        )

find_package(OpenCV REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include


)

add_library(FlightController
        include/flightController.h
        src/flightController.cpp)
target_link_libraries(FlightController
        ${catkin_LIBRARIES}
        )

add_library(trajectory_generator_f
        include/trajectory_generator_waypoint_f.h
        src/trajectory_generator_waypoint_f.cpp)
target_link_libraries(trajectory_generator_f
        ${catkin_LIBRARIES})

add_executable(flightController_node
        app/flightController_node.cpp
        )
target_link_libraries(flightController_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        FlightController
        trajectory_generator_f
        )
