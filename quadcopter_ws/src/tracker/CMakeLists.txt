cmake_minimum_required(VERSION 2.8.3)
project(tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        apriltag_ros
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(fhog
        include/fhog.h
        include/sse.hpp
        src/fhog.cpp)
target_link_libraries(fhog
        ${catkin_LIBRARIES}
        )

add_library(staple_tracker
        include/staple_tracker.hpp
        src/staple_tracker.cpp)
target_link_libraries(staple_tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_library(antiSwing
        include/antiSwing.h
        src/antiSwing.cpp)
target_link_libraries(antiSwing
        ${catkin_LIBRARIES}
        )

add_executable(tracker
        src/tracker.cpp
        src/fhog.cpp
        src/staple_tracker.cpp)
target_link_libraries(tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        fhog
        staple_tracker
        antiSwing
        )

add_executable(flightController
        src/payloadControl_PrecisePos.cpp)
target_link_libraries(flightController
        ${catkin_LIBRARIES}
        antiSwing
        )
