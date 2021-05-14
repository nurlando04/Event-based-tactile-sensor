cmake_minimum_required(VERSION 3.0.2)
project(tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	OpenCV
        roscpp
	std_msgs
        genmsg
        message_generation
)

## System dependencies are found with CMake's conventions
find_package(MetavisionSDK COMPONENTS devtools driver core analytics REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)



catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

#prophesee spatter tracking
add_executable(prophesee_ros_spatter
    src/metavision_spatter_tracking.cpp
    src/simple_video_writer.cpp
    src/simple_timer.cpp
)

target_link_libraries(
    prophesee_ros_spatter
    metavision_sdk_driver
    metavision_sdk_core
    metavision_sdk_base
    metavision_sdk_analytics
    Boost::program_options
    ${catkin_LIBRARIES}
)

add_dependencies(
    prophesee_ros_spatter
    prophesee_event_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)




