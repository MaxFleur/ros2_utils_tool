cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(mediassist4_ros_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if($ENV{ROS_DISTRO} STREQUAL "jazzy")
    add_definitions(-DROS_JAZZY)
else()
    add_definitions(-DROS_HUMBLE)
endif()

if(${Qt6_FOUND})
    find_package(QT NAMES Qt6 COMPONENTS Widgets REQUIRED)
    find_package(Qt6 COMPONENTS Widgets REQUIRED)
else()
    find_package(QT NAMES Qt5 COMPONENTS Widgets REQUIRED)
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
endif()

add_subdirectory(src)
add_subdirectory(test)

add_executable(tool_ui
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp resources/resources.qrc
)
add_executable(tool_bag_to_images
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/BagToImages.cpp
)
add_executable(tool_bag_to_video
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/BagToVideo.cpp
)
add_executable(tool_video_to_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/VideoToBag.cpp
)

ament_target_dependencies(tool_ui cv_bridge rclcpp rosbag2_cpp rosbag2_transport sensor_msgs std_msgs)
ament_target_dependencies(tool_bag_to_images cv_bridge rclcpp rosbag2_transport rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_bag_to_video cv_bridge rclcpp rosbag2_transport rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_video_to_bag cv_bridge rclcpp rosbag2_transport rosbag2_cpp sensor_msgs)

target_link_libraries(tool_ui
    Qt::Widgets rt_ros rt_ui
)
target_link_libraries(tool_bag_to_images
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_bag_to_video
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_video_to_bag
    Qt::Widgets rt_thread rt_utils
)

install(TARGETS
    tool_ui
    tool_bag_to_images
    tool_bag_to_video
    tool_video_to_bag
    tool_tests
    DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
