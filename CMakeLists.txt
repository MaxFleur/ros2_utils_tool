cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(ros2_utils_tool LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

find_package(PCL REQUIRED)

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
add_executable(tool_dummy_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/DummyBag.cpp
)
add_executable(tool_merge_bags
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/MergeBags.cpp
)
add_executable(tool_publish_images
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/PublishImages.cpp
)
add_executable(tool_publish_video
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/PublishVideo.cpp
)
add_executable(tool_video_to_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/VideoToBag.cpp
)

ament_target_dependencies(tool_ui cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_bag_to_images cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_bag_to_video cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_dummy_bag cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_merge_bags cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_publish_images cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_publish_video cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)
ament_target_dependencies(tool_video_to_bag cv_bridge pcl_conversions rclcpp rosbag2_cpp sensor_msgs)

target_link_libraries(tool_ui
    Qt::Widgets rt_ui
)
target_link_libraries(tool_bag_to_images
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_bag_to_video
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_dummy_bag
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_merge_bags
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_publish_images
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_publish_video
    Qt::Widgets rt_thread rt_utils
)
target_link_libraries(tool_video_to_bag
    Qt::Widgets rt_thread rt_utils
)

install(TARGETS
    tool_ui
    tool_bag_to_images
    tool_bag_to_video
    tool_dummy_bag
    tool_merge_bags
    tool_publish_images
    tool_publish_video
    tool_video_to_bag
    tool_tests
    DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
