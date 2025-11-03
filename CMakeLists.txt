cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(ros2_utils_tool LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

if($ENV{ROS_DISTRO} STREQUAL "humble")
    add_definitions(-DROS_HUMBLE)
else()
    add_definitions(-DROS_JAZZY)
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
add_executable(tool_bag_to_pcds
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/BagToPCDs.cpp
)
add_executable(tool_bag_to_video
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/BagToVideo.cpp
)
add_executable(tool_compress_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/CompressBag.cpp
)
add_executable(tool_decompress_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/DecompressBag.cpp
)
add_executable(tool_dummy_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/DummyBag.cpp
)
add_executable(tool_merge_bags
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/MergeBags.cpp
)
add_executable(tool_pcds_to_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/PCDsToBag.cpp
)
add_executable(tool_publish_images
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/PublishImages.cpp
)
add_executable(tool_publish_video
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/PublishVideo.cpp
)
add_executable(tool_tf2_to_json
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/TF2ToJson.cpp
)
add_executable(tool_video_to_bag
    ${CMAKE_CURRENT_LIST_DIR}/src/cli/VideoToBag.cpp
)

target_link_libraries(tool_ui
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport cv_bridge::cv_bridge
    ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${tf2_msgs_TARGETS}
    Qt::Widgets rt_ui
)
target_link_libraries(tool_bag_to_images
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp cv_bridge::cv_bridge ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    Qt::Widgets rt_bag_to_images_thread rt_utils
)
target_link_libraries(tool_bag_to_pcds
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    Qt::Widgets rt_bag_to_pcds_thread rt_utils
)
target_link_libraries(tool_bag_to_video
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp cv_bridge::cv_bridge ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
    Qt::Widgets rt_bag_to_video_thread rt_utils
)
target_link_libraries(tool_compress_bag
    rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport ${sensor_msgs_TARGETS}
    Qt::Widgets rt_change_compression_thread rt_utils
)
target_link_libraries(tool_decompress_bag
    rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport ${sensor_msgs_TARGETS}
    Qt::Widgets rt_change_compression_thread rt_utils
)
target_link_libraries(tool_dummy_bag
    rosbag2_cpp::rosbag2_cpp cv_bridge::cv_bridge ${sensor_msgs_TARGETS} ${tf2_msgs_TARGETS}
    Qt::Widgets rt_dummy_bag_thread rt_utils
)
target_link_libraries(tool_merge_bags
    rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport ${sensor_msgs_TARGETS}
    Qt::Widgets rt_merge_bags_thread rt_utils
)
target_link_libraries(tool_pcds_to_bag
    rosbag2_cpp::rosbag2_cpp ${sensor_msgs_TARGETS}
    Qt::Widgets rt_pcds_to_bag_thread rt_utils
)
target_link_libraries(tool_publish_images
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp cv_bridge::cv_bridge ${sensor_msgs_TARGETS}
    Qt::Widgets rt_publish_images_thread rt_utils
)
target_link_libraries(tool_publish_video
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp cv_bridge::cv_bridge ${sensor_msgs_TARGETS}
    Qt::Widgets rt_publish_video_thread rt_utils
)
target_link_libraries(tool_tf2_to_json
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_msgs_TARGETS}
    Qt::Widgets rt_tf2_to_json_thread rt_utils
)
target_link_libraries(tool_video_to_bag
    rosbag2_cpp::rosbag2_cpp cv_bridge::cv_bridge ${sensor_msgs_TARGETS}
    Qt::Widgets rt_video_to_bag_thread rt_utils
)

if($ENV{ROS_DISTRO} STREQUAL "kilted")
    target_link_libraries(tool_ui
        pcl_conversions::pcl_conversions
    )
    target_link_libraries(tool_bag_to_pcds
        pcl_conversions::pcl_conversions
    )
    target_link_libraries(tool_dummy_bag
        pcl_conversions::pcl_conversions
    )
    target_link_libraries(tool_pcds_to_bag
        pcl_conversions::pcl_conversions
    )
else()
    ament_target_dependencies(tool_ui pcl_conversions)
    ament_target_dependencies(tool_bag_to_pcds pcl_conversions)
    ament_target_dependencies(tool_dummy_bag pcl_conversions )
    ament_target_dependencies(tool_pcds_to_bag pcl_conversions)
endif()

install(TARGETS
    tool_ui
    tool_bag_to_images
    tool_bag_to_pcds
    tool_bag_to_video
    tool_compress_bag
    tool_decompress_bag
    tool_dummy_bag
    tool_merge_bags
    tool_pcds_to_bag
    tool_publish_images
    tool_publish_video
    tool_tf2_to_json
    tool_video_to_bag
    tool_tests
    DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
