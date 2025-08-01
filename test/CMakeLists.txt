find_package(catch_ros2 REQUIRED)

add_executable(tool_tests
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp

    ${CMAKE_CURRENT_LIST_DIR}/ros/threads/ThreadsTest.cpp

    ${CMAKE_CURRENT_LIST_DIR}/ui/settings/SettingsTest.cpp

    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsCLITest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsROSTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsUITest.cpp
)

if($ENV{ROS_DISTRO} STREQUAL "kilted")
    target_link_libraries(tool_tests
        rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport
        cv_bridge::cv_bridge ${geometry_msgs_TARGETS} pcl_conversions::pcl_conversions ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_msgs_TARGETS}
        catch_ros2::catch_ros2_with_main Qt::Widgets rt_all_threads rt_settings_input rt_utils
    )
else()
    ament_target_dependencies(tool_tests cv_bridge geometry_msgs pcl_conversions rclcpp rosbag2_cpp rosbag2_transport sensor_msgs std_msgs tf2_msgs)

    target_link_libraries(tool_tests
        catch_ros2::catch_ros2_with_main Qt::Widgets rt_all_threads rt_settings_input rt_utils
    )
endif()

