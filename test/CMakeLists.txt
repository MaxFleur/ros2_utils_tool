find_package(catch_ros2 REQUIRED)

add_executable(tool_tests
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp

    ${CMAKE_CURRENT_LIST_DIR}/ros/threads/ThreadsTest.cpp

    ${CMAKE_CURRENT_LIST_DIR}/ui/settings/SettingsTest.cpp

    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsCLITest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsROSTest.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/UtilsUITest.cpp
)

target_link_libraries(tool_tests
    rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport
    cv_bridge::cv_bridge ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_msgs_TARGETS}
    catch_ros2::catch_ros2_with_main Qt::Widgets rt_all_threads rt_settings_input rt_utils
)

if($ENV{ROS_DISTRO} STREQUAL "kilted")
    target_link_libraries(tool_tests
        pcl_conversions::pcl_conversions
    )
else()
    ament_target_dependencies(tool_tests pcl_conversions)
endif()

