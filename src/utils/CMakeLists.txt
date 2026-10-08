# CLI utils are for CLI tools only, while UI parts are for UI solely
# Split into separate libraries to prevent globalized imports
# Also make these static as they're used by lots of different targets
add_library (rt_utils_cli STATIC)
add_library (rt_utils_misc STATIC)
add_library (rt_utils_ui STATIC)

target_include_directories (rt_utils_cli
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories (rt_utils_misc
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
)
target_include_directories (rt_utils_ui
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_utils_cli PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/UtilsCLI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsCLI.hpp
)
target_sources(rt_utils_misc PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/NodeWrapper.hpp
    ${CMAKE_CURRENT_LIST_DIR}/Parameters.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsBag.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsIO.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsIO.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsTF2.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsThreads.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.hpp
)
target_sources(rt_utils_ui PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.hpp
)

target_link_libraries(rt_utils_cli
    PUBLIC Qt::Core rt_utils_misc rt_settings
    PRIVATE rt_basic_thread
)
target_link_libraries(rt_utils_misc
    PUBLIC Qt::Core rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp rosbag2_transport::rosbag2_transport
    PUBLIC ${sensor_msgs_TARGETS} tf2_ros::tf2_ros cv_bridge::cv_bridge ${OpenCV_LIBS} yaml-cpp
)
target_link_libraries(rt_utils_ui
    PUBLIC Qt::Widgets rt_utils_misc rt_settings
)
