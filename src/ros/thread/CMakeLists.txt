# Lots of classes here, only compile them all once
add_library (rt_all_threads STATIC)

target_include_directories (rt_all_threads
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_all_threads PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/BagMessageToFileThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagMessageToFileThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagTF2ToFileThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagTF2ToFileThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Thread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Thread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicsServicesThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.hpp
)

target_link_libraries(rt_all_threads
    PUBLIC Qt::Core rt_basic_thread rt_utils_misc rt_utils_ui rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp cv_bridge::cv_bridge
    PUBLIC ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_msgs_TARGETS} tf2_ros::tf2_ros yaml-cpp rt_pcl_conversions
    # Only used by the lib itself
    PRIVATE Python3::Python
)

# Libs for CLI tools
add_library (rt_basic_thread STATIC)

target_include_directories (rt_basic_thread
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_sources(rt_basic_thread PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.hpp
)
target_link_libraries(rt_basic_thread
    PUBLIC Qt::Core
)

add_library (rt_bag_message_to_file_thread INTERFACE)
add_library (rt_bag_to_images_thread INTERFACE)
add_library (rt_bag_to_pcds_thread INTERFACE)
add_library (rt_bag_to_video_thread INTERFACE)
add_library (rt_change_compression_thread INTERFACE)
add_library (rt_dummy_bag_thread INTERFACE)
add_library (rt_merge_bags_thread INTERFACE)
add_library (rt_pcds_to_bag_thread INTERFACE)
add_library (rt_publish_images_thread INTERFACE)
add_library (rt_publish_video_thread INTERFACE)
add_library (rt_send_tf2_thread INTERFACE)
add_library (rt_tf2_to_file_thread INTERFACE)
add_library (rt_video_to_bag_thread INTERFACE)

target_sources(rt_bag_to_images_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.hpp
)
target_sources(rt_bag_to_pcds_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.hpp
)
target_sources(rt_bag_to_video_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.hpp
)
target_sources(rt_bag_message_to_file_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagMessageToFileThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagMessageToFileThread.hpp
)
target_sources(rt_change_compression_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionBagThread.hpp
)
target_sources(rt_dummy_bag_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.hpp
)
target_sources(rt_merge_bags_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.hpp
)
target_sources(rt_pcds_to_bag_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagThread.hpp
)
target_sources(rt_publish_images_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.hpp
)
target_sources(rt_publish_video_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.hpp
)
target_sources(rt_send_tf2_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Thread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Thread.hpp
)
target_sources(rt_tf2_to_file_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagTF2ToFileThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagTF2ToFileThread.hpp
)
target_sources(rt_video_to_bag_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.hpp
)

target_link_libraries(rt_bag_message_to_file_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_bag_to_images_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_bag_to_pcds_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_bag_to_video_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_change_compression_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_dummy_bag_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_merge_bags_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_pcds_to_bag_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_publish_images_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_publish_video_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
target_link_libraries(rt_send_tf2_thread
    INTERFACE rt_basic_thread rt_utils_misc tf2_ros::tf2_ros
)
target_link_libraries(rt_tf2_to_file_thread
    INTERFACE rt_basic_thread rt_utils_misc yaml-cpp
)
target_link_libraries(rt_video_to_bag_thread
    INTERFACE rt_basic_thread rt_utils_misc
)
