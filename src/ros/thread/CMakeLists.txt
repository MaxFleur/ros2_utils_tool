# Lib for UI, utilizing all threads

add_library (rt_all_threads INTERFACE)

target_include_directories (rt_all_threads
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_all_threads INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.hpp
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
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.hpp
)

target_link_libraries(rt_all_threads
    INTERFACE Qt::Widgets rt_utils
)

# Libs for CLI tools

add_library (rt_basic_thread INTERFACE)
target_include_directories (rt_basic_thread
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_sources(rt_basic_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.hpp
)
target_link_libraries(rt_basic_thread
    INTERFACE Qt::Widgets
)

add_library (rt_bag_to_images_thread INTERFACE)
add_library (rt_bag_to_pcds_thread INTERFACE)
add_library (rt_bag_to_video_thread INTERFACE)
add_library (rt_change_compression_thread INTERFACE)
add_library (rt_dummy_bag_thread INTERFACE)
add_library (rt_merge_bags_thread INTERFACE)
add_library (rt_pcds_to_bag_thread INTERFACE)
add_library (rt_publish_images_thread INTERFACE)
add_library (rt_publish_video_thread INTERFACE)
add_library (rt_tf2_to_json_thread INTERFACE)
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
target_sources(rt_tf2_to_json_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonThread.hpp
)
target_sources(rt_video_to_bag_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.hpp
)

target_link_libraries(rt_bag_to_images_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_bag_to_pcds_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_bag_to_video_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_change_compression_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_dummy_bag_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_merge_bags_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_pcds_to_bag_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_publish_images_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_publish_video_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_tf2_to_json_thread
    INTERFACE rt_basic_thread rt_utils
)
target_link_libraries(rt_video_to_bag_thread
    INTERFACE rt_basic_thread rt_utils
)
