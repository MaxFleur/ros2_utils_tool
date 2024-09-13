add_library (rt_cli INTERFACE)

target_include_directories (rt_cli
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_cli INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImages.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBag.cpp
)

target_link_libraries(rt_cli
    INTERFACE Qt::Widgets rt_thread_cv_bridge rt_utils
)
