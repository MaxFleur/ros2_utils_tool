add_library (rt_thread_play_bag INTERFACE)

target_include_directories (rt_thread_play_bag
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_thread_play_bag INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/PlayBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PlayBagThread.hpp
)

target_link_libraries(rt_thread_play_bag
    INTERFACE Qt::Widgets rt_utils
)
