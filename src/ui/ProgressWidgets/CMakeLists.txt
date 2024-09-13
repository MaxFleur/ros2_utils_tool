add_library (rt_progress_widgets INTERFACE)

target_include_directories (rt_progress_widgets
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_progress_widgets INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicProgressWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ImagesProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ImagesProgressWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PlayBagProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PlayBagProgressWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoProgressWidget.hpp
)

target_link_libraries(rt_progress_widgets
    INTERFACE Qt::Widgets rt_thread_cv_bridge rt_thread_play_bag rt_utils
) 
