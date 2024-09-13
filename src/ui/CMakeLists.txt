add_subdirectory(ConfigWidgets)
add_subdirectory(ProgressWidgets)

add_library (rt_ui INTERFACE)

target_include_directories (rt_ui
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_ui INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/MainWindow.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MainWindow.hpp
    ${CMAKE_CURRENT_LIST_DIR}/StartWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/StartWidget.hpp
)

target_link_libraries(rt_ui
    INTERFACE Qt::Widgets rt_config_widgets rt_progress_widgets rt_utils
) 
