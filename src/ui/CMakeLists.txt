add_subdirectory(InputWidgets)
add_subdirectory(HelperWidgets)
add_subdirectory(settings)

add_library (rt_ui INTERFACE)

target_include_directories (rt_ui
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_ui INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/MainWindow.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MainWindow.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ProgressWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ProgressWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/SettingsDialog.cpp
    ${CMAKE_CURRENT_LIST_DIR}/SettingsDialog.hpp
    ${CMAKE_CURRENT_LIST_DIR}/StartWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/StartWidget.hpp
)

target_link_libraries(rt_ui
    INTERFACE Qt::Widgets rt_all_threads rt_input_widgets rt_settings rt_utils
) 
