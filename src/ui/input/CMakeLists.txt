add_subdirectory(bag)
add_subdirectory(conversion)
add_subdirectory(helper)
add_subdirectory(info)
add_subdirectory(publishing)

add_library (rt_input INTERFACE)

target_include_directories (rt_input
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_input INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedInputWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedInputWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicInputWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicInputWidget.hpp
)

target_link_libraries(rt_input
    INTERFACE Qt::Widgets rt_helper rt_settings_input rt_utils
) 
