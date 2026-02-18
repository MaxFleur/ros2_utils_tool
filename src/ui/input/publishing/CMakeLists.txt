add_library (rt_input_publishing INTERFACE)

target_include_directories (rt_input_publishing
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_input_publishing INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/PublishWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Widget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Widget.hpp
)

target_link_libraries(rt_input_publishing
    INTERFACE Qt::Widgets rt_input rt_helper rt_settings_input rt_utils
) 
