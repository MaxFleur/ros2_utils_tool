add_library (rt_input_info INTERFACE)

target_include_directories (rt_input_info
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_input_info INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagInfoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagInfoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicsServicesInfoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicsServicesInfoWidget.hpp
)

target_link_libraries(rt_input_info
    INTERFACE Qt::Widgets rt_input rt_helper rt_settings_input rt_utils
) 
