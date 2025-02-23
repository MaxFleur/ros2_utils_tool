add_subdirectory(input)

add_library (rt_settings INTERFACE)

target_include_directories (rt_settings
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_settings INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/GeneralSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DialogSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DialogSettings.hpp
)

target_link_libraries(rt_settings
    INTERFACE Qt::Widgets
)
