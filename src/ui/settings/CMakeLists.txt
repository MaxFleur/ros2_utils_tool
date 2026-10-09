# Not used that much, but still might be a compile time gain to make it static
add_subdirectory(input)

add_library (rt_settings STATIC)

target_include_directories (rt_settings
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_settings PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/GeneralSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/GeneralSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DialogSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DialogSettings.hpp
)

target_link_libraries(rt_settings
    PUBLIC Qt::Core rt_utils_misc
)
