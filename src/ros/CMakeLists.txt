add_subdirectory(thread)

add_library (rt_ros INTERFACE)

target_include_directories (rt_ros
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_ros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagPlayer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagPlayer.hpp
)

target_link_libraries(rt_ros
    INTERFACE Qt::Widgets rt_utils
)
