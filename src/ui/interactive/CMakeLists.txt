add_library (rt_interactive INTERFACE)

target_include_directories (rt_interactive
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_interactive INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ControlBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ControlBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ControlPlayBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ControlPlayBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ControlRecordBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ControlRecordBagWidget.hpp
)

target_link_libraries(rt_interactive
    INTERFACE Qt::Widgets rt_ros rt_utils
) 
