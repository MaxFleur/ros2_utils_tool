add_library (rt_helper_widgets INTERFACE)

target_include_directories (rt_helper_widgets
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_helper_widgets INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyTopicWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyTopicWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MessageCountWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MessageCountWidget.hpp
)

target_link_libraries(rt_helper_widgets
    INTERFACE Qt::Widgets
) 
