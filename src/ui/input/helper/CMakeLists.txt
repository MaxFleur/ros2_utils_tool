add_library (rt_helper INTERFACE)

target_include_directories (rt_helper
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_helper INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/LowDiskSpaceWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/LowDiskSpaceWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicComboBoxWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicComboBoxWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MessageCountWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MessageCountWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicListingInputWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicListingInputWidget.hpp
)

target_link_libraries(rt_helper
    INTERFACE Qt::Widgets rt_input
) 
