add_library (rt_config_widgets INTERFACE)

target_include_directories (rt_config_widgets
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_config_widgets INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicConfigWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicConfigWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PlayBagConfigWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PlayBagConfigWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.hpp
)

target_link_libraries(rt_config_widgets
    INTERFACE Qt::Widgets rt_utils
) 
