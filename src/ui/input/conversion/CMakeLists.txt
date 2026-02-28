add_library (rt_input_conversion INTERFACE)

target_include_directories (rt_input_conversion
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_input_conversion INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagTF2ToFileWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagTF2ToFileWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.hpp
)

target_link_libraries(rt_input_conversion
    INTERFACE Qt::Widgets rt_input rt_helper rt_settings_input rt_utils
) 
