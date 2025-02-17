add_library (rt_settings_input INTERFACE)

target_include_directories (rt_settings_input
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_settings_input INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedInputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedInputSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagInputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagInputSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagInputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagInputSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagInputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagInputSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ImageInputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ImageInputSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/InputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/InputSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsInputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsInputSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoInputSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoInputSettings.hpp
)

target_link_libraries(rt_settings_input
    INTERFACE Qt::Widgets rt_settings
)
