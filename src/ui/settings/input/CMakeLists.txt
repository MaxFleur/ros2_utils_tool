add_library (rt_settings_input INTERFACE)

target_include_directories (rt_settings_input
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_settings_input INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/CompressBagSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/CompressBagSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DeleteSourceSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DeleteSourceSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/RGBSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/RGBSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Settings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/SendTF2Settings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoSettings.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagSettings.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagSettings.hpp
)

target_link_libraries(rt_settings_input
    INTERFACE Qt::Widgets rt_settings
)
