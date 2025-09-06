add_library (rt_input_widgets INTERFACE)

target_include_directories (rt_input_widgets
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_input_widgets INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedInputWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/AdvancedInputWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagInfoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagInfoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToFileWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToFileWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicInputWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicInputWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TF2ToJsonWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicsServicesInfoWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/TopicsServicesInfoWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagWidget.hpp
)

target_link_libraries(rt_input_widgets
    INTERFACE Qt::Widgets rt_helper_widgets rt_settings_input rt_utils
) 
