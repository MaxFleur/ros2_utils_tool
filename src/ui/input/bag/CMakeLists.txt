add_library (rt_input_bag INTERFACE)

target_include_directories (rt_input_bag
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_input_bag INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagTreeWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagTreeWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ChangeCompressionWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/ConfigurePlayBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ConfigurePlayBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsWidget.hpp
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagWidget.cpp
    ${CMAKE_CURRENT_LIST_DIR}/RecordBagWidget.hpp
)

target_link_libraries(rt_input_bag
    INTERFACE Qt::Widgets rt_input rt_helper rt_settings_input rt_utils
) 
