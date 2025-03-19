add_library (rt_cli INTERFACE)

target_include_directories (rt_cli
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_cli INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImages.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDs.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/CompressBag.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBag.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBags.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBag.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImages.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBag.cpp
)

target_link_libraries(rt_cli
    INTERFACE Qt::Widgets rt_thread rt_utils
)
