add_library (rt_thread INTERFACE)

target_include_directories (rt_thread
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EncodingThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EncodingThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToImageThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToImageThread.hpp
)

target_link_libraries(rt_thread
    INTERFACE ${OpenCV_LIBS} Qt::Widgets rt_utils
)
