add_library (rt_thread INTERFACE)

target_include_directories (rt_thread
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_thread INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToImagesThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToPCDsThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BagToVideoThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EditBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/DummyBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/MergeBagsThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PCDsToBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishImagesThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PublishVideoThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoToBagThread.hpp
)

target_link_libraries(rt_thread
    INTERFACE ${OpenCV_LIBS} ${PCL_LIBRARIES} Qt::Widgets rt_utils
)
