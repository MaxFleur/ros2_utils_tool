add_library (rt_thread_cv_bridge INTERFACE)

target_include_directories (rt_thread_cv_bridge
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_thread_cv_bridge INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BasicThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/EncodingThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/EncodingThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToBagThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToBagThread.hpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToImageThread.cpp
    ${CMAKE_CURRENT_LIST_DIR}/WriteToImageThread.hpp
)

target_link_libraries(rt_thread_cv_bridge
    INTERFACE ${OpenCV_LIBS} Qt::Widgets rt_utils
)
