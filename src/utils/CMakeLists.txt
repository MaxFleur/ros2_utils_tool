add_library (rt_utils INTERFACE)

target_include_directories (rt_utils
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 

target_sources(rt_utils INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/Parameters.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsCLI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsCLI.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.hpp
)

target_link_libraries(rt_utils
    INTERFACE ${OpenCV_LIBS} Qt::Widgets rt_settings
)
