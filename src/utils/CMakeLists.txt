add_library (rt_utils_cli INTERFACE)
add_library (rt_utils_misc INTERFACE)
add_library (rt_utils_ui INTERFACE)

target_include_directories (rt_utils_cli
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
) 
target_include_directories (rt_utils_misc
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories (rt_utils_ui
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(rt_utils_cli INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/UtilsCLI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsCLI.hpp
)
target_sources(rt_utils_misc INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/NodeWrapper.hpp
    ${CMAKE_CURRENT_LIST_DIR}/Parameters.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsBag.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsGeneral.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsIO.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsIO.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsROS.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsTF2.hpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsThreads.hpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/VideoEncoder.hpp
)
target_sources(rt_utils_ui INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/UtilsUI.hpp
)

target_link_libraries(rt_utils_cli
    INTERFACE rt_settings
)
target_link_libraries(rt_utils_misc
    INTERFACE ${OpenCV_LIBS} rt_settings
)
target_link_libraries(rt_utils_ui
    INTERFACE Qt::Widgets rt_settings
)
