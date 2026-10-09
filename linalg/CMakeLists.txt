set(NAME lidar_viewer_linalg)

add_library(${NAME} INTERFACE)

target_include_directories(${NAME}
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
