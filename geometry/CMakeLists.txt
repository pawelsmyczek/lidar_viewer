set(NAME lidar_viewer_geometry)

add_library(${NAME} INTERFACE)

target_link_libraries(${NAME} INTERFACE lidar_viewer_linalg)

target_include_directories(${NAME}
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

