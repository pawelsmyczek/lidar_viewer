set(NAME lidar_viewer)

add_executable(${NAME} main.cxx)
target_link_libraries(${NAME} PRIVATE lidar_viewer_ui lidar_viewer_ui_gl2 lidar_viewer_device pthread)