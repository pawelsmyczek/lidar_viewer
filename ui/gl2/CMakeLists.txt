set(NAME lidar_viewer_ui_gl2)

add_library(${NAME} STATIC)

target_sources(${NAME} PUBLIC
        src/GetScreenParameters.cxx
        src/DisplayManagerGl.cxx
        src/ViewManagerGl.cxx
        src/DrawString.cxx)
        
target_include_directories(${NAME} PUBLIC inc)

target_link_libraries(${NAME} PRIVATE GL glut lidar_viewer_ui lidar_viewer_device
                                PUBLIC lidar_viewer_geometry)
