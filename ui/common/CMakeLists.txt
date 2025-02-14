set(NAME lidar_viewer_ui)

add_library(${NAME} STATIC)

target_sources(${NAME} PUBLIC
        src/DisplayPointCloud.cxx
        src/DisplayOctreeFromPointCloud.cxx
        src/DisplayFlatDepthImage.cxx
        src/DisplayStatistics.cxx
        src/ViewManager.cxx
        src/DisplayManager.cxx)
        
target_include_directories(${NAME} PUBLIC inc)

target_link_libraries(${NAME} PRIVATE GL glut lidar_viewer_device
                                PUBLIC lidar_viewer_geometry)
