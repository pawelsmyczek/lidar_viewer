set(NAME lidar_viewer_device)

add_library(${NAME} STATIC)

target_sources(${NAME} PRIVATE
        src/BinaryFile.cxx
        src/IoStream.cxx
        src/SerialPort.cxx
        src/CygLidarD1.cxx
        src/CustomBaudrateSetter.cxx)

target_include_directories(${NAME} PUBLIC inc)

target_link_libraries(${NAME} PRIVATE pthread)
