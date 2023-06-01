set(NAME dev)

add_library(${NAME} STATIC)

target_sources(${NAME} PUBLIC
        src/SerialPort.cxx
        src/CygLidarD1.cxx
        src/CustomBaudrateSetter.cxx)
target_include_directories(${NAME} PUBLIC inc)
