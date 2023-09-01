set(NAME ui)

add_library(${NAME} STATIC)

target_sources(${NAME} PUBLIC
        src/Viewer.cxx
        src/DisplayFunctions.cxx)
        
target_include_directories(${NAME} PUBLIC inc)

target_link_libraries(${NAME} PRIVATE GL glut dev)
