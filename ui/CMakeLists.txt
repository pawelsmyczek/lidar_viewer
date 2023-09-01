set(NAME ui)

add_library(${NAME} STATIC)

target_sources(${NAME} PRIVATE
        src/Viewer.cxx
        src/DisplayFunctions.cxx)
target_include_directories(${NAME} PUBLIC inc)

target_link_libraries(${NAME} PRIVATE dev GL glut)
