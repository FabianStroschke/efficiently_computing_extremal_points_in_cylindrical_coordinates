# Projects Settings
cmake_minimum_required (VERSION 3.1)
project (WIP)

set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
SET(BUILD_SHARED_LIBS OFF)

#needs to be modified per PC
#set(MSYS_ROOT "C:/msys64")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${MSYS_ROOT}/mingw64/cmake/Modules/")

# The main executable
add_executable(main main.cpp)
set_property(TARGET main PROPERTY CXX_STANDARD 11)
target_compile_options(main PRIVATE -Wall)

add_subdirectory(./lib/matplotplusplus)
target_link_libraries(main PUBLIC matplot)