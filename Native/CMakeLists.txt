cmake_minimum_required(VERSION 3.15)
project(StudioModsNative LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build as shared library (DLL on Windows)
add_library(StudioModsNative SHARED
    src/pbd_solver.cpp
    src/spatial_hash.cpp
    src/api.cpp
    src/cloth_solver.cpp
    src/sdf_builder.cpp
)

target_include_directories(StudioModsNative PUBLIC include)

# Enable AVX2 + fast math on MSVC
if(MSVC)
    target_compile_options(StudioModsNative PRIVATE /arch:AVX2 /O2 /fp:fast /GL)
    target_link_options(StudioModsNative PRIVATE /LTCG)
else()
    target_compile_options(StudioModsNative PRIVATE -mavx2 -mfma -O3 -ffast-math)
endif()

# Output to the game Plugins folder or local bin
set_target_properties(StudioModsNative PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
