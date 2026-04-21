cmake_minimum_required(VERSION 3.15)
project(StudioModsNative LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build as shared library (DLL on Windows)
add_library(StudioModsNative SHARED
    src/pbd_solver.cpp
    src/spatial_hash.cpp
    src/cloth_solver.cpp
    src/sdf_builder.cpp
)

target_include_directories(StudioModsNative PUBLIC include)

# Enable AVX2 + fast math on MSVC
if(MSVC)
    # Flags que queremos en ambos (arquitectura)
    target_compile_options(StudioModsNative PRIVATE /arch:AVX2)

    # Flags solo para configuraciones de optimización (Release, RelWithDebInfo, etc.)
    target_compile_options(StudioModsNative PRIVATE 
        $<$<NOT:$<CONFIG:Debug>>:/O2 /fp:fast /GL>
    )

    # El Link Time Code Generation también debe ser solo para Release
    target_link_options(StudioModsNative PRIVATE 
        $<$<NOT:$<CONFIG:Debug>>:/LTCG>
    )
else()
    target_compile_options(StudioModsNative PRIVATE -mavx2 -mfma)
    target_compile_options(StudioModsNative PRIVATE $<$<NOT:$<CONFIG:Debug>>:-O3 -ffast-math>)
endif()

# Output to the game Plugins folder or local bin
set_target_properties(StudioModsNative PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set(UNITY_PLUGIN_DEST "D:/Documents/CAARP/StudioNEOV2_Data/Plugins")

install(TARGETS StudioModsNative
        RUNTIME DESTINATION "${UNITY_PLUGIN_DEST}"
        LIBRARY DESTINATION "${UNITY_PLUGIN_DEST}"
)