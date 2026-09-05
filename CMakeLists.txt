cmake_minimum_required(VERSION 3.20)
project(hesim3d_core LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# Options
option(HESIM3D_BUILD_GUI "Build Dear ImGui interactive visualizer" ON)

include(FetchContent)
find_package(Threads REQUIRED)

# -----------------------------------------------------------------------------
# 1. Python & nanobind
# -----------------------------------------------------------------------------
find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(nanobind CONFIG REQUIRED)

# -----------------------------------------------------------------------------
# 2. Eigen3 (Header-only)
# -----------------------------------------------------------------------------
FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()
set(EIGEN3_INCLUDE_DIR ${eigen_SOURCE_DIR})

# -----------------------------------------------------------------------------
# 3. Google Filament (Prebuilt Linux binaries)
# -----------------------------------------------------------------------------
set(FILAMENT_VERSION "v1.76.0")
FetchContent_Declare(
    filament_pkg
    URL https://github.com/google/filament/releases/download/${FILAMENT_VERSION}/filament-${FILAMENT_VERSION}-linux.tgz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(filament_pkg)

set(FILAMENT_INCLUDE_DIRS
    "${filament_pkg_SOURCE_DIR}/filament/include"
    "${filament_pkg_SOURCE_DIR}/include"
)
set(FILAMENT_LIB_DIRS
    "${filament_pkg_SOURCE_DIR}/filament/lib/x86_64"
    "${filament_pkg_SOURCE_DIR}/lib/x86_64"
)

link_directories(
    ${FILAMENT_LIB_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d"
)

set(FILAMENT_LIBS_LIST
    gltfio
    gltfio_core
    filament
    backend
    bluevk
    bluegl
    filamat
    filabridge
    filaflat
    geometry
    utils
    camutils
    stb
    uberarchive
    uberzlib
    zstd
    abseil
    perfetto
    shaders
    smol-v
    dracodec
    basis_transcoder
    ktxreader
    image
    ibl
)

# -----------------------------------------------------------------------------
# 4. GLFW, Dear ImGui & ImPlot (for GUI Module)
# -----------------------------------------------------------------------------
set(GUI_SOURCES "")
set(GUI_LIBS "")
set(GUI_INCLUDES "")

if(HESIM3D_BUILD_GUI)
    # GLFW
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(glfw)

    # ImGui
    FetchContent_Declare(
        imgui
        URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.8.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(imgui)

    # ImPlot
    FetchContent_Declare(
        implot
        URL https://github.com/epezent/implot/archive/refs/tags/v0.16.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(implot)

    set(GUI_INCLUDES
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
        ${implot_SOURCE_DIR}
        ${glfw_SOURCE_DIR}/include
    )

    set(GUI_SOURCES
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
        ${implot_SOURCE_DIR}/implot.cpp
        ${implot_SOURCE_DIR}/implot_items.cpp
        src/cpp/gui/gui_app.cpp
        src/cpp/gui/viewports.cpp
        src/cpp/gui/control_panels.cpp
        src/cpp/gui/native_dialogs.cpp
    )

    set(GUI_LIBS glfw ${CMAKE_DL_LIBS} GL)
    add_compile_definitions(HESIM3D_ENABLE_GUI=1)
    add_compile_definitions(IMGUI_USE_WCHAR32=1)
endif()

# -----------------------------------------------------------------------------
# 5. Core Source Files
# -----------------------------------------------------------------------------
set(CORE_SOURCES
    src/cpp/bindings.cpp
    src/cpp/spline_se3.cpp
    src/cpp/filament_renderer.cpp
    src/cpp/gltf_loader.cpp
    src/cpp/lighting_manager.cpp
    ${GUI_SOURCES}
)

nanobind_add_module(
    _hesim3d_core
    NB_STATIC
    ${CORE_SOURCES}
)

target_compile_options(_hesim3d_core PRIVATE
    "-nostdinc++"
    "-isystem" "${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/libcxx_include"
)

target_include_directories(_hesim3d_core PRIVATE
    src/cpp
    ${EIGEN3_INCLUDE_DIR}
    ${FILAMENT_INCLUDE_DIRS}
    ${GUI_INCLUDES}
)

target_link_libraries(_hesim3d_core PRIVATE
    -Wl,--start-group
    ${FILAMENT_LIBS_LIST}
    -Wl,--end-group
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d/libc++.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d/libc++abi.so"
    ${GUI_LIBS}
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

target_link_options(_hesim3d_core PRIVATE
    "-Wl,-z,noexecstack"
)

set_target_properties(_hesim3d_core PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)


# Linux system libraries for graphics
if(UNIX AND NOT APPLE)
    find_package(X11 REQUIRED)
    target_link_libraries(_hesim3d_core PRIVATE
        ${X11_LIBRARIES}
        pthread
        dl
    )
endif()

# Install extension and bundled libc++ into Python package folder
install(TARGETS _hesim3d_core DESTINATION hesim3d)
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d/libc++.so.1"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d/libc++.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d/libc++abi.so.1"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d/libc++abi.so"
    DESTINATION hesim3d
)

# Install packaged runtime assets (fonts, presets, builtin scenes)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts" DESTINATION hesim3d/assets_data)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/assets/scenes" DESTINATION hesim3d/assets_data)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/assets/sensor_presets" DESTINATION hesim3d/assets_data)

add_custom_command(TARGET _hesim3d_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:_hesim3d_core> "${CMAKE_CURRENT_SOURCE_DIR}/src/hesim3d/"
    COMMENT "Auto-syncing _hesim3d_core to src/hesim3d/"
)

