cmake_minimum_required(VERSION 3.10)
project(eys3d_camera VERSION 2.0.0)

# Cross-distro ROS 2 build. Dependencies resolve to the same package
# names across supported distros.
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

# ===== Per-ABI dispatch + native scheduling tune =====
#
# The driver targets two ABIs, matching the prebuilt eSPDI .so set:
#   - x86_64  → libeSPDI_X86_64.so,    SSE2 baseline (ABI-mandated)
#   - aarch64 → libeSPDI_NVIDIA_64.so, NEON baseline (ABI-mandated)
#
# SIMD path selection is done at compile time via the `__aarch64__` /
# `__x86_64__` macros (see src/simd_kernels.cpp). No CMake-side
# branching is required for kernel dispatch.
#
# Flags applied to every build:
#
#   -fno-math-errno -fno-trapping-math
#     Semantically safe on this driver (no errno-after-libm reads, no
#     SIGFPE handler installed); enable broader folding and
#     vectorisation in GCC.
#
#   -mtune=native           (native builds only)
#     Adjusts GCC's scheduler model for the host CPU. The compiler
#     reads /proc/cpuinfo internally and picks the appropriate
#     microarchitecture. The instruction set is unchanged, so the
#     resulting binary remains portable across the ABI; cores that GCC
#     does not recognise fall back to generic tuning automatically.
#     Skipped for cross-compile builds, where the host CPU does not
#     reflect the deployment target — pass an explicit
#     `-DCMAKE_CXX_FLAGS="-mtune=<target>"` on the colcon command line
#     in that case.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-fno-math-errno -fno-trapping-math)
    if(NOT CMAKE_CROSSCOMPILING)
        add_compile_options(-mtune=native)
    endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(eys3d_camera_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# OpenMP — parallelises the depth → XYZ reprojection across image rows.
# Bundled with GCC's runtime (libgomp); no separate apt package needed
# on any standard Linux distro. The reprojection falls back to a serial
# loop when OpenMP support is not detected.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found: parallel point-cloud reprojection enabled")
else()
    message(STATUS "OpenMP not found: point-cloud reprojection will run single-threaded")
endif()

# eSPDI 5.1.0.7 ships in-package under eSPDI/ — header plus prebuilt
# .so for x86_64 and aarch64. The package is self-contained; no
# external SDK installation is required.
set(ESPDI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/eSPDI")

if(NOT EXISTS "${ESPDI_ROOT}/eSPDI.h")
    message(FATAL_ERROR
        "eSPDI headers not found at ${ESPDI_ROOT}. "
        "The SDK should ship in-package under eSPDI/.")
endif()

# Target architecture, not host — cross-compiling from x86_64 to aarch64
# (e.g. for a Jetson image) should pick the aarch64 .so.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR
   CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(ESPDI_LIB_NAME "libeSPDI_X86_64.so")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR
       CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(ESPDI_LIB_NAME "libeSPDI_NVIDIA_64.so")
else()
    message(FATAL_ERROR
        "Unsupported target arch '${CMAKE_SYSTEM_PROCESSOR}'. "
        "eSPDI ships prebuilt libs only for x86_64 and aarch64.")
endif()

# Resolve the versioned SDK binary (libeSPDI_<arch>.so.<x.y.z.w>). The
# tree carries no symlinks, so source archives stay buildable. COMPARE
# NATURAL needs CMake 3.18; Ubuntu 20.04 ships 3.16.
file(GLOB _espdi_libs "${ESPDI_ROOT}/${ESPDI_LIB_NAME}.*")
if(NOT _espdi_libs)
    message(FATAL_ERROR "libeSPDI not found: ${ESPDI_ROOT}/${ESPDI_LIB_NAME}.*")
endif()
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
    list(SORT _espdi_libs COMPARE NATURAL)
else()
    list(SORT _espdi_libs)
    list(LENGTH _espdi_libs _espdi_lib_count)
    if(_espdi_lib_count GREATER 1)
        message(WARNING
            "Several libeSPDI binaries in ${ESPDI_ROOT}; CMake ${CMAKE_VERSION} "
            "cannot order them by version, so the newest may not be picked.")
    endif()
endif()
list(GET _espdi_libs -1 ESPDI_LIB)
message(STATUS "eys3d_camera: linking ${ESPDI_LIB}")

# Emit DT_RPATH (rather than the default DT_RUNPATH) so dlopen still
# resolves linked libraries when LD_LIBRARY_PATH is unset. lld does not
# accept the flag, so skip it there. See the install RPATH section below
# for the corresponding INSTALL_RPATH_USE_LINK_PATH configuration.
if(CMAKE_CXX_COMPILER_LINKER_ID STREQUAL "LLD")
    set(EYS3D_CAMERA_RPATH_FLAGS "")
else()
    set(EYS3D_CAMERA_RPATH_FLAGS "-Wl,--disable-new-dtags")
endif()

# APC error-code name table, generated from the SDK header so runtime
# error messages carry the define name (APC_NOT_SUPPORT_RES) instead of
# a bare rc. Regenerates automatically when the SDK header changes.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(ESPDI_ERROR_INC "${CMAKE_CURRENT_BINARY_DIR}/generated/espdi_error_names.inc")
add_custom_command(
    OUTPUT "${ESPDI_ERROR_INC}"
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_BINARY_DIR}/generated"
    COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_espdi_error_names.py"
            "${ESPDI_ROOT}/eSPDI_def.h" "${ESPDI_ERROR_INC}"
    DEPENDS "${ESPDI_ROOT}/eSPDI_def.h"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_espdi_error_names.py"
    COMMENT "Generating APC error-code name table"
)
add_custom_target(gen_espdi_error_names DEPENDS "${ESPDI_ERROR_INC}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ESPDI_ROOT}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

set(DEPENDENCIES
    ament_index_cpp
    diagnostic_msgs
    diagnostic_updater
    geometry_msgs
    rclcpp
    rclcpp_action
    rclcpp_components
    eys3d_camera_interfaces
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    yaml_cpp_vendor
)

# eYs3D self-calibration (selfk) core, bundled in-package under selfk/: the
# public header plus a static archive per architecture, linked into the
# component. Set -DEYS3D_WITH_SELFCAL=OFF to build without it.
option(EYS3D_WITH_SELFCAL "Build with self-calibration (selfk) support" ON)
if(EYS3D_WITH_SELFCAL)
    set(SELFK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/selfk")
    # Normalise the arch to the name the archives ship under (uname -m:
    # x86_64 / aarch64), matching the eSPDI block, so an arm64/AMD64 alias from a
    # cross toolchain still resolves the archive.
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR
       CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
        set(SELFK_ARCH "x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR
           CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        set(SELFK_ARCH "aarch64")
    else()
        set(SELFK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
    endif()
    set(SELFK_PREBUILT
        "${SELFK_DIR}/lib/${SELFK_ARCH}/libeys3d_selfk_${SELFK_ARCH}.a")
    # Build rules that only apply where the sources they build are present.
    file(GLOB _selfk_local "${CMAKE_CURRENT_SOURCE_DIR}/cmake/local/selfk_*.cmake")
    if(_selfk_local)
        foreach(_rule IN LISTS _selfk_local)
            include("${_rule}")
        endforeach()
    elseif(EXISTS "${SELFK_PREBUILT}")
        add_library(eys3d_selfk STATIC IMPORTED)
        set_target_properties(eys3d_selfk PROPERTIES
            IMPORTED_LOCATION "${SELFK_PREBUILT}"
            INTERFACE_INCLUDE_DIRECTORIES "${SELFK_DIR}/include"
            # The archive was built against the in-package eSPDI; the final .so
            # must resolve those symbols.
            INTERFACE_LINK_LIBRARIES "${ESPDI_LIB}")
    else()
        message(FATAL_ERROR
            "EYS3D_WITH_SELFCAL is ON but no self-calibration archive for "
            "${CMAKE_SYSTEM_PROCESSOR} was found (${SELFK_PREBUILT}). Set "
            "-DEYS3D_WITH_SELFCAL=OFF to build without it, or ask eYs3D for an "
            "archive built for this architecture.")
    endif()
endif()

add_library(eys3d_camera_component SHARED
    src/espdi_device.cpp
    src/camera_node.cpp
    src/hole_filling.cpp
    src/spatial_filter.cpp
    src/register_settings.cpp
    src/selfcal_manager.cpp
    src/simd_kernels.cpp
    src/temporal_filter.cpp
    src/video_modes.cpp
    src/zd_lookup.cpp
)
if(EYS3D_WITH_SELFCAL)
    target_link_libraries(eys3d_camera_component eys3d_selfk)
    target_compile_definitions(eys3d_camera_component PRIVATE EYS3D_WITH_SELFCAL)
    target_include_directories(eys3d_camera_component PRIVATE ${SELFK_DIR}/include)
endif()
target_compile_definitions(eys3d_camera_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
add_dependencies(eys3d_camera_component gen_espdi_error_names)
target_link_libraries(eys3d_camera_component
    ${ESPDI_LIB}    # also exports libjpeg-turbo 2.0.4 (tjDecompress2 etc.)
    Threads::Threads
)
# yaml-cpp 0.7+ provides the namespaced target yaml-cpp::yaml-cpp; the
# bundled Foxy vendor ships 0.6.x which exposes only the bare name.
if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(eys3d_camera_component yaml-cpp::yaml-cpp)
else()
    target_link_libraries(eys3d_camera_component yaml-cpp)
endif()
if(OpenMP_CXX_FOUND)
    target_link_libraries(eys3d_camera_component OpenMP::OpenMP_CXX)
endif()
# Embed the libeSPDI absolute path into the install RPATH;
# INSTALL_RPATH_USE_LINK_PATH adds the link-time library directories so
# ROS libraries resolve without LD_LIBRARY_PATH. See above for the
# --disable-new-dtags rationale.
set_target_properties(eys3d_camera_component PROPERTIES
    # $ORIGIN at install time is the install/lib directory next to the
    # component .so; the SDK .so plus its OpenCL loader stub are installed
    # there too so the binary stays relocatable.
    INSTALL_RPATH "$ORIGIN:$ORIGIN/OpenCL"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE
    LINK_FLAGS "${EYS3D_CAMERA_RPATH_FLAGS}"
)
ament_target_dependencies(eys3d_camera_component ${DEPENDENCIES})

rclcpp_components_register_nodes(eys3d_camera_component "eys3d_camera::CameraNode")

add_executable(camera_node src/camera_node_main.cpp)
target_link_libraries(camera_node eys3d_camera_component)

# Live performance monitor. Standalone executable; subscribes to the
# driver's topics with SerializedMessage callbacks so the per-frame
# rmw → ROS message deserialisation is skipped (the monitor itself
# stays below 1 % of one core even on a quad-A55 host).
add_executable(perf_monitor src/perf_monitor.cpp)
ament_target_dependencies(perf_monitor
    rclcpp diagnostic_msgs sensor_msgs)
# RPATH pinning for the standalone binary, sized to survive
# secure-execution mode (glibc enters this mode when the binary carries
# any file capability; DT_RUNPATH, LD_LIBRARY_PATH, and non-trusted
# $ORIGIN expansion are all disabled there). The absolute install-prefix
# lib path is embedded; the binary is therefore not relocatable.
set_target_properties(camera_node PROPERTIES
    # $ORIGIN here is install/lib/eys3d_camera; the SDK + OpenCL stub live
    # one directory up in install/lib. The install-prefix absolute path
    # is also embedded so the binary still resolves when invoked outside
    # the workspace overlay.
    INSTALL_RPATH "$ORIGIN/..:$ORIGIN/../OpenCL:${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE
    LINK_FLAGS "${EYS3D_CAMERA_RPATH_FLAGS}"
)
ament_target_dependencies(camera_node rclcpp rclcpp_components)

install(TARGETS
    eys3d_camera_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
)

# Install the eSPDI runtime alongside the component .so. The SDK's
# SONAME is version-tagged, so the versioned file alone satisfies
# DT_NEEDED.
install(FILES
    "${ESPDI_LIB}"
    DESTINATION lib
)

# Install the OpenCL loader stub bundled with the SDK (libeSPDI is built
# with NEEDED libOpenCL.so.1). Keep the SDK's $ORIGIN-relative neighbour
# layout — the .so lives in a subdirectory next to the SDK so the SDK's
# internal RPATH ($ORIGIN/OpenCL/<arch>/lib) can still resolve it on
# systems without a system OpenCL implementation.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR
   CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(OPENCL_STUB_DIR "${ESPDI_ROOT}/OpenCL/x86_64/lib")
else()
    set(OPENCL_STUB_DIR "${ESPDI_ROOT}/OpenCL/aarch64/lib")
endif()
# The stub ships as a single plain file named after its SONAME
# (libOpenCL.so.1) — the only name the loader asks for at runtime.
install(FILES "${OPENCL_STUB_DIR}/libOpenCL.so.1"
    DESTINATION lib/OpenCL
)

install(TARGETS
    camera_node
    perf_monitor
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}/
)

# Ship the in-package per-chip register quality cfg files (cfg/DM_Quality_Cfg/)
# to share/eys3d_camera/cfg/DM_Quality_Cfg/ so the launch file can resolve
# them via ament_index_cpp::get_package_share_directory at runtime.
set(DM_QUALITY_CFG_SRC "${CMAKE_CURRENT_SOURCE_DIR}/cfg/DM_Quality_Cfg")
if(EXISTS "${DM_QUALITY_CFG_SRC}")
    install(DIRECTORY "${DM_QUALITY_CFG_SRC}"
        DESTINATION share/${PROJECT_NAME}/cfg)
else()
    message(WARNING
        "DM_Quality_Cfg directory not found at ${DM_QUALITY_CFG_SRC}. "
        "Post-streaming register tuning will be skipped unless "
        "dm_quality_cfg_dir is set explicitly at launch.")
endif()

# Filter tuning profiles (cfg/filter_profiles/*.yaml). Loaded by the
# launch as a ROS 2 params file when `filter_profile:=<name>` is set;
# selects the post-processing filter parameter set without editing
# the launch command line. The enable switches stay on the launch.
set(FILTER_PROFILES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/cfg/filter_profiles")
if(EXISTS "${FILTER_PROFILES_SRC}")
    install(DIRECTORY "${FILTER_PROFILES_SRC}"
        DESTINATION share/${PROJECT_NAME}/cfg)
endif()

# PATTERN __pycache__ EXCLUDE: importing a launch file byte-compiles it beside
# the source, and a wholesale DIRECTORY install would put build-host bytecode
# in the installed package.
install(DIRECTORY launch rviz config urdf udev
    DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE
)

# The three .dae meshes the URDFs reference, by name rather than by directory,
# so nothing else left in meshes/ reaches the installed package.
install(FILES
    meshes/G100P.dae
    meshes/G62.dae
    meshes/R77.dae
    DESTINATION share/${PROJECT_NAME}/meshes
)

# Launch-file check. Needs no camera and no ROS graph, so it runs wherever
# `colcon test` does, including on the customer's own build.
if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_launch_files test/test_launch_files.py
        TIMEOUT 120
    )
endif()

ament_export_include_directories(include)
ament_export_libraries(eys3d_camera_component)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
