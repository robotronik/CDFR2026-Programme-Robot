set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CROSS_COMPILE_PREFIX aarch64-linux-gnu)

set(CMAKE_C_COMPILER ${CROSS_COMPILE_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PREFIX}-g++)

# Sysroot ARM64 (OpenCV, SQLite, libcamera) fourni par
# scripts/fetch_arm64_sysroot.sh. On accepte SYSROOT (nom du preset CMake) et
# ARM64_SYSROOT (nom historique de build.sh) pour désigner ce sysroot.
if(DEFINED ENV{ARM64_SYSROOT})
    set(_arm_sysroot "$ENV{ARM64_SYSROOT}")
elseif(DEFINED ENV{SYSROOT})
    set(_arm_sysroot "$ENV{SYSROOT}")
endif()

# The compiler's own sysroot, plus /usr, so Debian/Ubuntu multiarch libraries
# (installed under /usr/lib/<triplet>, e.g. libsqlite3) are found by CMake.
set(CMAKE_FIND_ROOT_PATH /usr/${CROSS_COMPILE_PREFIX} /usr)

# OpenCV, SQLite and libcamera arm64 cannot be co-installed with their amd64
# counterparts (libopencv-dev is not Multi-Arch:same), so the fetch script
# extracts them into a local sysroot searched here.
if(_arm_sysroot)
    list(APPEND CMAKE_FIND_ROOT_PATH "${_arm_sysroot}")
endif()

# libcamera (Raspberry Pi 5 native capture, cf.
# src/modules/vision/src/LibcameraCamera.cpp) is located through pkg-config,
# which looks in the host directories by default and would miss the ARM64
# sysroot. Point it at the sysroot's .pc files and have it prefix the emitted
# -I/-L paths with the sysroot, since the .pc files use the absolute prefix=/usr
# of the target filesystem.
if(_arm_sysroot)
    set(_arm_pc_dirs "${_arm_sysroot}/usr/lib/aarch64-linux-gnu/pkgconfig"
                     "${_arm_sysroot}/usr/share/pkgconfig")
    string(REPLACE ";" ":" _arm_pc_path "${_arm_pc_dirs}")
    set(ENV{PKG_CONFIG_PATH} "${_arm_pc_path}")
    set(ENV{PKG_CONFIG_LIBDIR} "${_arm_pc_path}")
    set(ENV{PKG_CONFIG_SYSROOT_DIR} "${_arm_sysroot}")
endif()

# Strictly isolate target searches from host filesystem
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_compile_definitions(__CROSS_COMPILE_ARM__)
