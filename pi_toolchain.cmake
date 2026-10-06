set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CROSS_COMPILE_PREFIX aarch64-linux-gnu)

set(CMAKE_C_COMPILER ${CROSS_COMPILE_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PREFIX}-g++)
set(CMAKE_LINKER aarch64-linux-gnu-ld)

# The compiler's own sysroot, plus /usr, so Debian/Ubuntu multiarch libraries
# (installed under /usr/lib/<triplet>, e.g. libsqlite3) are found by CMake.
set(CMAKE_FIND_ROOT_PATH /usr/${CROSS_COMPILE_PREFIX} /usr)

# OpenCV and SQLite arm64 cannot be co-installed with their amd64 counterparts:
# libopencv-dev is not Multi-Arch:same, so dpkg refuses to install the arm64
# flavour next to the amd64 one. scripts/fetch_arm64_sysroot.sh instead
# downloads the arm64 .debs and extracts them into a local sysroot, pointed to
# by the ARM64_SYSROOT environment variable (see build.sh).
if(DEFINED ENV{ARM64_SYSROOT})
    set(CMAKE_FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH} $ENV{ARM64_SYSROOT})
endif()

# libcamera (Raspberry Pi 5 native capture, cf. src/vision/LibcameraCamera.cpp)
# is located through pkg-config, which looks in the host directories by default
# and would miss the ARM64 sysroot. Point it at the sysroot's .pc files and have
# it prefix the emitted -I/-L paths with the sysroot, since the .pc files use
# the absolute prefix=/usr of the target filesystem.
if(DEFINED ENV{ARM64_SYSROOT})
    set(_arm_pc_dirs "$ENV{ARM64_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig"
                     "$ENV{ARM64_SYSROOT}/usr/share/pkgconfig")
    string(REPLACE ";" ":" _arm_pc_path "${_arm_pc_dirs}")
    set(ENV{PKG_CONFIG_PATH} "${_arm_pc_path}")
    set(ENV{PKG_CONFIG_LIBDIR} "${_arm_pc_path}")
    set(ENV{PKG_CONFIG_SYSROOT_DIR} "$ENV{ARM64_SYSROOT}")
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_definitions(-D__CROSS_COMPILE_ARM__)
