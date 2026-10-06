# Toolchain for cross-compiling to AArch64 (Raspberry Pi).
# The aarch64-linux-gnu cross toolchain and arm64 multiarch packages
# (e.g. libsqlite3-dev:arm64) are provided by docker/Dockerfile.arm64.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Target libraries live in the multiarch directory /usr/lib/aarch64-linux-gnu,
# which the compiler and CMake's default search paths already cover.

add_compile_definitions(__CROSS_COMPILE_ARM__)
