add_library(vision OBJECT)

target_sources(vision
    PRIVATE
        src/ArucoDetector.cpp
        src/ArucoLocalizer.cpp
        src/Cam.cpp
        src/FeaturesLocalizer.cpp
)

# libcamera capture is ARM-only: the x86_64 build uses the V4L2 backend so it runs
# without libcamera installed.
if(CMAKE_CROSSCOMPILING)
    target_sources(vision PRIVATE src/LibcameraCamera.cpp)
endif()

target_include_directories(vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# OpenCV: native ArUco marker detection and frame handling.
if(NOT DEFINED OpenCV_DIR AND NOT CMAKE_CROSSCOMPILING)
    file(GLOB _opencv_config_dirs
        "/usr/lib/*/cmake/opencv4"
        "/usr/lib/cmake/opencv4"
        "/usr/local/lib/cmake/opencv4"
        "/usr/include/opencv4/")
    if(_opencv_config_dirs)
        list(GET _opencv_config_dirs 0 _opencv_config_dir)
        set(OpenCV_DIR "${_opencv_config_dir}" CACHE PATH "OpenCV CMake config directory")
    endif()
endif()
find_package(OpenCV REQUIRED CONFIG)
target_include_directories(vision PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(vision
    PUBLIC
        utils
        external_robot_interfaces
        project_compiler_options
        Threads::Threads
        ${OpenCV_LIBS}
)

# libcamera: native capture on the Raspberry Pi 5 (OV9281 behind the PiSP ISP,
# whose frames the V4L2 backend cannot decode).
if(CMAKE_CROSSCOMPILING)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
    if(NOT TARGET PkgConfig::LIBCAMERA)
        message(FATAL_ERROR "libcamera not found for the ARM target (install libcamera-dev:arm64)")
    endif()
    target_link_libraries(vision PUBLIC PkgConfig::LIBCAMERA)
    target_compile_definitions(vision PUBLIC CAMERA_USE_LIBCAMERA)

    # Debian/Ubuntu and Raspberry Pi OS do not ship base/event_dispatcher.h nor
    # base/thread.h in libcamera-dev; provide local compatibility headers when the
    # official ones are missing.
    find_path(LIBCAMERA_EVENT_DISPATCHER_HEADER
        NAMES libcamera/base/event_dispatcher.h
        PATHS ${LIBCAMERA_INCLUDE_DIRS}
        NO_DEFAULT_PATH)
    if(NOT LIBCAMERA_EVENT_DISPATCHER_HEADER)
        target_include_directories(vision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/compat)
    endif()

    # libcamera 0.2+ groups the XRGB/ARGB formats into PixelFormat; 0.3 renames the
    # 24-bit RGB/BGR formats to RGB24/BGR24. The frame converter adapts via these.
    if(LIBCAMERA_VERSION VERSION_GREATER_EQUAL "0.2")
        target_compile_definitions(vision PUBLIC CAMERA_LIBCAMERA_HAVE_XRGB8888)
    endif()
    if(LIBCAMERA_VERSION VERSION_GREATER_EQUAL "0.3")
        target_compile_definitions(vision PUBLIC CAMERA_LIBCAMERA_FORMAT_RGB24)
    endif()

    # OpenCV/SQLite are the only target libraries in the sysroot; their own
    # dependencies are resolved at runtime on the Pi, so allow unresolved symbols
    # in linked shared objects (unresolved symbols in our own objects stay errors).
    target_link_options(vision INTERFACE -Wl,--allow-shlib-undefined)
endif()
