add_library(vision OBJECT)

set(VISION_SOURCES
    src/ArucoDetector.cpp
    src/ArucoLocalizer.cpp
    src/Cam.cpp
    src/FeaturesLocalizer.cpp
)

# libcamera : capture native sur Raspberry Pi 5. Le pipeline V4L2 n'expose que
# des trames Bayer brutes (l'ISP PiSP est en amont), que le détecteur ArUco ne
# sait pas décoder ; libcamera pilote capteur -> ISP -> trames utilisables.
# Sélectionné automatiquement pour la compilation croisée ARM, le backend V4L2
# restant utilisé en local (x86_64) pour que les tests tournent sans libcamera
# installée (cf. cmake/aarch64_toolchain.cmake).
if(CMAKE_CROSSCOMPILING)
    list(APPEND VISION_SOURCES src/LibcameraCamera.cpp)
endif()

target_sources(vision
    PRIVATE
        ${VISION_SOURCES}
)

target_include_directories(vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(vision
    PUBLIC
        utils
        external_robot_interfaces
        opencv_deps
        project_compiler_options
)

if(CMAKE_CROSSCOMPILING)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
    if(NOT TARGET PkgConfig::LIBCAMERA)
        message(FATAL_ERROR
            "libcamera introuvable en compilation croisée. Exécutez "
            "./scripts/fetch_arm64_sysroot.sh pour l'ajouter au sysroot ARM64, "
            "puis relancez ./build.sh build_arm.")
    endif()
    target_link_libraries(vision PUBLIC PkgConfig::LIBCAMERA)
    target_compile_definitions(vision PUBLIC CAMERA_USE_LIBCAMERA)

    # Debian/Ubuntu et Raspberry Pi OS ne livrent pas base/event_dispatcher.h ni
    # base/thread.h dans libcamera-dev, alors que le code de capture en a besoin
    # pour faire tourner la boucle d'événements. On ajoute des en-têtes de
    # compatibilité maison uniquement lorsque les officiels manquent.
    find_path(LIBCAMERA_EVENT_DISPATCHER_HEADER
        NAMES libcamera/base/event_dispatcher.h
        PATHS ${LIBCAMERA_INCLUDE_DIRS}
        NO_DEFAULT_PATH)
    if(NOT LIBCAMERA_EVENT_DISPATCHER_HEADER)
        target_include_directories(vision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/compat)
        message(STATUS
            "libcamera/base/event_dispatcher.h absent : ajout des en-têtes de compatibilité maison")
    endif()

    # libcamera 0.2+ regroupe les formats XRGB/ARGB dans le type PixelFormat à
    # la place des anciennes classes ; les tests de format du convertisseur
    # utilisent ce garde.
    if(LIBCAMERA_VERSION VERSION_GREATER_EQUAL "0.2")
        target_compile_definitions(vision PUBLIC CAMERA_LIBCAMERA_HAVE_XRGB8888)
    endif()
    # Les formats 24 bits RGB/BGR ont été renommés RGB24/BGR24 en libcamera 0.3
    # (RGB888/BGR888 en 0.2) : le convertisseur s'adapte via ce garde.
    if(LIBCAMERA_VERSION VERSION_GREATER_EQUAL "0.3")
        target_compile_definitions(vision PUBLIC CAMERA_LIBCAMERA_FORMAT_RGB24)
    endif()
endif()

add_library(ProgramCDFR::vision ALIAS vision)
