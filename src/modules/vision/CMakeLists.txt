add_library(vision OBJECT)

target_sources(vision
    PRIVATE
        src/ArucoCam.cpp
        src/ransac.cpp
)

target_include_directories(vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(ProgramCDFR::vision ALIAS vision)
