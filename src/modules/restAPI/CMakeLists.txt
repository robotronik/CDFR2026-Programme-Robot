add_library(restAPI OBJECT)

target_sources(restAPI
    PRIVATE
        src/restAPI.cpp
        src/manual_mode.cpp
)

target_include_directories(restAPI
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(ProgramCDFR::restAPI ALIAS mat)
