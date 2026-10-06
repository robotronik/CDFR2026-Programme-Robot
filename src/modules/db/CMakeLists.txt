add_library(db OBJECT)

target_sources(db
    PRIVATE
        src/ActionDurationDB.cpp
)

target_include_directories(db
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(db
    PUBLIC
        utils
        project_compiler_options
        SQLite3::SQLite3
)

add_library(ProgramCDFR::db ALIAS db)
