cmake_minimum_required(VERSION 3.15)

#-----------------------------------------
# RPLIDAR SDK
#-----------------------------------------
set(RPLIDAR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk")

add_library(rplidar_sdk STATIC
    "${RPLIDAR_DIR}/sdk/src/sl_lidar_driver.cpp"
    "${RPLIDAR_DIR}/sdk/src/hal/thread.cpp"
    "${RPLIDAR_DIR}/sdk/src/sl_crc.cpp"
    "${RPLIDAR_DIR}/sdk/src/sl_serial_channel.cpp"
    "${RPLIDAR_DIR}/sdk/src/sl_lidarprotocol_codec.cpp"
    "${RPLIDAR_DIR}/sdk/src/sl_async_transceiver.cpp"
    "${RPLIDAR_DIR}/sdk/src/sl_tcp_channel.cpp"
    "${RPLIDAR_DIR}/sdk/src/sl_udp_channel.cpp"
    "${RPLIDAR_DIR}/sdk/src/rplidar_driver.cpp"
    "${RPLIDAR_DIR}/sdk/src/dataunpacker/dataunpacker.cpp"
    "${RPLIDAR_DIR}/sdk/src/dataunpacker/unpacker/handler_capsules.cpp"
    "${RPLIDAR_DIR}/sdk/src/dataunpacker/unpacker/handler_hqnode.cpp"
    "${RPLIDAR_DIR}/sdk/src/dataunpacker/unpacker/handler_normalnode.cpp"
    "${RPLIDAR_DIR}/sdk/src/arch/linux/net_serial.cpp"
    "${RPLIDAR_DIR}/sdk/src/arch/linux/net_socket.cpp"
    "${RPLIDAR_DIR}/sdk/src/arch/linux/timer.cpp"
)

target_include_directories(rplidar_sdk PUBLIC
    "${RPLIDAR_DIR}/sdk/include"
    "${RPLIDAR_DIR}/sdk/src"
)

target_include_directories(rplidar_sdk PRIVATE
    "${RPLIDAR_DIR}/sdk/src/dataunpacker"
    "${RPLIDAR_DIR}/sdk/src/dataunpacker/unpacker"
)

target_link_libraries(rplidar_sdk PUBLIC Threads::Threads)
target_compile_definitions(rplidar_sdk PRIVATE _CPU_ENDIAN_LITTLE)

#-----------------------------------------
# libi2c (prebuilt static library, per architecture)
#-----------------------------------------
add_library(i2c_lib STATIC IMPORTED GLOBAL)
if(CMAKE_CROSSCOMPILING)
    set_target_properties(i2c_lib PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/aarch64-linux-gnu/libi2c.a"
    )
else()
    set_target_properties(i2c_lib PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/x86_64-linux-gnu/libi2c.a"
    )
endif()
