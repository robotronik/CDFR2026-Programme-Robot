#pragma once

#include <cstdint>
#include "drive_interface.h"
#include "i2c/I2CDevice.hpp"

/*
class Asserv : public asservissement_interface, public I2CDevice {
    using I2CDevice::I2CDevice;
public:
    Asserv(int slave_address);

    // Explicit override (optional but helps clarify intent)
    void I2cSendData(uint8_t command, uint8_t* data, int length) override {
        // Call I2CDevice's implementation
        I2CDevice::I2cSendData(command, data, length);
    }

    void I2cReceiveData(uint8_t command, uint8_t* data, int length) override {
        // Call I2CDevice's implementation
        I2CDevice::I2cReceiveData(command, data, length);
    }
};
*/