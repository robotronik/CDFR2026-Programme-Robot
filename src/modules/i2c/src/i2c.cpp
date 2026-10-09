#include "i2c/i2c.hpp"
#include <stdio.h>
#include <string.h> // Include for memcpy
#include "utils/logger.hpp"

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
}

int I2COpenSlave(int slave_address){
    int adapter_nr = 1; /* probably dynamically determined */
    char filename[20];
    int i2cFile = -1;

    if (slave_address == -1) // I2C Emulation
    {
        LOG_INFO("Emulating I2C");
        return -1;
    }
    else{
        snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
        i2cFile = open(filename, O_RDWR);
        if (i2cFile < 0) {
            /* ERROR HANDLING; you can check errno to see what went wrong */
            throw std::runtime_error("Couldn't open I2C file");
        }

        if (ioctl(i2cFile, I2C_SLAVE, slave_address) < 0) {
            close(i2cFile);
            LOG_ERROR("ioctl error");
            throw std::runtime_error("Couldn't ioctl I2C file");
        }
        LOG_GREEN_INFO("Opened I2C file for slave ", slave_address);
    }
    return i2cFile;
}

int I2cSendData (int f, uint8_t command, uint8_t* data, int length, int tries){
    if (f >= 0){
        for (int i = 0; i < tries; i++){
            if(length != 0){
                if (0 == i2c_smbus_write_i2c_block_data(f, command, length, data))
                    return 0;
            }
            else{
                if (0 == i2c_smbus_write_byte(f, command))
                    return 0;
            }
        }
        LOG_ERROR("I2C Send Data failed after ", tries, " tries");
        return -1;
    }
    else
        return 0;
}


int I2cReceiveData (int f, uint8_t command, uint8_t* data, int length, int tries){
    if (f >= 0){
        for (int i = 0; i < tries; i++){
            if (i2c_smbus_write_byte(f, command))
                continue;        
            usleep(200); // wait for sensor to finish loading the output buffer data
            i2c_smbus_read_byte(f);
            if (read(f, data, length) != length)
                continue;
            return 0;
        }
        LOG_ERROR("I2C Send Data failed after ", tries, " tries");
        return -1;
    }
    else{
        // Emulate I2C by return data full of 0x00
        for (int i = 0; i < length; i++){
            data[i] = 0x00;
        }
        return 0;
    }
}

int I2cSendBlockReceiveData (int f, uint8_t command, uint8_t* data, int length, uint8_t* out_data, int out_length, int tries){
    if (f >= 0){
        for (int i = 0; i < tries; i++){
            if(length != 0){
                if (i2c_smbus_write_i2c_block_data(f, command, length, data))
                    continue;
            }
            else{
                if (i2c_smbus_write_byte(f, command))
                    continue;
            }
            usleep(200); // wait for sensor to finish loading the output buffer data
            if (read(f, out_data, out_length) != out_length)
                continue;
            return 0;
        }
        LOG_ERROR("I2C Send Data failed after ", tries, " tries");
        return -1;
    }
    else{
        // Emulate I2C by return data full of 0x00
        for (int i = 0; i < out_length; i++){
            out_data[i] = 0x00;
        }
        return 0;
    }
}

// Pointer to an array pointer
int8_t ReadInt8(uint8_t *buffer[]){
    int8_t val;
    memcpy(&val, *buffer, 1);
    *buffer += 1;
    return val;
}

int16_t ReadInt16(uint8_t *buffer[]){
    int16_t val;
    memcpy(&val, *buffer, 2);
    *buffer += 2;
    return val;
}

int32_t ReadInt32(uint8_t *buffer[]){
    int32_t val;
    memcpy(&val, *buffer, 4);
    *buffer += 4;
    return val;
}


int64_t ReadInt64(uint8_t *buffer[]){
    int64_t val;
    memcpy(&val, *buffer, 8);
    *buffer += 8;
    return val;
}

uint8_t ReadUInt8(uint8_t *buffer[]){
    uint8_t val;
    memcpy(&val, *buffer, 1);
    *buffer += 1;
    return val;
}

uint16_t ReadUInt16(uint8_t *buffer[]){
    uint16_t val;
    memcpy(&val, *buffer, 2);
    *buffer += 2;
    return val;
}

uint32_t ReadUInt32(uint8_t *buffer[]){
    uint32_t val;
    memcpy(&val, *buffer, 4);
    *buffer += 4;
    return val;
}


uint64_t ReadUInt64(uint8_t *buffer[]){
    uint64_t val;
    memcpy(&val, *buffer, 8);
    *buffer += 8;
    return val;
}

// Writes val at buffer and increments the buffer
void WriteInt8(uint8_t *buffer[], int8_t val){
    memcpy(*buffer, &val, 1);
    *buffer += 1;
}

void WriteInt16(uint8_t *buffer[], int16_t val){
    memcpy(*buffer, &val, 2);
    *buffer += 2;
}

void WriteInt32(uint8_t *buffer[], int32_t val){
    memcpy(*buffer, &val, 4);
    *buffer += 4;
}

void WriteInt64(uint8_t *buffer[], int64_t val){
    memcpy(*buffer, &val, 8);
    *buffer += 8;
}
void WriteUInt8(uint8_t *buffer[], uint8_t val){
    memcpy(*buffer, &val, 1);
    *buffer += 1;
}

void WriteUInt16(uint8_t *buffer[], uint16_t val){
    memcpy(*buffer, &val, 2);
    *buffer += 2;
}

void WriteUInt32(uint8_t *buffer[], uint32_t val){
    memcpy(*buffer, &val, 4);
    *buffer += 4;
}

void WriteUInt64(uint8_t *buffer[], uint64_t val){
    memcpy(*buffer, &val, 8);
    *buffer += 8;
}