#include<inc/i2c_host.h>
#include<iostream>

#include <sys/json.h>
#include <sys/neutrino.h>

int main() {
    // Note: multiple slave device should share the same bus ID and do not need to register the bus again.


    // Note: input msg will be in unordered map
    I2CDevice device = I2CDevice(1, 0X4A, I2C_MEM);

    mem_access_t data;

    uint8_t payload[4] = {5,5,5};

    data.addr = 0X00;
    data.buf = payload;
    data.size=4;

    std::variant<direct_access_t, mem_access_t> meta_data = data;

    std::cout << "R Return Code:" << (int)device.read(meta_data) << std::endl;

    std::cout << "Mem Read Result: 0x" << std::hex << (int)data.buf[0] << " " << std::hex << (int)data.buf[1] << " " << std::hex << (int)data.buf[2] << std::endl;

    uint8_t word[4] = {'t', 'h', 'i', 's'};
    data.buf = word;
    data.size=1;

    meta_data = data;

    std::cout << "W Return Code:" << (int)device.write(meta_data) <<std::endl;

    // Tested with logic analyzer result
    return 0;
}