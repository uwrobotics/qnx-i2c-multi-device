#include<i2c_host.h>
#include<iostream>

int main() {
    I2CDevice device = I2CDevice(1, 0X4A, I2C_MEM);

    mem_access_t data;

    uint8_t payload[3] = {5,5,5};

    data.addr = 0X00;
    data.buf = payload;
    data.size=3;

    std::variant<direct_access_t, mem_access_t> meta_data = data;

    std::cout << "Return Code:" << (int)device.read(meta_data) << std::endl;

    std::cout << "Mem Read Result: 0x" << std::hex << (int)data.buf[0] << " " << std::hex << (int)data.buf[1] << " " << std::hex << (int)data.buf[2] << std::endl;

    // Tested with logic analyzer result
    return 0;
}