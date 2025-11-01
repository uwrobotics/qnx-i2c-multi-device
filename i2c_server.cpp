#include<i2c_host.h>
#include<iostream>

int main() {
    I2CDevice device = I2CDevice(1, 0X4A, I2C_MEM);

    mem_access_t data;

    uint8_t payload[1];

    data.addr = 0X00;
    data.buf = payload;
    data.size=1;

    std::variant<direct_access_t, mem_access_t> meta_data = data;

    device.read(meta_data);

    std::cout << data.buf[0] << std::endl;

    return 0;
}