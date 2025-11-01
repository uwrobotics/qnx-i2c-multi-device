/**
 * @file i2c_host.h
 * @brief I2C device host interface for QNX
 *
 * Provides a C++ class interface for communicating with I2C slave devices
 * on the QNX platform. Supports both direct and memory-addressed I2C operations.
 *
 * @todo Support interrupt-based service
 * @see https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.prog/topic/inthandler_Thread_or_Event.html
 *
 * @note For driver details, refer to:
 * https://www.qnx.com/developers/docs/qnxeverywhere/com.qnx.doc.qnxeverywhere/topic/lpg/driver_I2C_interfacing.html
 */

#ifndef I2C_HOST_H
#define I2C_HOST_H

#include <variant>
#include <public/i2c_flag.h>

/**
 * @class I2CDevice
 * @brief Interface for communicating with I2C slave devices
 *
 * This class manages the connection to an I2C slave device and provides
 * methods for reading and writing data. It handles the underlying file
 * descriptor management and mutex synchronization.
 *
 * Example usage:
 * @code
 * // Create device with direct polling mode
 * I2CDevice sensor(1, 0x50, I2C_DIRECT);  // Bus 1, slave address 0x50, direct mode
 *
 * // Direct read
 * uint8_t buffer[4];
 * direct_access_t direct = {.buf = buffer, .len = 4};
 * std::variant<direct_access_t, mem_access_t> data = direct;
 * sensor.read(data);
 *
 * // Memory-addressed write
 * uint8_t write_buf[] = {0xAA, 0xBB};
 * mem_access_t mem = {.addr = 0x10, .buf = write_buf, .size = 2};
 * std::variant<direct_access_t, mem_access_t> write_data = mem;
 * sensor.write(write_data);
 *
 * sensor.smbus_cleanup();
 * @endcode
 */
class I2CDevice {
    public:
        /**
         * @brief Constructs an I2CDevice object and connects to the specified slave
         *
         * @param bus_number The I2C bus number (e.g., 0 for /dev/i2c0, 1 for /dev/i2c1)
         * @param slave_addr The 7-bit I2C slave device address
         * @param mode The device operation mode (I2C_DIRECT, I2C_MEM, or I2C_Listen)
         */
        I2CDevice(uint8_t bus_number, uint8_t slave_addr, device_type_t mode);

        /**
         * @brief Destroys the I2CDevice object and releases resources
         *
         * Automatically disconnects from the device and releases the file descriptor.
         */
        ~I2CDevice();

        /**
         * @brief Reads data from the I2C slave device
         *
         * Supports two access modes:
         * - direct_access_t: Direct read without register addressing
         * - mem_access_t: Read from a specific internal register/memory address
         *
         * @param[in,out] data Variant containing either direct_access_t or mem_access_t
         *                     The buffer will be filled with data read from the device
         * @return i2c_status_t I2C_SUCCESS on success, error code otherwise
         */
        i2c_status_t read(std::variant<direct_access_t, mem_access_t> &data);

        /**
         * @brief Writes data to the I2C slave device
         *
         * Supports two access modes:
         * - direct_access_t: Direct write without register addressing
         * - mem_access_t: Write to a specific internal register/memory address
         *
         * @param[in] data Variant containing either direct_access_t or mem_access_t
         *                 with the data to be written
         * @return i2c_status_t I2C_SUCCESS on success, error code otherwise
         */
        i2c_status_t write(const std::variant<direct_access_t, mem_access_t> &data);

        /**
         * @brief Cleans up I2C device resources
         *
         * Closes the I2C device file descriptor and releases any held mutexes.
         * Should be called after completing I2C operations to free resources.
         *
         * @return i2c_status_t I2C_SUCCESS on success, I2C_ERROR_CLEANING_UP on failure
         */
        i2c_status_t smbus_cleanup();

        /**
         * @brief Retrieves the slave device address
         *
         * @return uint16_t The configured slave address
         */
        uint16_t getSlaveAddress() const;

        /**
         * @brief Interrupt callback for I2C master receive
         *
         * This callback is invoked when an I2C master receive a packet
         * in interrupt-driven mode (I2C_Listen). It should be called by the interrupt
         * handler to process received data.
         *
         * @note Only applicable when device is configured with I2C_Listen mode
         * @see device_type_t::I2C_Listen
         */
        void *I2CDevice::I2C_MasterRxCallback(void* arg);

    private:
        uint8_t _bus_number;        /**< I2C bus number */
        i2c_slave_t _slave;         /**< Slave device configuration */
        pthread_t _tid;  // Interrupt Thread To BE Implemented(TODO)


        /**
         * @brief Establishes connection to the I2C device
         *
         * Opens the I2C bus device file and configures the slave address.
         *
         * @return i2c_status_t I2C_SUCCESS on success, error code otherwise
         */
        i2c_status_t connect();

        /**
         * @brief Disconnects from the I2C device
         *
         * Closes the file descriptor and releases resources.
         *
         * @return i2c_status_t I2C_SUCCESS on success, error code otherwise
         */
        i2c_status_t disconnect();
};

#endif // I2C_HOST_H
