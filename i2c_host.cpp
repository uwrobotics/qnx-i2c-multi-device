/**
 * @file i2c_host.cpp
 * @brief Implementation of I2CDevice class for QNX I2C communication
 *
 * Based on QNX hardware component samples for Raspberry Pi I2C operations.
 * Uses devctl() for I2C communication with slave devices.
 */

#include "i2c_host.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <devctl.h>
#include <pthread.h>


#define MAX_I2C_BUSES       8
#define IRQ_NUM 149

// File descriptor for I2C device
static int i2c_fd[MAX_I2C_BUSES] = { -1, -1, -1, -1, -1, -1, -1, -1};
// Mutex for thread-safe I2C operations
static pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief Constructor - initializes I2C device and establishes connection
 *
 * @param bus_number The I2C bus number (0 = /dev/i2c0, 1 = /dev/i2c1, etc.)
 * @param slave_addr The 7-bit I2C slave device address
 * @param mode The device operation mode (I2C_DIRECT, I2C_MEM, or I2C_Listen)
 */
I2CDevice::I2CDevice(uint8_t bus_number, uint8_t slave_addr, device_type_t mode)
    : _bus_number(bus_number)
{
    // Initialize slave configuration
    _slave.addr = slave_addr;
    _slave.fmt = I2C_ADDRFMT_7BIT;
    _slave.mode = mode;

    // Establish connection to I2C device
    if (connect() != I2C_SUCCESS) {
        fprintf(stderr, "Failed to connect to I2C device on bus %d, addr 0x%02X\n",
                _bus_number, _slave.addr);
    }
}

/**
 * @brief Destructor - cleans up resources and closes I2C connection
 */
I2CDevice::~I2CDevice()
{
    disconnect();
}

/**
 * @brief Establishes connection to the I2C device
 *
 * Opens the I2C bus device file (/dev/i2cN) and prepares for communication.
 *
 * @return i2c_status_t I2C_SUCCESS on success, I2C_ERROR_NOT_CONNECTED on failure
 */
i2c_status_t I2CDevice::connect()
{
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c%d", _bus_number);

    pthread_mutex_lock(&i2c_mutex);

    // Open I2C device with read/write permissions
    i2c_fd[_bus_number] = open(dev_path, O_RDWR);
    if (i2c_fd[_bus_number] == -1) {
        pthread_mutex_unlock(&i2c_mutex);
        fprintf(stderr, "Failed to open %s: %s\n", dev_path, strerror(errno));
        return I2C_ERROR_NOT_CONNECTED;
    }

    pthread_mutex_unlock(&i2c_mutex);
    return I2C_SUCCESS;
}

/**
 * @brief Disconnects from the I2C device
 *
 * Closes the file descriptor and releases resources.
 *
 * @return i2c_status_t I2C_SUCCESS on success, I2C_ERROR_CLEANING_UP on failure
 */
i2c_status_t I2CDevice::disconnect()
{
    pthread_mutex_lock(&i2c_mutex);

    if (i2c_fd[_bus_number] != -1) {
        if (close(i2c_fd[_bus_number]) == -1) {
            pthread_mutex_unlock(&i2c_mutex);
            fprintf(stderr, "Failed to close I2C device: %s\n", strerror(errno));
            return I2C_ERROR_CLEANING_UP;
        }
        i2c_fd[_bus_number] = -1;
    }

    pthread_mutex_unlock(&i2c_mutex);
    return I2C_SUCCESS;
}

/**
 * @brief Reads data from the I2C slave device
 *
 * Supports two access modes:
 * - direct_access_t: Direct read without register addressing
 * - mem_access_t: Read from a specific internal register/memory address
 *
 * @param[in,out] data Variant containing either direct_access_t or mem_access_t
 * @return i2c_status_t I2C_SUCCESS on success, error code otherwise
 */
i2c_status_t I2CDevice::read(std::variant<direct_access_t, mem_access_t> &data)
{
    // TODO: Redo the implementation, there should not be mutex lock and need to open the i2c FD and then handle the logic
    pthread_mutex_lock(&i2c_mutex);

    if (i2c_fd[_bus_number] == -1) {
        pthread_mutex_unlock(&i2c_mutex);
        return I2C_ERROR_NOT_CONNECTED;
    }

    i2c_status_t status = I2C_SUCCESS;

    // Handle direct read (no register address)
    if (std::holds_alternative<direct_access_t>(data)) {
        auto& direct = std::get<direct_access_t>(data);

        // Allocate I2C send structure for direct read
        size_t msg_size = sizeof(i2c_send_t) + direct.len;
        i2c_send_t *msg = (i2c_send_t *)malloc(msg_size);
        if (!msg) {
            pthread_mutex_unlock(&i2c_mutex);
            return I2C_ERROR_ALLOC_FAILED;
        }

        // Configure message for receive operation
        msg->slave.addr = _slave.addr;
        msg->slave.fmt = _slave.fmt;
        msg->len = direct.len;
        msg->stop = 1;

        // Perform I2C read operation
        if (devctl(i2c_fd, DCMD_I2C_SEND, msg, msg_size, NULL) != EOK) {
            fprintf(stderr, "I2C direct read failed: %s\n", strerror(errno));
            status = I2C_ERROR_OPERATION_FAILED;
        } else {
            // Copy received data to buffer
            memcpy(direct.buf, msg->buf, direct.len);
        }

        free(msg);
    }
    // Handle memory-addressed read (with register address)
    else if (std::holds_alternative<mem_access_t>(data)) {
        auto& mem = std::get<mem_access_t>(data);

        // Allocate I2C sendrecv structure for write-then-read operation
        size_t msg_size = sizeof(i2c_sendrecv_t) + sizeof(uint32_t) + mem.size;
        i2c_sendrecv_t *msg = (i2c_sendrecv_t *)malloc(msg_size);
        if (!msg) {
            pthread_mutex_unlock(&i2c_mutex);
            return I2C_ERROR_ALLOC_FAILED;
        }

        // Configure message for write register address, then read data
        msg->slave.addr = _slave.addr;
        msg->slave.fmt = _slave.fmt;
        msg->send_len = sizeof(uint32_t);  // Send register address
        msg->recv_len = mem.size;          // Receive data bytes
        msg->stop = 1;

        // Copy register address to send buffer
        memcpy(msg->buf, &mem.addr, sizeof(uint32_t));

        // Perform I2C write-then-read operation
        if (devctl(i2c_fd, DCMD_I2C_SENDRECV, msg, msg_size, NULL) != EOK) {
            fprintf(stderr, "I2C memory read failed: %s\n", strerror(errno));
            status = I2C_ERROR_OPERATION_FAILED;
        } else {
            // Copy received data (after the sent register address)
            memcpy(mem.buf, msg->buf + msg->send_len, mem.size);
        }

        free(msg);
    }

    pthread_mutex_unlock(&i2c_mutex);
    return status;
}

/**
 * @brief Writes data to the I2C slave device
 *
 * Supports two access modes:
 * - direct_access_t: Direct write without register addressing
 * - mem_access_t: Write to a specific internal register/memory address
 *
 * @param[in] data Variant containing either direct_access_t or mem_access_t
 * @return i2c_status_t I2C_SUCCESS on success, error code otherwise
 */
i2c_status_t I2CDevice::write(const std::variant<direct_access_t, mem_access_t> &data)
{
    pthread_mutex_lock(&i2c_mutex);

    if (i2c_fd[_bus_number] == -1) {
        pthread_mutex_unlock(&i2c_mutex);
        return I2C_ERROR_NOT_CONNECTED;
    }

    i2c_status_t status = I2C_SUCCESS;

    // Handle direct write (no register address)
    if (std::holds_alternative<direct_access_t>(data)) {
        auto& direct = std::get<direct_access_t>(data);

        // Allocate I2C send structure
        size_t msg_size = sizeof(i2c_send_t) + direct.len;
        i2c_send_t *msg = (i2c_send_t *)malloc(msg_size);
        if (!msg) {
            pthread_mutex_unlock(&i2c_mutex);
            return I2C_ERROR_ALLOC_FAILED;
        }

        // Configure message for send operation
        msg->slave.addr = _slave.addr;
        msg->slave.fmt = _slave.fmt;
        msg->len = direct.len;
        msg->stop = 1;

        // Copy data to send buffer
        memcpy(msg->buf, direct.buf, direct.len);

        // Perform I2C write operation
        if (devctl(i2c_fd, DCMD_I2C_SEND, msg, msg_size, NULL) != EOK) {
            fprintf(stderr, "I2C direct write failed: %s\n", strerror(errno));
            status = I2C_ERROR_OPERATION_FAILED;
        }

        free(msg);
    }
    // Handle memory-addressed write (with register address)
    else if (std::holds_alternative<mem_access_t>(data)) {
        auto& mem = std::get<mem_access_t>(data);

        // Allocate I2C send structure (register address + data)
        size_t msg_size = sizeof(i2c_send_t) + sizeof(uint32_t) + mem.size;
        i2c_send_t *msg = (i2c_send_t *)malloc(msg_size);
        if (!msg) {
            pthread_mutex_unlock(&i2c_mutex);
            return I2C_ERROR_ALLOC_FAILED;
        }

        // Configure message for send operation
        msg->slave.addr = _slave.addr;
        msg->slave.fmt = _slave.fmt;
        msg->len = sizeof(uint32_t) + mem.size;  // Register address + data
        msg->stop = 1;

        // Copy register address and data to send buffer
        memcpy(msg->buf, &mem.addr, sizeof(uint32_t));
        memcpy(msg->buf + sizeof(uint32_t), mem.buf, mem.size);

        // Perform I2C write operation
        if (devctl(i2c_fd, DCMD_I2C_SEND, msg, msg_size, NULL) != EOK) {
            fprintf(stderr, "I2C memory write failed: %s\n", strerror(errno));
            status = I2C_ERROR_OPERATION_FAILED;
        }

        free(msg);
    }

    pthread_mutex_unlock(&i2c_mutex);
    return status;
}

/**
 * @brief Cleans up I2C device resources
 *
 * Closes the I2C device file descriptor and releases any held mutexes.
 * This is typically called when done with all I2C operations.
 *
 * @return i2c_status_t I2C_SUCCESS on success, I2C_ERROR_CLEANING_UP on failure
 */
i2c_status_t I2CDevice::smbus_cleanup()
{
    return disconnect();
}

/**
 * @brief Retrieves the configured slave device address
 *
 * @return uint16_t The slave address (7-bit or 10-bit)
 */
uint16_t I2CDevice::getSlaveAddress() const
{
    return _slave.addr;
}
