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

#include <iostream>


/** @brief Maximum number of I2C buses supported by this driver */
#define MAX_I2C_BUSES       8

/** @brief IRQ number for I2C interrupt handling (platform-specific) */
#define IRQ_NUM 149

/**
 * @brief File descriptors for I2C device connections
 *
 * Array of file descriptors, one per I2C bus. Initialized to -1 (disconnected).
 */
static int i2c_fd[MAX_I2C_BUSES] = { -1, -1, -1, -1, -1, -1, -1, -1};

/**
 * @brief Mutex for thread-safe I2C operations
 *
 * Ensures exclusive access to I2C bus resources across multiple threads.
 */
static pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief I2C receive data message structure
 *
 * Used with DCMD_I2C_SENDRECV devctl command to read data from I2C devices.
 * Memory is allocated dynamically to accommodate variable-length data.
 */
struct i2c_recv_data_msg_t
{
    i2c_sendrecv_t hdr;     /**< I2C send/receive header with slave info and lengths */
    uint8_t bytes[];        /**< Flexible array member for received data bytes (C99) */
};

/**
 * @brief I2C send data message structure
 *
 * Used with DCMD_I2C_SEND devctl command to write data to I2C devices.
 * Memory is allocated dynamically to accommodate variable-length data.
 */
struct i2c_send_data_msg_t
{
    i2c_send_t hdr;         /**< I2C send header with slave info and data length */
    uint8_t bytes[];        /**< Flexible array member for data bytes to send (C99) */
};


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
    _slave.info.addr = slave_addr;
    _slave.info.fmt = I2C_ADDRFMT_7BIT;
    _slave.mode = mode;

    // Establish connection to I2C device
    if (connect() != I2C_SUCCESS) {
        fprintf(stderr, "Failed to connect to I2C device on bus %d, addr 0x%02X\n",
                _bus_number, _slave.info.addr);
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
 * @return i2c_state_t I2C_SUCCESS on success, I2C_ERROR_NOT_CONNECTED on failure
 */
i2c_state_t I2CDevice::connect()
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
 * @return i2c_state_t I2C_SUCCESS on success, I2C_ERROR_CLEANING_UP on failure
 */
i2c_state_t I2CDevice::disconnect()
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
 * @return i2c_state_t I2C_SUCCESS on success, error code otherwise
 */
i2c_state_t I2CDevice::read(std::variant<direct_access_t, mem_access_t> &data)
{
    if (this->connect())
    {
        perror("open_i2c_fd");
        return I2C_ERROR_NOT_CONNECTED;
    }

    const int MIN_READ_SIZE = 1;

    // Handle direct read (no register address)
    if (std::holds_alternative<direct_access_t>(data)) {
        auto& mem = std::get<direct_access_t>(data);

        if(mem.size < MIN_READ_SIZE) {
            mem.size = MIN_READ_SIZE;
        }

        struct i2c_recv_data_msg_t *msg = NULL;

        msg = (struct i2c_recv_data_msg_t*)malloc(sizeof(struct i2c_recv_data_msg_t) + mem.size); // allocate enough memory for both the calling information and received data
        if (!msg)
        {
            perror("alloc failed");
            return I2C_ERROR_ALLOC_FAILED;
        }

        // Assign the I2C device and format of message
        msg->hdr.slave.addr = _slave.info.addr;
        msg->hdr.slave.fmt = _slave.info.fmt;
        msg->hdr.send_len = 0; // no register to send
        msg->hdr.recv_len = mem.size;
        msg->hdr.stop = 1;

        // Send the I2C message
        int status, err; // status information about the devctl() call
        err = devctl(i2c_fd[_bus_number], DCMD_I2C_SENDRECV, msg, sizeof(*msg) + mem.size, (&status));
        if (err != EOK)
        {
            free(msg);
            fprintf(stderr, "error with devctl: %s\n", strerror(err));
            return I2C_ERROR_OPERATION_FAILED;
        }

        // return the read data
        memcpy(mem.buf, msg->bytes, mem.size);

        // Free allocated message
        free(msg);

        return I2C_SUCCESS;
    }
    // Handle memory-addressed read (with register address)
    else if (std::holds_alternative<mem_access_t>(data)) {
        auto& mem = std::get<mem_access_t>(data);

        if(mem.size < MIN_READ_SIZE) {
            mem.size = MIN_READ_SIZE;
        }

        struct i2c_recv_data_msg_t *msg = NULL;

        msg = (struct i2c_recv_data_msg_t*)malloc(sizeof(struct i2c_recv_data_msg_t) + mem.size); // allocate enough memory for both the calling information and received data
        if (!msg)
        {
            perror("alloc failed");
            return I2C_ERROR_ALLOC_FAILED;
        }

            // Assign the I2C device and format of message
        msg->hdr.slave.addr = _slave.info.addr;
        msg->hdr.slave.fmt = _slave.info.fmt;
        msg->hdr.send_len = 1; //send addr
        msg->hdr.recv_len = mem.size;
        msg->hdr.stop = 1;

        msg->bytes[0] = mem.addr;

        // Send the I2C message
        int status, err; // status information about the devctl() call
        err = devctl(i2c_fd[_bus_number], DCMD_I2C_SENDRECV, msg, sizeof(struct i2c_recv_data_msg_t) + mem.size, (&status));
        if (err != EOK)
        {
            free(msg);
            fprintf(stderr, "error with devctl: %s\n", strerror(err));
            return I2C_ERROR_OPERATION_FAILED;
        }

        // return the read data
        memcpy(mem.buf, msg->bytes, mem.size);

        // Free allocated message
        free(msg);

        return I2C_SUCCESS;
    } else {
        perror("error_cmd");
        return I2C_ERROR_NOT_CONNECTED;
    }
}

/**
 * @brief Writes data to the I2C slave device
 *
 * Supports two access modes:
 * - direct_access_t: Direct write without register addressing
 * - mem_access_t: Write to a specific internal register/memory address
 *
 * @param[in] data Variant containing either direct_access_t or mem_access_t
 * @return i2c_state_t I2C_SUCCESS on success, error code otherwise
 */
i2c_state_t I2CDevice::write(const std::variant<direct_access_t, mem_access_t> &data)
{
    if (this->connect())
    {
        perror("open_i2c_fd");
        return I2C_ERROR_NOT_CONNECTED;
    }

    const int MIN_MEM_WRITE_SIZE = 2; //Reg+Data
    const int MIN_WRITE_SIZE = 1; //Data

    // Handle direct write (no register address)
    if (std::holds_alternative<direct_access_t>(data)) {
        auto& mem = std::get<direct_access_t>(data);

        if(mem.size < MIN_WRITE_SIZE) {
            perror("write_size_err");
            return I2C_ERROR_OPERATION_FAILED;
        }

        // Allocate memory for the message
        struct i2c_send_data_msg_t *msg = NULL;


        msg = (struct i2c_send_data_msg_t*)malloc(sizeof(struct i2c_send_data_msg_t) + mem.size); // allocate enough memory for both the calling information and received data        
        if (!msg)
        {
            perror("alloc failed");
            return I2C_ERROR_ALLOC_FAILED;
        }

        // Assign the I2C device and format of message
        msg->hdr.slave.addr = _slave.info.addr;
        msg->hdr.slave.fmt = _slave.info.fmt;
        msg->hdr.len = mem.size;
        msg->hdr.stop = 1;

        // Add the write data
        memcpy(msg->bytes, mem.buf, mem.size);


        // Send the I2C message
        int status, err; // status information about the devctl() call
        err = devctl(i2c_fd[_bus_number], DCMD_I2C_SEND, msg, sizeof(struct i2c_send_data_msg_t) + mem.size, (&status));
        if (err != EOK)
        {
            free(msg);
            fprintf(stderr, "error with devctl: %s\n", strerror(err));
            return I2C_ERROR_OPERATION_FAILED;
        }

        // Free allocated message
        free(msg);

        return I2C_SUCCESS;
    }
    // Handle memory-addressed write (with register address)
    else if (std::holds_alternative<mem_access_t>(data)) {
        auto& mem = std::get<mem_access_t>(data);

        if(mem.size < MIN_MEM_WRITE_SIZE) {
            perror("write_size_err");
            return I2C_ERROR_OPERATION_FAILED;
        }

        const size_t MEM_ADDR_SIZE = 1;

        // Allocate memory for the message
        struct i2c_send_data_msg_t *msg = NULL;


        msg = (struct i2c_send_data_msg_t*)malloc(sizeof(struct i2c_send_data_msg_t) + MEM_ADDR_SIZE + mem.size); // allocate enough memory for both the calling information and received data        
        if (!msg)
        {
            perror("alloc failed");
            return I2C_ERROR_ALLOC_FAILED;
        }

        // Assign the I2C device and format of message
        msg->hdr.slave.addr = _slave.info.addr;
        msg->hdr.slave.fmt = _slave.info.fmt;
        msg->hdr.len = MEM_ADDR_SIZE+mem.size; //addr + data
        msg->hdr.stop = 1;

        // Add the write data
        msg->bytes[0] = mem.addr;
        memcpy(&msg->bytes[MEM_ADDR_SIZE], mem.buf, mem.size);

        // Send the I2C message
        int status, err; // status information about the devctl() call
        err = devctl(i2c_fd[_bus_number], DCMD_I2C_SEND, msg, sizeof(struct i2c_send_data_msg_t) + MEM_ADDR_SIZE + mem.size, (&status));
        if (err != EOK)
        {
            free(msg);
            fprintf(stderr, "error with devctl: %s\n", strerror(err));
            return I2C_ERROR_OPERATION_FAILED;
        }

        // Free allocated message
        free(msg);

        return I2C_SUCCESS;
    } else {
        perror("error_cmd");
        return I2C_ERROR_NOT_CONNECTED;
    }
}

/**
 * @brief Cleans up I2C device resources
 *
 * Closes the I2C device file descriptor and releases any held mutexes.
 * This is typically called when done with all I2C operations.
 *
 * @return i2c_state_t I2C_SUCCESS on success, I2C_ERROR_CLEANING_UP on failure
 */
i2c_state_t I2CDevice::smbus_cleanup()
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
    return _slave.info.addr;
}

/**
 * @todo Implement thread-based interrupt support for I2C_Listen mode
 * @see I2C_MasterRxCallback
 */