/**
 * @file i2c_flag.h
 * @brief I2C device communication types and status codes
 *
 * Defines the data structures and return codes used for I2C device
 * communication in the QNX environment.
 */

#ifndef I2C_FLAG_H
#define I2C_FLAG_H

#include <stdint.h>
#include <stddef.h>
#include <hw/i2c.h>

/**
 * @brief Return codes for I2C client API operations
 *
 * Status codes returned by I2C operations to indicate success or failure.
 */
typedef enum {
    I2C_SUCCESS = 0,                    /**< Operation completed successfully */
    I2C_ERROR_NOT_CONNECTED = -1,       /**< Device not connected or invalid file descriptor */
    I2C_ERROR_ALLOC_FAILED = -2,        /**< Memory allocation failed */
    I2C_ERROR_OPERATION_FAILED = -3,    /**< I2C read/write operation failed */
    I2C_ERROR_CLEANING_UP = -4          /**< Error during cleanup (fd close or mutex release) */
} i2c_state_t;

/**
 * @brief I2C device operation modes
 *
 * Defines the communication mode for I2C device operations.
 * The mode determines how the device will handle data transfer.
 */
typedef enum {
    I2C_DIRECT,     /**< Direct polling mode - synchronous read/write operations */
    I2C_MEM,        /**< Memory-addressed mode - read/write data from specific memory addresses */
    I2C_Listen      /**< Interrupt-driven mode - enables interrupt for active listening */
} device_type_t;

/**
 * @brief I2C slave device address configuration
 *
 * Describes the slave device address, addressing format, and operation mode.
 *
 * Example initialization:
 * @code
 * i2c_slave_t slave = {
 *     .addr = 0x50,
 *     .fmt = I2C_ADDRFMT_7BIT,
 *     .mode = I2C_DIRECT
 * };
 * @endcode
 */
typedef struct {
    i2c_addr_t    info;
    device_type_t mode;     /**< Device operation mode: I2C_DIRECT, I2C_MEM, or I2C_Listen */
} i2c_slave_t;

/**
 * @brief Direct I2C data access structure
 *
 * Used for direct read/write operations without internal register addressing.
 * Typical use case: reading raw data from simple I2C sensors.
 *
 * Initialize with: direct_access_t data = {.buf = buffer, .size = 4};
 */
typedef struct {
    uint8_t *buf;           /**< Pointer to data buffer for read/write */
    size_t size;            /**< Number of bytes to read/write */
} direct_access_t;

/**
 * @brief Memory-addressed I2C access structure
 *
 * Used for read/write operations to specific internal registers or memory addresses.
 * Typical use case: accessing registers in I2C EEPROMs or sensor configuration registers.
 *
 * Initialize with: mem_access_t data = {.addr = 0x10, .buf = buffer, .size = 2};
 */
typedef struct {
    uint8_t addr;           /**< Internal register/memory address within the device */
    uint8_t *buf;           /**< Pointer to data buffer for read/write */
    size_t size;            /**< Number of bytes to read/write */
} mem_access_t;

#endif // I2C_FLAG_H