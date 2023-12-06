/*
 * esp32_i2c.h
 *
 *  Created on: 14 nov 2023
 *      Author: Franco
 */
#include <stdint.h>
#include "esp_err.h"
#include "zmod4xxx_types.h"

#ifndef ZMOD4510_ESP32_I2C_H_
#define ZMOD4510_ESP32_I2C_H_


/**
 * @brief Sleep for some time. Depending on target and application this can \n
 *        be used to go into power down or to do task switching.
 * @param [in] ms will sleep for at least this number of milliseconds
 */
void esp32_sleep(uint32_t ms);

/* I2C communication */
/**
 * @brief Read a register over I2C using protocol described in Renesas ZMOD4xxx \n
 *        Datasheet section "I2C Interface and Data Transmission Protocol"
 * @param [in] i2c_addr 7-bit I2C slave address of the ZMOD45xx
 * @param [in] reg_addr address of internal register to read
 * @param [out] buf destination buffer; must have at least a size of len*uint8_t
 * @param [in] len number of bytes to read
 * @return error code
 */
esp_err_t esp32_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf,
                      uint8_t len);

/**
 * @brief Write a register over I2C using protocol described in Renesas ZMOD4xxx \n
 *        Datasheet section "I2C Interface and Data Transmission Protocol"
 * @param [in] i2c_addr 7-bit I2C slave address of the ZMOD4xxx
 * @param [in] reg_addr address of internal register to write
 * @param [in] buf source buffer; must have at least a size of len*uint8_t
 * @param [in] len number of bytes to write
 * @return error code
 */
esp_err_t esp32_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf,
                       uint8_t len);

void reset_zmod4510(void);

/**
 * @brief   Initialize the target hardware
 * @param   [in] dev pointer to the device
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
esp_err_t init_hardware(zmod4xxx_dev_t *dev);

/**
 * @brief   deinitialize target hardware
 * @return  error code
 * @retval  0 success
 * @retval  "!= 0" error
 */
esp_err_t deinit_hardware();



#endif /* COMPONENTS_ZMOD4510_SRC_ESP32_I2C_H_ */
