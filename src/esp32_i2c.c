// Target specific I2C functions
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "zmod4xxx_types.h"

#define I2C_MASTER_SCL_IO           CONFIG_ZMOD4510_SCL_IO      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_ZMOD4510_SDA_IO      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              CONFIG_ZMOD4510_I2C_MASTER_NUM                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          CONFIG_ZMOD4510_I2C_MASTER_FREQ_HZ /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define ZMOD4510_EN_PIN CONFIG_ZMOD4510_ENABLE_PIN

static const char* TAG = "ESP32_I2C_HAL";

esp_err_t esp32_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf,
                      uint8_t len)
{
	esp_err_t err;
	err = i2c_master_write_to_device(I2C_MASTER_NUM,i2c_addr,&reg_addr,1,I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS);
	if(err != ESP_OK) return err;

	vTaskDelay(10/portTICK_PERIOD_MS);

	err = i2c_master_read_from_device(I2C_MASTER_NUM, i2c_addr, buf, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

	return err;
}

esp_err_t esp32_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf,
                       uint8_t len)
{
    int ret;
    uint8_t write_buf[1 + len];
    write_buf[0] = reg_addr;
    for(int i = 1;i<=len;i++){
    	write_buf[i] = buf[i-1];
    }
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, i2c_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

void reset_zmod4510(){
	gpio_set_level(ZMOD4510_EN_PIN, 0);
	vTaskDelay(500/portTICK_PERIOD_MS);
	gpio_set_level(ZMOD4510_EN_PIN, 1);
}

void esp32_sleep(uint32_t ms){
	vTaskDelay(ms/portTICK_PERIOD_MS);
}

esp_err_t init_hardware(zmod4xxx_dev_t *dev)
{
    int i2c_master_port = I2C_MASTER_NUM;

    ESP_LOGI(TAG, "Installing driver on SDA pin %d and SCL pin %d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    ESP_LOGI(TAG, "Resetting sensor for initialization");
    gpio_set_direction(ZMOD4510_EN_PIN, GPIO_MODE_OUTPUT);

    reset_zmod4510();

    dev->write = esp32_i2c_write;
	dev->read = esp32_i2c_read;
	dev->delay_ms = esp32_sleep;
	dev->reset = reset_zmod4510;

    ESP_LOGI(TAG, "Installing I2C driver");
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


esp_err_t deinit_hardware()
{
	return i2c_driver_delete(I2C_MASTER_NUM);
}
