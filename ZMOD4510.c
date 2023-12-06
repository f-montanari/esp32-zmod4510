#include <stdio.h>
#include <stdint.h>

#include "ZMOD4510.h"
#include "zmod4510_config_oaq2.h"
#include "zmod4xxx.h"
#include "oaq_2nd_gen.h"
#include "esp32_i2c.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

static const char* TAG = "ZMOD4510_MAIN";

// TODO: Shouldn't keep these variables in global scope. Ideally I'd create a structure for these
// like the API does. Should find a better way to refactor these structures (and maybe avoid
// including it in the outer zmod4510_dev_t structure) while keeping code integrity.

int8_t ret;
zmod4xxx_dev_t dev;

/* Sensor specific variables */
uint8_t zmod4xxx_status;
uint8_t track_number[ZMOD4XXX_LEN_TRACKING];
uint8_t adc_result[ZMOD4510_ADC_DATA_LEN];
uint8_t prod_data[ZMOD4510_PROD_DATA_LEN];
oaq_2nd_gen_handle_t algo_handle;
oaq_2nd_gen_results_t algo_results;
oaq_2nd_gen_inputs_t algo_input;

float zmod4510_humidity = 50.0;
float zmod4510_temperature = 25.0;
zmod4510_dev_t* zmod4510_device;


void handle_error(zmod4510_dev_t* device)
{
	device->status = SENSOR_ERROR;
	ret = deinit_hardware();
	if (ret) {
		ESP_LOGI(TAG,"Error %d during deinitializing hardware, exiting program!\n",
			   ret);
		return;
	}
}


#ifdef CONFIG_ZMOD4510_ENABLE_INTERRUPT
#define ESP_INTR_FLAG_DEFAULT 0
static QueueHandle_t gpio_evt_queue = NULL;
#endif

#ifdef CONFIG_ZMOD4510_ENABLE_INTERRUPT

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue,&gpio_num,NULL);
}

static void zmod4510_task_handle_irq(void* arg)
{
	uint32_t ionum;
	for(;;)
	{
		// Is this our interrupt?
		if(xQueueReceive(gpio_evt_queue, &ionum, portMAX_DELAY) && (ionum & CONFIG_ZMOD4510_ENABLE_PIN))
		{
			ESP_LOGI(TAG, "IRQ received, getting data...");

			/* Read sensor ADC output. */
			ret = zmod4xxx_read_adc_result(&dev, adc_result);
			if (ret) {
				ESP_LOGI(TAG, "Error %d during reading of ADC results, exiting program!\n",
					   ret);
				handle_error(zmod4510_device);
			}

			/*
			* Check validity of the ADC results. For more information, read the
			* Programming Manual, section "Error Codes".
			*/
			ret = zmod4xxx_check_error_event(&dev);
			if (ret) {
				ESP_LOGI(TAG, "Error during reading status register (%d)\n", ret);
				handle_error(zmod4510_device);
			}

			algo_input.adc_result = adc_result;
			/*
			* The ambient compensation needs humidity [RH] and temperature [DegC]
			* measurements! Input them here.
			*/
			algo_input.humidity_pct = zmod4510_humidity;
			algo_input.temperature_degc = zmod4510_temperature;

			/* Calculate algorithm results */
			ret = calc_oaq_2nd_gen(&algo_handle, &dev, &algo_input, &algo_results);
			/* Skip 900 stabilization samples for oaq_2nd_gen algorithm. */
			if ((ret != OAQ_2ND_GEN_OK) && (ret != OAQ_2ND_GEN_STABILIZATION)) {
				ESP_LOGI(TAG, "Error %d during calculating algorithm, exiting program!\n",
					   ret);
				handle_error(zmod4510_device);

			} else {
				ESP_LOGI(TAG, "*********** Measurements ***********\n");
				for (int i = 0; i < 8; i++) {
					ESP_LOGI(TAG, " Rmox[%d] = ", i);
					ESP_LOGI(TAG, "%.3f kOhm\n", algo_results.rmox[i] / 1e3);
				}
				ESP_LOGI(TAG, " O3_conc_ppb = %6.3f\n", algo_results.O3_conc_ppb);
				ESP_LOGI(TAG, " Fast AQI = %i\n", algo_results.FAST_AQI);
				ESP_LOGI(TAG, " EPA AQI = %i\n", algo_results.EPA_AQI);
				if (ret == OAQ_2ND_GEN_STABILIZATION) {
					zmod4510_device->status = SENSOR_WARMING_UP;
					ESP_LOGI(TAG, "Warm-Up!\n");
				} else {
					zmod4510_device->status = SENSOR_READY;
					ESP_LOGI(TAG, "Valid!\n");
				}
				ESP_LOGI(TAG, "************************************\n");

			}
		}
	}
}


void zmod4510_force_read()
{
	uint32_t i = CONFIG_ZMOD4510_ENABLE_PIN;
	zmod4510_task_handle_irq((void*) &i);
}


void zmod4510_enable_interrupts()
{

	// Based on the interrupt example from:
	// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.pin_bit_mask = (1ULL << CONFIG_ZMOD4510_INT_GPIO);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
	gpio_set_intr_type(CONFIG_ZMOD4510_ENABLE_PIN, GPIO_INTR_NEGEDGE);

	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	//start gpio task
	xTaskCreate(zmod4510_task_handle_irq, "zmod4510_task_handle_irq", 2048, NULL, 10, NULL);
}


void zmod4510_start_irq_measurement(zmod4510_dev_t* device, float humidity, float temperature)
{

	zmod4510_device = device;
	zmod4510_humidity = humidity;
	zmod4510_temperature = temperature;

	device->status = SENSOR_MEASURING;

	/* Start a measurement. */
	ret = zmod4xxx_start_measurement(&dev);
	if (ret) {
		ESP_LOGI(TAG, "Error %d during starting measurement, exiting program!\n",
			   ret);
		handle_error(zmod4510_device);
	}
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(CONFIG_ZMOD4510_INT_GPIO, gpio_isr_handler, (void*) CONFIG_ZMOD4510_INT_GPIO);
}

#endif
/*
 * Use this method for polling data from the sensor. It takes temperature in Celsius degrees and humidity in percentage.
 */
void zmod4510_update_sensor_data(zmod4510_dev_t* device, float temperature, float humidity){

	device->status = SENSOR_MEASURING;

	/* Start a measurement. */
	ret = zmod4xxx_start_measurement(&dev);
	if (ret) {
		ESP_LOGI(TAG, "Error %d during starting measurement, exiting program!\n",
			   ret);
		handle_error(device);
	}
	/* Perform delay. Required to keep proper measurement timing. */
	dev.delay_ms(ZMOD4510_OAQ2_SAMPLE_TIME);

	/* Verify completion of measurement sequence. */
	ret = zmod4xxx_read_status(&dev, &zmod4xxx_status);
	if (ret) {
		ESP_LOGI(TAG, "Error %d during reading sensor status, exiting program!\n",
			   ret);
		handle_error(device);
	}
	/* Check if measurement is running. */
	if (zmod4xxx_status & STATUS_SEQUENCER_RUNNING_MASK) {
		/*
		* Check if reset during measurement occured. For more information,
		* read the Programming Manual, section "Error Codes".
		*/

		ret = zmod4xxx_check_error_event(&dev);
		ESP_LOGI(TAG, "Current ZMOD status: %d", zmod4xxx_status);
		ESP_LOGI(TAG, "Current ERR status: %d", ret);
		switch (ret) {
		case ERROR_POR_EVENT:
			ESP_LOGI(TAG,
				"Measurement completion fault. Unexpected sensor reset.\n");
			break;
		case ZMOD4XXX_OK:
			ESP_LOGI(TAG,
				"Measurement completion fault. Wrong sensor setup.\n");
			break;
		default:
			ESP_LOGI(TAG, "Error during reading status register (%d)\n", ret);
			break;
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
		handle_error(device);
		return;
	}


	/* Read sensor ADC output. */
	ret = zmod4xxx_read_adc_result(&dev, adc_result);
	if (ret) {
		ESP_LOGI(TAG, "Error %d during reading of ADC results, exiting program!\n",
			   ret);
		handle_error(device);
	}

	/*
	* Check validity of the ADC results. For more information, read the
	* Programming Manual, section "Error Codes".
	*/
	ret = zmod4xxx_check_error_event(&dev);
	if (ret) {
		ESP_LOGI(TAG, "Error during reading status register (%d)\n", ret);
		handle_error(device);
	}

	algo_input.adc_result = adc_result;
	/*
	* The ambient compensation needs humidity [RH] and temperature [DegC]
	* measurements! Input them here.
	*/
	algo_input.humidity_pct = humidity;
	algo_input.temperature_degc = temperature;

	/* Calculate algorithm results */
	ret = calc_oaq_2nd_gen(&algo_handle, &dev, &algo_input, &algo_results);
	/* Skip 900 stabilization samples for oaq_2nd_gen algorithm. */
	if ((ret != OAQ_2ND_GEN_OK) && (ret != OAQ_2ND_GEN_STABILIZATION)) {
		ESP_LOGI(TAG, "Error %d during calculating algorithm, exiting program!\n",
			   ret);
		handle_error(device);

	} else {
		ESP_LOGI(TAG, "*********** Measurements ***********\n");
		for (int i = 0; i < 8; i++) {
			ESP_LOGI(TAG, " Rmox[%d] = ", i);
			ESP_LOGI(TAG, "%.3f kOhm\n", algo_results.rmox[i] / 1e3);
		}
		ESP_LOGI(TAG, " O3_conc_ppb = %6.3f\n", algo_results.O3_conc_ppb);
		ESP_LOGI(TAG, " Fast AQI = %i\n", algo_results.FAST_AQI);
		ESP_LOGI(TAG, " EPA AQI = %i\n", algo_results.EPA_AQI);
		if (ret == OAQ_2ND_GEN_STABILIZATION) {
			device->status = SENSOR_WARMING_UP;
			ESP_LOGI(TAG, "Warm-Up!\n");
		} else {
			device->status = SENSOR_READY;
			ESP_LOGI(TAG, "Valid!\n");
		}
		ESP_LOGI(TAG, "************************************\n");

	}

}

void zmod4510_init_sensor(zmod4510_dev_t* device)
{
	ESP_LOGI(TAG, "Initializing hardware");
	vTaskDelay(2000/portTICK_PERIOD_MS);
	ret = init_hardware(&dev);
	if (ret) {
		ESP_LOGI(TAG,"Error %d during initialize hardware, exiting program!\n", ret);
		handle_error(device);
	}

	ESP_LOGI(TAG, "Correctly initialized hardware");

#ifdef CONFIG_ZMOD4510_ENABLE_INTERRUPT
	zmod4510_enable_interrupts();
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
#endif


	/* Sensor related data */
	dev.i2c_addr = ZMOD4510_I2C_ADDR;
	dev.pid = ZMOD4510_PID;
	dev.init_conf = &zmod_oaq2_sensor_cfg[INIT];
	dev.meas_conf = &zmod_oaq2_sensor_cfg[MEASUREMENT];
	dev.prod_data = prod_data;

	/* Read product ID and configuration parameters. */
	ret = zmod4xxx_read_sensor_info(&dev);
	if (ret) {
		ESP_LOGI(TAG,"Error %d during reading sensor information, exiting program!\n",
			   ret);
		handle_error(device);
	}

	ESP_LOGI(TAG,"Correctly read sensor information");
	ESP_LOGI(TAG,"Product ID: %u", dev.pid);

	device->pid = dev.pid;

	ret = zmod4xxx_read_tracking_number(&dev, track_number);
	if(ret){
		ESP_LOGI(TAG,"Error %d during reading sensor tracking number, exiting program!\n",
			   ret);
		handle_error(device);
	}

	printf("Tracking number: 0x0000");
	for (uint8_t i = 0; i< sizeof(track_number);i++)
	{
		printf("%x",track_number[i]);
	}

	printf("\n");

	/* Determine calibration parameters and configure measurement. */
	ret = zmod4xxx_prepare_sensor(&dev);
	if (ret) {
		ESP_LOGI(TAG,"Error %d during preparation of the sensor, exiting program!\n",
			   ret);
		handle_error(device);
	}

	/*
	* One-time initialization of the algorithm. Handle passed to calculation
	* function.
	*/
	ret = init_oaq_2nd_gen(&algo_handle);
	if (ret) {
		ESP_LOGI(TAG,"Error %d during initializing algorithm, exiting program!\n",
				ret);
		handle_error(device);
	}

	// Initialize component structure.
	device->EPA_AQI = 0;
	device->FAST_AQI = 0;
	device->O3_conc_ppb = 0;
	device->status = SENSOR_WARMING_UP;
	return;
}
