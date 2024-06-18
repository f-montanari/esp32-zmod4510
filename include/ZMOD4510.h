
#ifndef ZMOD4510_H_
#define ZMOD4510_H_

typedef enum{
	SENSOR_READY = 0,
	SENSOR_DATA_READY,
	SENSOR_MEASURING,
	SENSOR_WARMING_UP,
	SENSOR_ERROR
}zmod4510_status_t;

typedef struct {
    float rmox[8]; /**< MOx resistance. */
    float Rmox_smooth;
    float O3_conc_1hr;
    float O3_conc_8hr;
    float O3_conc_ppb; /**< O3_conc_ppb stands for the ozone concentration in part-per-billion */
    uint16_t pid; /**< product id of the sensor */
    uint16_t
        FAST_AQI; /**< FAST_AQI stands for a 1-minute average of the Air Quality Index according to the EPA standard based on ozone */
    uint16_t
        EPA_AQI; /**< EPA_AQI stands for the Air Quality Index according to the EPA standard based on ozone. */
    zmod4510_status_t status;
}zmod4510_dev_t;

void zmod4510_init_sensor(zmod4510_dev_t*);
void zmod4510_update_sensor_data(zmod4510_dev_t*, float, float);
void zmod4510_start_irq_measurement(zmod4510_dev_t*, float, float);
void zmod4510_force_read();
#endif
