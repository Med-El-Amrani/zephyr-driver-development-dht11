/* File : dht11.h */
#ifndef ZEPHYR_DRIVERS_SENSOR_DHT11_H_
#define ZEPHYR_DRIVERS_SENSOR_DHT11_H_
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>


struct dht11_data {
    int16_t temperature; /* in 0.1 degree Celsius */
    int16_t humidity;    /* in 0.1 %RH */
    k_timepoint_t last_read_time; /* Enforce read-interval at runtime*/
};

struct dht11_config
{
    struct gpio_dt_spec data_gpio;
    struct gpio_dt_spec power_gpio;
    uint32_t read_interval_ms;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_DHT11_H_ */
