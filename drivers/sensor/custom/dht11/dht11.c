/* File : dht11.c */
#define DT_DRV_COMPAT custom_dht11

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "dht11.h"

LOG_MODULE_REGISTER(dht11, CONFIG_SENSOR_LOG_LEVEL);
/* Placeholder for DHT11 driver implementation */

/* Initialize sensor */
static int dht11_init(const struct device *dev){
    const struct dht11_config *config= dev->config;
    struct dht_data *data = dev->data;
    int ret;

    /* Check if gpio device is ready*/
    if(!gpio_is_ready_dt(&config->data_gpio)){
        LOG_ERR("DHT11 data GPIO device not ready");
        return -ENODEV;
    }

    /* Check power gpio idf sepcified*/
    if(config->power_gpio.port != NULL){
        if(!gpio_is_ready_dt(&config->power_gpio)){
            LOG_ERR("DHT11 power gpio device not reday");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_HIGH);
        if(ret < 0){
            LOG_ERR("Failed to configure DHT11 power GPIO");
            return ret;
        }
        k_sleep(K_MSEC(1000)); /* Wait for sensor to power up */
    }

    /* Configure Data GPIO */
    ret = gpio_pin_configure_dt(&config->data_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if(ret < 0){
        LOG_ERR("Failed to configure data gpio");
        return ret;
    } 

    data->last_read_time = sys_timepoint_calc(K_NO_WAIT);

    LOG_INF("DHT11 sensor initialized");
    

    return 0;
}

/* Sensor channel get */
static int dht11_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val){
    struct dht11_data *data = dev->data;

    switch(chan){
        case SENSOR_CHAN_AMBIENT_TEMP:
            val->val1 = data->temperature / 10;
            val->val2 = (data->temperature % 10) * 100000;
            break;
        case SENSOR_CHAN_HUMIDITY:
            val->val1 = data->humidity / 10;
            val->val2 = (data->humidity % 10) * 100000;
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api dht11_driver_api = {
    .channel_get = dht11_channel_get,
};